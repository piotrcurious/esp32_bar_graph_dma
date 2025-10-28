Looking at your code, I've identified several issues that need fixing. Here's the corrected version:

```cpp
/* esp32_tft_dma_bar_plot_descriptor_chain.ino

Objective: implement highest-throughput SPI DMA by creating a chained-like transaction 
pipeline where command and pixel data for each bar are queued as back-to-back SPI 
transactions and the SPI driver toggles DC automatically using a lightweight pre-callback.
*/

#include <Arduino.h>
#include <TFT_eSPI.h>
#include "esp_heap_caps.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ---------- CONFIG ------------
#define TFT_ROTATION 1
#define BAR_COUNT 60
#define BAR_PIXEL_WIDTH 4
#define BAR_PIXEL_HEIGHT 6
#define PLOT_X 8
#define PLOT_Y 40
#define PLOT_SPACING 1
#define MAX_BAR_LENGTH_PIXELS 200

#define COLOR_FILL 0x07E0
#define COLOR_BG   0xFFFF

#define BAR_SOURCE_LEN (MAX_BAR_LENGTH_PIXELS + BAR_PIXEL_WIDTH)

// Pool size must be >= number of in-flight transactions (we use 4 per bar pipeline)
#define TRANS_POOL_SIZE 32

// Maximum bytes for command buffer (CASET/RASET/RAMWR params etc.)
#define CMD_BUF_LEN 16

// ---------- GLOBALS ----------
TFT_eSPI tft = TFT_eSPI();

static uint16_t* barSource = nullptr; // DMA-capable pixel buffer
static size_t barSourceLen = BAR_SOURCE_LEN;

volatile bool plotInProgress = false;
volatile uint8_t plotProgress = 0;

static const uint16_t* plotDataArray = nullptr;
static size_t plotDataCount = 0;
static size_t currentBarIndex = 0;

static TaskHandle_t plotTaskHandle = NULL;
static spi_device_handle_t spiHandle = NULL;

// Transaction pool structure
struct TransEntry {
    spi_transaction_t trans;
    volatile bool in_use;
    uint8_t cmdbuf[CMD_BUF_LEN] __attribute__((aligned(4))); // DMA-aligned command buffer
};
static TransEntry* transPool = nullptr;

// Forward declaration
void plotTask(void* pv);

// Helper: pack fields into trans.user: lower 16 bits = pool index, bit 31 = DC level
static inline void set_trans_user(spi_transaction_t* t, uint16_t idx, bool dcHigh) {
    uintptr_t v = ((uintptr_t)idx & 0xFFFF) | ((uintptr_t)(dcHigh ? 1 : 0) << 31);
    t->user = (void*)v;
}

static inline uint16_t get_trans_idx(spi_transaction_t* t) {
    return (uint16_t)((uintptr_t)t->user & 0xFFFF);
}

static inline bool get_trans_dc(spi_transaction_t* t) {
    return (((uintptr_t)t->user >> 31) & 1) != 0;
}

// Pre-callback: executed in ISR context just before SPI transaction begins
static void IRAM_ATTR spi_pre_cb(spi_transaction_t* trans) {
    bool dcHigh = get_trans_dc(trans);
    gpio_set_level((gpio_num_t)TFT_DC, dcHigh ? 1 : 0);
}

// Post-callback: mark pool entry free and notify plot task
static void IRAM_ATTR spi_post_cb(spi_transaction_t* trans) {
    uint16_t idx = get_trans_idx(trans);
    if (idx < TRANS_POOL_SIZE) {
        transPool[idx].in_use = false;
    }
    BaseType_t xHigher = pdFALSE;
    if (plotTaskHandle) {
        vTaskNotifyGiveFromISR(plotTaskHandle, &xHigher);
    }
    if (xHigher == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

// ---------- UTIL ----------
static inline size_t round_up_words(size_t bytes) {
    return (bytes + 3) & ~3;
}

// Convert logical bar value to offset in barSource
static inline size_t valueToSourceOffset(uint16_t value) {
    if (value > MAX_BAR_LENGTH_PIXELS) value = MAX_BAR_LENGTH_PIXELS;
    size_t half = barSourceLen / 2;
    if (value >= half) return 0;
    return half - value;
}

// Acquire 'n' free entries from pool; returns 0 on success, -1 on failure
static int acquire_free_entries(int need, int* out_idxs) {
    int found = 0;
    for (int i = 0; i < TRANS_POOL_SIZE && found < need; ++i) {
        if (!transPool[i].in_use) {
            transPool[i].in_use = true;
            out_idxs[found++] = i;
        }
    }
    if (found < need) {
        // rollback
        for (int i = 0; i < found; ++i) {
            transPool[out_idxs[i]].in_use = false;
        }
        return -1;
    }
    return 0;
}

// ---------- PLOT START ----------
void startBarPlot(const uint16_t* dataArray, size_t count, uint16_t* optionalBarSource) {
    if (plotInProgress) {
        Serial.println("Plot already running");
        return;
    }
    plotDataArray = dataArray;
    plotDataCount = count;
    currentBarIndex = 0;
    plotProgress = 0;
    plotInProgress = true;
    if (optionalBarSource) {
        barSource = optionalBarSource;
    }
    
    int totalBars = (int)plotDataCount;
    int totalPlotWidth = totalBars * BAR_PIXEL_WIDTH + (totalBars - 1) * PLOT_SPACING;
    tft.fillRect(PLOT_X, PLOT_Y, totalPlotWidth, BAR_PIXEL_HEIGHT, COLOR_BG);
    
    xTaskNotifyGive(plotTaskHandle);
}

// ---------- PLOT TASK (chain-like queued transactions) ----------
void plotTask(void* pv) {
    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (!plotInProgress) continue;

        while (plotInProgress && currentBarIndex < plotDataCount) {
            uint16_t val = plotDataArray[currentBarIndex];
            if (val > MAX_BAR_LENGTH_PIXELS) val = MAX_BAR_LENGTH_PIXELS;
            size_t offset = valueToSourceOffset(val);
            uint16_t* pixels = barSource + offset;

            int w = BAR_PIXEL_WIDTH;
            int h = BAR_PIXEL_HEIGHT;
            int dx = PLOT_X + (int)currentBarIndex * (BAR_PIXEL_WIDTH + PLOT_SPACING);
            int dy = PLOT_Y;

            // Acquire 4 pool entries (3 command entries + 1 data entry)
            int idxs[4];
            while (acquire_free_entries(4, idxs) != 0) {
                ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
                if (!plotInProgress) break;
            }
            if (!plotInProgress) break;

            // Prepare CASET (Column Address Set)
            {
                TransEntry* e = &transPool[idxs[0]];
                spi_transaction_t* t = &e->trans;
                memset(t, 0, sizeof(spi_transaction_t));
                memset(e->cmdbuf, 0, CMD_BUF_LEN);
                
                e->cmdbuf[0] = 0x2A; // CASET command
                e->cmdbuf[1] = (uint8_t)(dx >> 8);
                e->cmdbuf[2] = (uint8_t)(dx & 0xFF);
                e->cmdbuf[3] = (uint8_t)((dx + w - 1) >> 8);
                e->cmdbuf[4] = (uint8_t)((dx + w - 1) & 0xFF);

                t->length = 8 * 5; // bits
                t->tx_buffer = e->cmdbuf;
                t->flags = 0;
                set_trans_user(t, idxs[0], false); // DC low
                
                esp_err_t ret = spi_device_queue_trans(spiHandle, t, portMAX_DELAY);
                if (ret != ESP_OK) {
                    Serial.printf("Queue CASET failed: %d\n", ret);
                }
            }

            // Prepare RASET (Row Address Set)
            {
                TransEntry* e = &transPool[idxs[1]];
                spi_transaction_t* t = &e->trans;
                memset(t, 0, sizeof(spi_transaction_t));
                memset(e->cmdbuf, 0, CMD_BUF_LEN);
                
                e->cmdbuf[0] = 0x2B; // RASET
                e->cmdbuf[1] = (uint8_t)(dy >> 8);
                e->cmdbuf[2] = (uint8_t)(dy & 0xFF);
                e->cmdbuf[3] = (uint8_t)((dy + h - 1) >> 8);
                e->cmdbuf[4] = (uint8_t)((dy + h - 1) & 0xFF);

                t->length = 8 * 5;
                t->tx_buffer = e->cmdbuf;
                t->flags = 0;
                set_trans_user(t, idxs[1], false);
                
                esp_err_t ret = spi_device_queue_trans(spiHandle, t, portMAX_DELAY);
                if (ret != ESP_OK) {
                    Serial.printf("Queue RASET failed: %d\n", ret);
                }
            }

            // Prepare RAMWR command (single byte)
            {
                TransEntry* e = &transPool[idxs[2]];
                spi_transaction_t* t = &e->trans;
                memset(t, 0, sizeof(spi_transaction_t));
                memset(e->cmdbuf, 0, CMD_BUF_LEN);
                
                e->cmdbuf[0] = 0x2C; // RAMWR
                
                t->length = 8 * 1;
                t->tx_buffer = e->cmdbuf;
                t->flags = 0;
                set_trans_user(t, idxs[2], false);
                
                esp_err_t ret = spi_device_queue_trans(spiHandle, t, portMAX_DELAY);
                if (ret != ESP_OK) {
                    Serial.printf("Queue RAMWR failed: %d\n", ret);
                }
            }

            // Prepare PIXEL DATA transaction
            {
                TransEntry* e = &transPool[idxs[3]];
                spi_transaction_t* t = &e->trans;
                memset(t, 0, sizeof(spi_transaction_t));
                
                size_t bytes = w * h * 2;
                t->length = bytes * 8; // bits (not word-aligned here, driver handles it)
                t->tx_buffer = pixels;
                t->flags = 0;
                set_trans_user(t, idxs[3], true); // DC high for pixel data
                
                esp_err_t ret = spi_device_queue_trans(spiHandle, t, portMAX_DELAY);
                if (ret != ESP_OK) {
                    Serial.printf("Queue pixel data failed: %d\n", ret);
                }
            }

            // Wait for all 4 transactions to complete
            for (int i = 0; i < 4; ++i) {
                ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            }

            currentBarIndex++;
            plotProgress = (uint8_t)((currentBarIndex * 100) / (plotDataCount ? plotDataCount : 1));
        }

        if (currentBarIndex >= plotDataCount) {
            plotInProgress = false;
            plotProgress = 100;
            Serial.println("Plot finished (descriptor-like chaining via queued transactions)");
        }
    }
}

// ---------- SETUP ----------
void setup() {
    Serial.begin(115200);
    delay(50);

    tft.init();
    tft.setRotation(TFT_ROTATION);
    tft.fillScreen(TFT_BLACK);

    // Allocate DMA-capable bar source
    barSource = (uint16_t*) heap_caps_malloc(barSourceLen * sizeof(uint16_t), MALLOC_CAP_DMA);
    if (!barSource) {
        Serial.println("FATAL: barSource alloc fail");
        while (1) delay(1000);
    }
    size_t half = barSourceLen / 2;
    for (size_t i = 0; i < barSourceLen; ++i) {
        barSource[i] = (i < half) ? COLOR_FILL : COLOR_BG;
    }

    // Allocate trans pool in DMA memory
    transPool = (TransEntry*) heap_caps_malloc(sizeof(TransEntry) * TRANS_POOL_SIZE, MALLOC_CAP_DMA);
    if (!transPool) {
        Serial.println("FATAL: transPool alloc fail");
        while (1) delay(1000);
    }
    for (int i = 0; i < TRANS_POOL_SIZE; ++i) {
        memset(&transPool[i].trans, 0, sizeof(spi_transaction_t));
        transPool[i].in_use = false;
        memset(transPool[i].cmdbuf, 0, CMD_BUF_LEN);
    }

    // Get spi device handle from TFT_eSPI (requires library patch)
    // WORKAROUND: If getSpiDeviceHandle() doesn't exist, you'll need to patch TFT_eSPI
    // or use tft.getSetup() to extract SPI parameters
    #ifdef TFT_eSPI_VERSION
    // Assuming patch is applied
    // spiHandle = tft.getSpiDeviceHandle();
    
    // TEMPORARY: If patch not available, this will fail at compile time
    // You need to add this method to TFT_eSPI.h/.cpp:
    // spi_device_handle_t TFT_eSPI::getSpiDeviceHandle() { return spi.handle(); }
    #endif
    
    // For demonstration, check if handle exists
    // NOTE: This line assumes you've patched TFT_eSPI. Comment out if not available.
    // spiHandle = tft.getSpiDeviceHandle();
    
    // WORKAROUND if getSpiDeviceHandle() not available:
    // You'll need to create your own SPI device on the same bus or extract from TFT_eSPI internals
    Serial.println("WARNING: getSpiDeviceHandle() requires TFT_eSPI patch");
    Serial.println("Please add to TFT_eSPI: spi_device_handle_t getSpiDeviceHandle() { return spi.handle(); }");
    
    if (!spiHandle) {
        Serial.println("FATAL: Cannot get SPI handle. Apply TFT_eSPI patch.");
        while (1) delay(1000);
    }

    // Register our lightweight pre & post callbacks
    esp_err_t r1 = spi_device_set_pre_cb(spiHandle, spi_pre_cb);
    esp_err_t r2 = spi_device_set_post_cb(spiHandle, spi_post_cb);
    if (r1 != ESP_OK || r2 != ESP_OK) {
        Serial.printf("Failed to set pre/post cb: %d %d\n", r1, r2);
        while (1) delay(1000);
    }

    // Create plot task
    xTaskCreatePinnedToCore(plotTask, "plotTask", 4096, NULL, 1, &plotTaskHandle, 1);

    // Demo data
    static uint16_t demo[BAR_COUNT];
    for (size_t i = 0; i < BAR_COUNT; ++i) {
        demo[i] = (i * (MAX_BAR_LENGTH_PIXELS / BAR_COUNT)) % (MAX_BAR_LENGTH_PIXELS + 1);
    }
    
    delay(100); // Let task start
    startBarPlot(demo, BAR_COUNT, nullptr);
}

void loop() {
    static uint32_t last = 0;
    if (millis() - last > 500) {
        last = millis();
        Serial.printf("plotInProgress=%d progress=%u%%\n", plotInProgress ? 1 : 0, plotProgress);
    }
}
```

## Key Fixes:

1. **Missing FreeRTOS headers** - Added `#include "freertos/FreeRTOS.h"` and `#include "freertos/task.h"`

2. **Forward declaration** - Added `void plotTask(void* pv);` before it's referenced

3. **Memory alignment** - Added `__attribute__((aligned(4)))` to cmdbuf for proper DMA alignment

4. **Transaction length** - Fixed pixel data transaction to use `bytes * 8` instead of `aligned * 8` (driver handles alignment)

5. **Error checking** - Added esp_err_t checks for queue operations

6. **Printf formatting** - Added missing `\n` characters

7. **getSpiDeviceHandle() issue** - Added warnings and comments about the required TFT_eSPI patch, which is critical for this code to work

8. **Task initialization** - Added small delay before starting plot to ensure task is ready

The most critical issue is that **you must patch TFT_eSPI** to expose the SPI handle. Without this, the code cannot work. The patch needs to add this method to TFT_eSPI class:

```cpp
spi_device_handle_t getSpiDeviceHandle() { return spi.handle(); }
```
