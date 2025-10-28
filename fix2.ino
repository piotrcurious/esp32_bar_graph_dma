/* esp32_tft_dma_bar_plot_descriptor_chain.ino - FIXED

Objective: implement high-throughput SPI DMA by creating a chained-like transaction 
pipeline where command and pixel data for each bar are queued as back-to-back SPI 
transactions with proper DC line handling.

IMPORTANT FIXES:
- Split commands and parameters into separate transactions (DC low for cmd, high for params)
- Fixed transaction length calculations
- Added critical sections for pool management
- Improved error handling and race condition prevention
- Fixed pixel data length alignment
*/

#include <Arduino.h>
#include <TFT_eSPI.h>
#include "esp_heap_caps.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

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

// Pool size: we need 6 transactions per bar (CASET cmd+params, RASET cmd+params, RAMWR cmd, DATA)
#define TRANS_POOL_SIZE 24

// Maximum bytes for command buffer
#define CMD_BUF_LEN 16

// ---------- GLOBALS ----------
TFT_eSPI tft = TFT_eSPI();

static uint16_t* barSource = nullptr;
static size_t barSourceLen = BAR_SOURCE_LEN;

volatile bool plotInProgress = false;
volatile uint8_t plotProgress = 0;

static const uint16_t* plotDataArray = nullptr;
static size_t plotDataCount = 0;
static size_t currentBarIndex = 0;

static TaskHandle_t plotTaskHandle = NULL;
static spi_device_handle_t spiHandle = NULL;
static portMUX_TYPE poolMux = portMUX_INITIALIZER_UNLOCKED;

// Transaction pool structure
struct TransEntry {
    spi_transaction_t trans;
    volatile bool in_use;
    uint8_t cmdbuf[CMD_BUF_LEN]; // DMA-capable command buffer
};
static TransEntry* transPool = nullptr;

// Helper: pack fields into trans.user
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
        portENTER_CRITICAL_ISR(&poolMux);
        transPool[idx].in_use = false;
        portEXIT_CRITICAL_ISR(&poolMux);
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
static inline size_t valueToSourceOffset(uint16_t value) {
    if (value > MAX_BAR_LENGTH_PIXELS) value = MAX_BAR_LENGTH_PIXELS;
    size_t half = barSourceLen / 2;
    if (value >= half) return 0;
    return half - value;
}

// Acquire 'n' free entries from pool with proper locking
static int acquire_free_entries(int need, int* out_idxs) {
    int found = 0;
    portENTER_CRITICAL(&poolMux);
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
        portEXIT_CRITICAL(&poolMux);
        return -1;
    }
    portEXIT_CRITICAL(&poolMux);
    return 0;
}

// ---------- SETUP ----------
void setup() {
    Serial.begin(115200);
    delay(50);
    Serial.println("Starting TFT DMA Bar Plot...");

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

    // Get spi device handle from TFT_eSPI
    // NOTE: This requires TFT_eSPI to be patched with getSpiDeviceHandle() method
    // Add to TFT_eSPI.h: spi_device_handle_t getSpiDeviceHandle() { return _spi; }
    #ifdef TFT_ESPI_HAS_SPI_HANDLE
        spiHandle = tft.getSpiDeviceHandle();
    #else
        Serial.println("ERROR: TFT_eSPI must expose spi handle");
        Serial.println("Add this to TFT_eSPI.h:");
        Serial.println("  spi_device_handle_t getSpiDeviceHandle() { return _spi; }");
        while (1) delay(1000);
    #endif

    if (!spiHandle) {
        Serial.println("FATAL: Could not get SPI handle");
        while (1) delay(1000);
    }

    // Register callbacks
    esp_err_t r1 = spi_device_set_pre_cb(spiHandle, spi_pre_cb);
    esp_err_t r2 = spi_device_set_post_cb(spiHandle, spi_post_cb);
    if (r1 != ESP_OK || r2 != ESP_OK) {
        Serial.printf("Failed to set callbacks: pre=%d post=%d\n", r1, r2);
        while (1) delay(1000);
    }

    // Create plot task
    xTaskCreatePinnedToCore(plotTask, "plotTask", 8192, NULL, 2, &plotTaskHandle, 1);
    
    if (!plotTaskHandle) {
        Serial.println("FATAL: Could not create plot task");
        while (1) delay(1000);
    }

    // Demo data
    static uint16_t demo[BAR_COUNT];
    for (size_t i = 0; i < BAR_COUNT; ++i) {
        demo[i] = (i * (MAX_BAR_LENGTH_PIXELS / BAR_COUNT)) % (MAX_BAR_LENGTH_PIXELS + 1);
    }
    
    delay(100);
    Serial.println("Starting plot...");
    startBarPlot(demo, BAR_COUNT, nullptr);
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

// ---------- PLOT TASK ----------
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

            // We need 6 transactions: CASET(cmd), CASET(params), RASET(cmd), RASET(params), RAMWR(cmd), DATA
            int idxs[6];
            while (acquire_free_entries(6, idxs) != 0) {
                ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
                if (!plotInProgress) break;
            }
            if (!plotInProgress) break;

            // Transaction 0: CASET command byte (DC low)
            {
                TransEntry* e = &transPool[idxs[0]];
                spi_transaction_t* t = &e->trans;
                memset(t, 0, sizeof(spi_transaction_t));
                e->cmdbuf[0] = 0x2A;
                t->length = 8;
                t->tx_buffer = e->cmdbuf;
                set_trans_user(t, idxs[0], false);
                t->flags = 0;
                spi_device_queue_trans(spiHandle, t, portMAX_DELAY);
            }

            // Transaction 1: CASET parameters (DC high)
            {
                TransEntry* e = &transPool[idxs[1]];
                spi_transaction_t* t = &e->trans;
                memset(t, 0, sizeof(spi_transaction_t));
                e->cmdbuf[0] = (uint8_t)(dx >> 8);
                e->cmdbuf[1] = (uint8_t)(dx & 0xFF);
                e->cmdbuf[2] = (uint8_t)((dx + w - 1) >> 8);
                e->cmdbuf[3] = (uint8_t)((dx + w - 1) & 0xFF);
                t->length = 32;
                t->tx_buffer = e->cmdbuf;
                set_trans_user(t, idxs[1], true);
                spi_device_queue_trans(spiHandle, t, portMAX_DELAY);
            }

            // Transaction 2: RASET command byte (DC low)
            {
                TransEntry* e = &transPool[idxs[2]];
                spi_transaction_t* t = &e->trans;
                memset(t, 0, sizeof(spi_transaction_t));
                e->cmdbuf[0] = 0x2B;
                t->length = 8;
                t->tx_buffer = e->cmdbuf;
                set_trans_user(t, idxs[2], false);
                spi_device_queue_trans(spiHandle, t, portMAX_DELAY);
            }

            // Transaction 3: RASET parameters (DC high)
            {
                TransEntry* e = &transPool[idxs[3]];
                spi_transaction_t* t = &e->trans;
                memset(t, 0, sizeof(spi_transaction_t));
                e->cmdbuf[0] = (uint8_t)(dy >> 8);
                e->cmdbuf[1] = (uint8_t)(dy & 0xFF);
                e->cmdbuf[2] = (uint8_t)((dy + h - 1) >> 8);
                e->cmdbuf[3] = (uint8_t)((dy + h - 1) & 0xFF);
                t->length = 32;
                t->tx_buffer = e->cmdbuf;
                set_trans_user(t, idxs[3], true);
                spi_device_queue_trans(spiHandle, t, portMAX_DELAY);
            }

            // Transaction 4: RAMWR command (DC low)
            {
                TransEntry* e = &transPool[idxs[4]];
                spi_transaction_t* t = &e->trans;
                memset(t, 0, sizeof(spi_transaction_t));
                e->cmdbuf[0] = 0x2C;
                t->length = 8;
                t->tx_buffer = e->cmdbuf;
                set_trans_user(t, idxs[4], false);
                spi_device_queue_trans(spiHandle, t, portMAX_DELAY);
            }

            // Transaction 5: Pixel data (DC high)
            {
                TransEntry* e = &transPool[idxs[5]];
                spi_transaction_t* t = &e->trans;
                memset(t, 0, sizeof(spi_transaction_t));
                size_t bytes = w * h * 2;
                t->length = bytes * 8; // length is in BITS
                t->tx_buffer = pixels;
                set_trans_user(t, idxs[5], true);
                spi_device_queue_trans(spiHandle, t, portMAX_DELAY);
            }

            // Wait for all 6 transactions to complete
            for (int i = 0; i < 6; ++i) {
                ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            }

            currentBarIndex++;
            plotProgress = (uint8_t)((currentBarIndex * 100) / (plotDataCount ? plotDataCount : 1));
        }

        if (currentBarIndex >= plotDataCount) {
            plotInProgress = false;
            plotProgress = 100;
            Serial.println("Plot finished!");
        }
    }
}

void loop() {
    static uint32_t last = 0;
    if (millis() - last > 500) {
        last = millis();
        Serial.printf("Progress: %u%%\n", plotProgress);
    }
}

/*
REQUIRED TFT_eSPI PATCH:
Add to TFT_eSPI.h in the public section of TFT_eSPI class:

    spi_device_handle_t getSpiDeviceHandle() { 
        #ifdef ESP32
            return _spi; 
        #else
            return NULL;
        #endif
    }

Then add #define TFT_ESPI_HAS_SPI_HANDLE at the top of this sketch.
*/
