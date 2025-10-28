/* esp32_tft_dma_bar_plot.ino

Preallocated spi_transaction_t pool (no malloc/free during plotting).

Integrated callback flow into TFT_eSPI by using an exposed spi_device_handle_t from the TFT_eSPI instance (requires a tiny patch to the TFT_eSPI library).


Overview of changes:

1. Instead of allocating a spi_transaction_t for each bar we pre-allocate a small pool of transactions in DMA-capable memory. Each pool entry contains a spi_transaction_t and an in_use flag. This avoids dynamic allocation in the hot path and is deterministic.


2. We integrate with TFT_eSPI by retrieving its internal spi_device_handle_t and installing our post-transfer callback with spi_device_set_post_cb(). This avoids creating a separate SPI device and eliminates bus conflicts.



Required tiny patch to TFT_eSPI library (apply before compiling):

In TFT_eSPI.h (add near the top of the class declaration):

// expose underlying spi device handle for advanced direct-DMA usage
public:
  spi_device_handle_t getSpiDeviceHandle();

// add a protected member to hold it (if not already tracked):
protected:
  spi_device_handle_t _spiHandle = nullptr;

In TFT_eSPI.cpp, after spi_bus_add_device(...) stores the handle locally, save it into the member. For example, if the library currently does:

spi_device_handle_t spi;
spi_bus_add_device(..., &spi);

change to:

spi_device_handle_t spi;
spi_bus_add_device(..., &spi);
_spiHandle = spi;

and implement the getter in TFT_eSPI.cpp:

spi_device_handle_t TFT_eSPI::getSpiDeviceHandle() {
  return _spiHandle;
}

NOTE: Different TFT_eSPI versions may initialize the SPI handle elsewhere. The important point is to expose the existing handle rather than creating a new one.


---

The sketch below relies on that patched TFT_eSPI and then:

calls tft.getSpiDeviceHandle()

calls spi_device_set_post_cb(handle, spi_post_cb) to register our ISR callback

uses a small pool of preallocated DMA-capable spi_transaction_t structures



---

*/

#include <Arduino.h> #include <TFT_eSPI.h> #include "esp_heap_caps.h" #include "driver/spi_master.h" #include "driver/gpio.h"

// ---------- USER CONFIG ---------- #define TFT_ROTATION 1 #define BAR_COUNT 60 #define BAR_PIXEL_WIDTH 4 #define BAR_PIXEL_HEIGHT 6 #define PLOT_X 8 #define PLOT_Y 40 #define PLOT_SPACING 1 #define MAX_BAR_LENGTH_PIXELS 200

#define COLOR_FILL 0x07E0 #define COLOR_BG   0xFFFF

#define BAR_SOURCE_LEN (MAX_BAR_LENGTH_PIXELS + BAR_PIXEL_WIDTH)

// Pool size: must be <= spi_device_interface_config_t.queue_size used by TFT_eSPI #define TRANS_POOL_SIZE 8

// ---------- GLOBALS ---------- TFT_eSPI tft = TFT_eSPI();

static uint16_t* barSource = nullptr; static size_t barSourceLen = BAR_SOURCE_LEN;

volatile bool plotInProgress = false; volatile uint8_t plotProgress = 0;

static const uint16_t* plotDataArray = nullptr; static size_t plotDataCount = 0; static size_t currentBarIndex = 0;

static TaskHandle_t plotTaskHandle = NULL;

// Transaction pool allocated in DMA-capable memory struct TransEntry { spi_transaction_t trans;    // must be the first member if you want pointer equivalence volatile bool in_use;       // flag }; static TransEntry* transPool = nullptr; // allocated via heap_caps_malloc(MALLOC_CAP_DMA)

// SPI device handle (retrieved from TFT_eSPI after patching the library) static spi_device_handle_t spiHandle = NULL;

// Forward declarations void createBarSource(uint16_t colorA, uint16_t colorB); void startBarPlot(const uint16_t* dataArray, size_t count, uint16_t* optionalBarSource = nullptr); void plotTask(void* pvParameters); static inline size_t valueToSourceOffset(uint16_t value);

// post-callback for SPI transactions (called in ISR context by SPI driver) static void IRAM_ATTR spi_post_cb(spi_transaction_t* trans) { // The transaction's user field stores the pool index (as uintptr_t) uintptr_t idx = (uintptr_t) trans->user; // Mark the pool entry as free if (idx < TRANS_POOL_SIZE) { transPool[idx].in_use = false; } // Notify plot task that this transfer finished BaseType_t xHigherPriorityTaskWoken = pdFALSE; if (plotTaskHandle) { vTaskNotifyGiveFromISR(plotTaskHandle, &xHigherPriorityTaskWoken); if (xHigherPriorityTaskWoken) portYIELD_FROM_ISR(); } }

// ---------- SETUP ---------- void setup() { Serial.begin(115200); delay(50);

tft.init(); tft.setRotation(TFT_ROTATION); tft.fillScreen(TFT_BLACK);

// allocate DMA-capable source bar buffer barSource = (uint16_t*) heap_caps_malloc(barSourceLen * sizeof(uint16_t), MALLOC_CAP_DMA); if (!barSource) { Serial.println("FATAL: failed to allocate barSource in DMA-capable memory"); while (1) delay(1000); } createBarSource(COLOR_FILL, COLOR_BG);

// Allocate transaction pool in DMA-capable memory transPool = (TransEntry*) heap_caps_malloc(sizeof(TransEntry) * TRANS_POOL_SIZE, MALLOC_CAP_DMA); if (!transPool) { Serial.println("FATAL: failed to allocate transPool in DMA-capable memory"); while (1) delay(1000); } // Initialize pool entries for (int i = 0; i < TRANS_POOL_SIZE; ++i) { memset(&transPool[i].trans, 0, sizeof(spi_transaction_t)); transPool[i].in_use = false; // store pool index in user field when we use it transPool[i].trans.user = nullptr; }

// Retrieve spi device handle from TFT_eSPI (requires the library patch described above) spi_device_handle_t libHandle = tft.getSpiDeviceHandle(); if (!libHandle) { Serial.println("FATAL: TFT_eSPI does not expose spi handle. Apply the small patch to TFT_eSPI."); while (1) delay(1000); } spiHandle = libHandle;

// Install our post-callback on the existing TFT_eSPI spi device handle. // This will call our spi_post_cb whenever a transaction finishes. esp_err_t r = spi_device_set_post_cb(spiHandle, spi_post_cb); if (r != ESP_OK) { Serial.printf("Failed to set post callback: %d ", r); while (1) delay(1000); }

// create the plot task (blocked until startBarPlot triggers it) if (plotTaskHandle == NULL) { xTaskCreatePinnedToCore(plotTask, "plotTask", 4096, NULL, 1, &plotTaskHandle, 1); }

// demo data static uint16_t demo[BAR_COUNT]; for (size_t i=0;i<BAR_COUNT;i++) demo[i] = (i * (MAX_BAR_LENGTH_PIXELS / BAR_COUNT)) % (MAX_BAR_LENGTH_PIXELS+1); startBarPlot(demo, BAR_COUNT, nullptr); }

// ---------- HELPERS ---------- void createBarSource(uint16_t colorA, uint16_t colorB) { size_t half = barSourceLen / 2; for (size_t i=0;i<barSourceLen;i++) barSource[i] = (i < half) ? colorA : colorB; }

static inline size_t valueToSourceOffset(uint16_t value) { if (value > MAX_BAR_LENGTH_PIXELS) value = MAX_BAR_LENGTH_PIXELS; size_t half = barSourceLen / 2; if (value >= half) return 0; return (half - value); }

// ---------- PLOT STARTER ---------- void startBarPlot(const uint16_t* dataArray, size_t count, uint16_t* optionalBarSource) { if (plotInProgress) { Serial.println("Plot already running"); return; } plotDataArray = dataArray; plotDataCount = count; currentBarIndex = 0; plotProgress = 0; plotInProgress = true; if (optionalBarSource) barSource = optionalBarSource;

// Clear plot area once int totalBars = (int)plotDataCount; int totalPlotWidth = totalBars * BAR_PIXEL_WIDTH + (totalBars - 1) * PLOT_SPACING; tft.fillRect(PLOT_X, PLOT_Y, totalPlotWidth, BAR_PIXEL_HEIGHT, COLOR_BG);

// Kick the plot task to start processing the first bar xTaskNotifyGive(plotTaskHandle); }

// ---------- PLOT TASK (event-driven, uses preallocated pool) ---------- void plotTask(void* pvParameters) { while (true) { // Wait for a notification to start/continue ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

if (!plotInProgress) continue;

while (plotInProgress && currentBarIndex < plotDataCount) {
  uint16_t value = plotDataArray[currentBarIndex];
  if (value > MAX_BAR_LENGTH_PIXELS) value = MAX_BAR_LENGTH_PIXELS;
  size_t offset = valueToSourceOffset(value);
  uint16_t* srcPtr = barSource + offset;

  int w = BAR_PIXEL_WIDTH;
  int h = BAR_PIXEL_HEIGHT;
  int dx = PLOT_X + (int)currentBarIndex * (BAR_PIXEL_WIDTH + PLOT_SPACING);
  int dy = PLOT_Y;

  // Build CASET/RASET and RAMWR commands using TFT_eSPI helper functions
  // We send the commands via TFT_eSPI (blocking) because they are short.
  // Then we queue the pixel data as a DMA transaction using the TFT_eSPI spi handle.

  // CASET
  tft.writecommand(0x2A);
  uint8_t casetbuf[4] = { (uint8_t)(dx>>8), (uint8_t)(dx&0xFF), (uint8_t)((dx + w - 1)>>8), (uint8_t)((dx + w - 1)&0xFF) };
  tft.writedata(casetbuf, 4);

  // RASET
  tft.writecommand(0x2B);
  uint8_t rasetbuf[4] = { (uint8_t)(dy>>8), (uint8_t)(dy&0xFF), (uint8_t)((dy + h - 1)>>8), (uint8_t)((dy + h - 1)&0xFF) };
  tft.writedata(rasetbuf, 4);

  // RAMWR
  tft.writecommand(0x2C);

  // Acquire a free transaction entry from the pool
  int free_idx = -1;
  for (int i = 0; i < TRANS_POOL_SIZE; ++i) {
    bool expected = false;
    // Try to take ownership atomically
    if (!transPool[i].in_use) {
      // Mark as in_use
      transPool[i].in_use = true;
      free_idx = i;
      break;
    }
  }
  if (free_idx < 0) {
    // No pool entries available - this should not happen if TRANS_POOL_SIZE >= queue_size
    Serial.println("No free transaction entries - increase TRANS_POOL_SIZE");
    // Wait for one to free up
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    continue;
  }

  // Prepare transaction
  spi_transaction_t* t = &transPool[free_idx].trans;
  memset(t, 0, sizeof(spi_transaction_t));
  t->length = w * h * 16; // bits
  t->tx_buffer = srcPtr;   // DMA-capable pixel buffer pointer
  t->user = (void*)(uintptr_t)free_idx;

  // Queue transaction (non-blocking)
  esp_err_t r = spi_device_queue_trans(spiHandle, t, portMAX_DELAY);
  if (r != ESP_OK) {
    Serial.printf("spi_device_queue_trans failed: %d

", r); // release pool entry transPool[free_idx].in_use = false; plotInProgress = false; break; }

// Wait until post-callback notifies that transfer completed
  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

  // Update progress and next bar
  currentBarIndex++;
  plotProgress = (uint8_t)((currentBarIndex * 100) / (plotDataCount ? plotDataCount : 1));
}

if (currentBarIndex >= plotDataCount) {
  plotInProgress = false;
  plotProgress = 100;
  Serial.println("Plot finished (integrated callback + pool)");
}

} }

void loop() { static uint32_t last = 0; if (millis() - last > 500) { last = millis(); Serial.printf("plotInProgress=%d progress=%u%% ", plotInProgress ? 1 : 0, plotProgress); } }

/* Final notes and recommendations:

The patch to TFT_eSPI is small and necessary so we reuse the library's spi device. Without that modification the sketch must create its own SPI device (which risks conflicting with the library), or you must modify the library to allow registration of a post-callback.

TRANS_POOL_SIZE should match or exceed the spi device queue_size used by TFT_eSPI (search for devcfg.queue_size in the library) to avoid pool exhaustion.

For maximum speed pre-allocate a pool of pixel buffers as well if you will generate pixel content dynamically. In our design the pixel buffer is a single reusable "barSource" so that is not necessary.

If you want I can also prepare a patch (diff) targeted at your specific TFT_eSPI version that edits the exact source file locations. Supply the library version or paste the relevant section and I'll craft the diff. */
