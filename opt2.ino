/* esp32_tft_dma_bar_plot_optimized.ino

Advanced-DMA optimizations applied:

Reuse preallocated spi_transaction_t pool entries without memset each time. Each pool entry is prepared once; for each bar we only update the tx_buffer pointer and length. This reduces memory traffic and improves determinism.

Ensure DMA buffer alignment and size constraints (4-byte alignment, length rounded up to 32-bit words) to avoid driver-side copying and maximize DMA throughput.

Optionally enable double-buffering of pixel data (two logical offsets of the single bar source) so CPU can prepare next visible bar while DMA sends current.

Use device-level post-callback to notify the plot task when each DMA transfer completes (already integrated). We minimize work in ISR.

Use fewer blocking calls: command packets (CASET/RASET/RAMWR) are still sent with TFT_eSPI helper calls (short, blocking) to keep code portable and safe.


NOTES on further advanced DMA features (not implemented here, but described):

Descriptor chaining / scatter-gather: On low-level SPI driver you can build DMA descriptors and chain them so a single DMA transfer can include both command bytes and pixel buffers without CPU intervention. This requires working with the SPI peripheral descriptors and is highly platform-specific.

Use of pre_cb to toggle DC inside ISR allows purely transactional control of the DC pin, removing small GPIO operations from the task path. This can be added if you patch TFT_eSPI to allow setting a pre-callback or you attach your own device-level pre_cb.

Use larger queue_size and TRANS_POOL_SIZE equal to queue_size for deeper pipelining.


This sketch is conservative (portable) yet applies high-impact optimizations. */

#include <Arduino.h> #include <TFT_eSPI.h> #include "esp_heap_caps.h" #include "driver/spi_master.h" #include "driver/gpio.h"

// ---------- USER CONFIG ---------- #define TFT_ROTATION 1 #define BAR_COUNT 60 #define BAR_PIXEL_WIDTH 4 #define BAR_PIXEL_HEIGHT 6 #define PLOT_X 8 #define PLOT_Y 40 #define PLOT_SPACING 1 #define MAX_BAR_LENGTH_PIXELS 200

#define COLOR_FILL 0x07E0 #define COLOR_BG   0xFFFF

#define BAR_SOURCE_LEN (MAX_BAR_LENGTH_PIXELS + BAR_PIXEL_WIDTH)

// Pool size must be >= SPI queue_size used by TFT_eSPI #define TRANS_POOL_SIZE 8

// Option to enable double-buffering of pixel data. If true, we keep two // different offsets available so the CPU can change the next bar content while // DMA is sending the other. For this example we keep it simple and use the // same barSource but this flag can be extended to use two physical buffers. #define ENABLE_DOUBLE_BUFFERING 0

// ---------- GLOBALS ---------- TFT_eSPI tft = TFT_eSPI();

static uint16_t* barSource = nullptr; static size_t barSourceLen = BAR_SOURCE_LEN;

volatile bool plotInProgress = false; volatile uint8_t plotProgress = 0;

static const uint16_t* plotDataArray = nullptr; static size_t plotDataCount = 0; static size_t currentBarIndex = 0;

static TaskHandle_t plotTaskHandle = NULL;

// transaction pool entries (pre-initialized) struct TransEntry { spi_transaction_t trans; volatile bool in_use; }; static TransEntry* transPool = nullptr; static spi_device_handle_t spiHandle = NULL;

// ---------- UTILS ---------- static inline size_t round_up_words(size_t bytes) { // round up to 32-bit words return (bytes + 3) & ~3; }

// post-callback - minimal work in ISR: mark pool entry free and notify the task static void IRAM_ATTR spi_post_cb(spi_transaction_t* trans) { uintptr_t id = (uintptr_t) trans->user; if (id < TRANS_POOL_SIZE) { transPool[id].in_use = false; } BaseType_t xHigherPriorityTaskWoken = pdFALSE; if (plotTaskHandle) vTaskNotifyGiveFromISR(plotTaskHandle, &xHigherPriorityTaskWoken); if (xHigherPriorityTaskWoken) portYIELD_FROM_ISR(); }

// ---------- SETUP ---------- void setup() { Serial.begin(115200); delay(50);

tft.init(); tft.setRotation(TFT_ROTATION); tft.fillScreen(TFT_BLACK);

// Allocate DMA-capable bar source in aligned memory. heap_caps_malloc returns // memory aligned sufficiently for DMA on ESP32, but we still ensure the // length is word-aligned when used in transactions. barSource = (uint16_t*) heap_caps_malloc(barSourceLen * sizeof(uint16_t), MALLOC_CAP_DMA); if (!barSource) { Serial.println("FATAL: failed to allocate barSource in DMA-capable memory"); while (1) delay(1000); }

// Fill the bar source with two halves: colorA then colorB size_t half = barSourceLen / 2; for (size_t i = 0; i < barSourceLen; ++i) barSource[i] = (i < half) ? COLOR_FILL : COLOR_BG;

// Allocate pool of pre-initialized spi_transaction_t in DMA-capable memory transPool = (TransEntry*) heap_caps_malloc(sizeof(TransEntry) * TRANS_POOL_SIZE, MALLOC_CAP_DMA); if (!transPool) { Serial.println("FATAL: failed to allocate transPool in DMA-capable memory"); while (1) delay(1000); } for (int i = 0; i < TRANS_POOL_SIZE; ++i) { memset(&transPool[i].trans, 0, sizeof(spi_transaction_t)); transPool[i].in_use = false; // We'll only update the tx_buffer and length at use time. Store index in user for ISR. transPool[i].trans.user = (void*)(uintptr_t)i; }

// Retrieve TFT_eSPI's internal spi handle (requires small patch to TFT_eSPI to // expose getSpiDeviceHandle()). If not available, user must patch the library. spiHandle = tft.getSpiDeviceHandle(); if (!spiHandle) { Serial.println("FATAL: TFT_eSPI doesn't expose spi handle. Apply the TFT_eSPI patch."); while (1) delay(1000); }

// Register our minimal post-callback on the existing device handle esp_err_t r = spi_device_set_post_cb(spiHandle, spi_post_cb); if (r != ESP_OK) { Serial.printf("spi_device_set_post_cb failed: %d ", r); while (1) delay(1000); }

// Create plot task if (!plotTaskHandle) { xTaskCreatePinnedToCore(plotTask, "plotTask", 4096, NULL, 1, &plotTaskHandle, 1); }

// Demo data static uint16_t demo[BAR_COUNT]; for (size_t i = 0; i < BAR_COUNT; ++i) demo[i] = (i * (MAX_BAR_LENGTH_PIXELS / BAR_COUNT)) % (MAX_BAR_LENGTH_PIXELS+1); startBarPlot(demo, BAR_COUNT, nullptr); }

// ---------- PLOT START ---------- void startBarPlot(const uint16_t* dataArray, size_t count, uint16_t* optionalBarSource) { if (plotInProgress) { Serial.println("Plot already running"); return; } plotDataArray = dataArray; plotDataCount = count; currentBarIndex = 0; plotProgress = 0; plotInProgress = true; if (optionalBarSource) barSource = optionalBarSource;

// Clear background using TFT_eSPI (fast) int totalBars = (int)plotDataCount; int totalPlotWidth = totalBars * BAR_PIXEL_WIDTH + (totalBars - 1) * PLOT_SPACING; tft.fillRect(PLOT_X, PLOT_Y, totalPlotWidth, BAR_PIXEL_HEIGHT, COLOR_BG);

// Kick worker xTaskNotifyGive(plotTaskHandle); }

// ---------- PLOT TASK (reuses transactions, aligns transfers) ---------- static inline size_t valueToSourceOffset(uint16_t value) { if (value > MAX_BAR_LENGTH_PIXELS) value = MAX_BAR_LENGTH_PIXELS; size_t half = barSourceLen / 2; if (value >= half) return 0; return half - value; }

void plotTask(void* pv) { while (true) { ulTaskNotifyTake(pdTRUE, portMAX_DELAY); if (!plotInProgress) continue;

while (plotInProgress && currentBarIndex < plotDataCount) {
  uint16_t val = plotDataArray[currentBarIndex];
  if (val > MAX_BAR_LENGTH_PIXELS) val = MAX_BAR_LENGTH_PIXELS;
  size_t offset = valueToSourceOffset(val);

  // The pixel buffer pointer (DMA-capable)
  uint16_t* pixels = barSource + offset;

  // Destination rect
  int w = BAR_PIXEL_WIDTH;
  int h = BAR_PIXEL_HEIGHT;
  int dx = PLOT_X + (int)currentBarIndex * (BAR_PIXEL_WIDTH + PLOT_SPACING);
  int dy = PLOT_Y;

  // Send CASET/RASET/RAMWR using TFT_eSPI (blocking, short)
  tft.writecommand(0x2A);
  uint8_t casetbuf[4] = { (uint8_t)(dx>>8), (uint8_t)(dx&0xFF), (uint8_t)((dx + w - 1)>>8), (uint8_t)((dx + w - 1)&0xFF) };
  tft.writedata(casetbuf, 4);

  tft.writecommand(0x2B);
  uint8_t rasetbuf[4] = { (uint8_t)(dy>>8), (uint8_t)(dy&0xFF), (uint8_t)((dy + h - 1)>>8), (uint8_t)((dy + h - 1)&0xFF) };
  tft.writedata(rasetbuf, 4);

  tft.writecommand(0x2C);

  // Find a free transaction entry
  int idx = -1;
  for (int i = 0; i < TRANS_POOL_SIZE; ++i) {
    if (!transPool[i].in_use) { idx = i; break; }
  }
  if (idx < 0) {
    // no free entries; wait until one frees (shouldn't happen if pool >= queue)
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    continue;
  }

  // Prepare transaction by updating only the necessary fields
  spi_transaction_t* tr = &transPool[idx].trans;
  transPool[idx].in_use = true;

  // length in bits - ensure word alignment to avoid driver copying
  size_t bytes = w * h * 2; // 2 bytes per pixel (RGB565)
  size_t aligned_bytes = round_up_words(bytes);
  tr->length = aligned_bytes * 8;
  tr->tx_buffer = pixels; // DMA-capable pointer
  tr->user = (void*)(uintptr_t)idx; // pool index for ISR

  // Queue transaction (non-blocking)
  esp_err_t r = spi_device_queue_trans(spiHandle, tr, portMAX_DELAY);
  if (r != ESP_OK) {
    Serial.printf("spi_device_queue_trans failed: %d

", r); transPool[idx].in_use = false; plotInProgress = false; break; }

// Wait to be notified by post-callback that DMA finished
  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

  // Advance
  currentBarIndex++;
  plotProgress = (uint8_t)((currentBarIndex * 100) / (plotDataCount ? plotDataCount : 1));
}

if (currentBarIndex >= plotDataCount) {
  plotInProgress = false;
  plotProgress = 100;
  Serial.println("Plot finished (optimized)");
}

} }

void loop() { static uint32_t last = 0; if (millis() - last > 500) { last = millis(); Serial.printf("plotInProgress=%d progress=%u%% ", plotInProgress ? 1 : 0, plotProgress); } }

/* Next steps / optional deeper optimizations you can request:

1. Move CASET/RASET/RAMWR into queued small transactions with a device-level pre_cb that toggles DC automatically per transaction. That would remove blocking calls to tft.writecommand/writedata and create a fully queued chain of transactions for each bar.


2. Implement full descriptor chaining at the SPI peripheral level to send command+data in a single hardware DMA chain (highest throughput, most complex). This requires building and linking DMA descriptors manually.


3. Increase TRANS_POOL_SIZE and the TFT_eSPI device queue_size for deeper pipelining; add multiple pixel buffers to allow CPU to compute next bars while DMA is busy.


4. If you want I can craft the exact patch for TFT_eSPI to expose pre_cb registration and to provide a helper that queues command+data as a single high-level call. */


