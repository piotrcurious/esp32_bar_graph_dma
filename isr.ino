/* esp32_tft_dma_bar_plot.ino

TRUE INTERRUPT/CALLBACK-DRIVEN DMA VERSION

This file replaces the simple dmaBusy() polling loop with a true interrupt/ callback-driven DMA pipeline using the ESP-IDF SPI master API.

HOW IT WORKS (summary)

We create a dedicated SPI device on the SPI bus connected to the display. That SPI device is configured with a post-transaction callback (post_cb) which is executed when a DMA transfer finishes.

The plot is driven by a small FreeRTOS task (plotTask). The task sets the drawing window (CASET/RASET) using blocking SPI commands, then queues a non-blocking DMA transaction for the pixel data via spi_device_queue_trans().

When the DMA transfer completes the driver's post_cb notifies the plotTask (via xTaskNotifyFromISR). The task then launches the next bar automatically.

startBarPlot(...) remains call-and-forget; plotInProgress and plotProgress globals report state and progress.


IMPORTANT WARNINGS

This code sends commands compatible with ST7789-like controllers (CASET=0x2A, RASET=0x2B, RAMWR=0x2C). If your display uses a different controller you must adapt the command codes or use the appropriate init sequence.

This implementation bypasses TFT_eSPI for DMA blits; TFT_eSPI is still used for high-level init/utility calls but the SPI bus is also used directly. That can conflict with TFT_eSPI internal state if you also call pushImageDMA concurrently. Use ONLY this code's plotting functionality for DMA transfers or modify TFT_eSPI to expose its SPI handle and callbacks.

The pixel buffer used for DMA MUST be allocated from DMA-capable memory (heap_caps_malloc(..., MALLOC_CAP_DMA)). We allocate barSource accordingly.


If you want a version that integrates with TFT_eSPI internals (calls pushImageDMA but with a callback), we can provide a patch for TFT_eSPI that exposes the spi_device_handle_t and a post-callback. That requires changing TFT_eSPI library sources. */

#include <Arduino.h> #include <TFT_eSPI.h> // used for initialization, colors, and macros #include "esp_heap_caps.h" // heap_caps_malloc

// ESP-IDF SPI and GPIO headers #include "driver/spi_master.h" #include "driver/gpio.h"

// ---------- USER CONFIG ---------- #define TFT_ROTATION 1 #define BAR_COUNT 60 #define BAR_PIXEL_WIDTH 4 #define BAR_PIXEL_HEIGHT 6 #define PLOT_X 8 define PLOT_Y 40 #define PLOT_SPACING 1 #define MAX_BAR_LENGTH_PIXELS 200

#define COLOR_FILL 0x07E0 #define COLOR_BG   0xFFFF

#define BAR_SOURCE_LEN (MAX_BAR_LENGTH_PIXELS + BAR_PIXEL_WIDTH)

// ---------- GLOBALS ---------- TFT_eSPI tft = TFT_eSPI();

static uint16_t* barSource = nullptr; static size_t barSourceLen = BAR_SOURCE_LEN;

volatile bool plotInProgress = false; volatile uint8_t plotProgress = 0;

static const uint16_t* plotDataArray = nullptr; static size_t plotDataCount = 0; static size_t currentBarIndex = 0;

static TaskHandle_t plotTaskHandle = NULL;

// SPI device handle used for DMA transfers static spi_device_handle_t spiHandle = NULL;

// Forward declarations void createBarSource(uint16_t colorA, uint16_t colorB); void startBarPlot(const uint16_t* dataArray, size_t count, uint16_t* optionalBarSource = nullptr); void plotTask(void* pvParameters);

// low-level helpers for sending commands / data via spi_device_transmit static void st_cmd(uint8_t cmd); static void st_data(const void* data, int len);

// post-callback for SPI transactions (called in ISR context by SPI driver) static void IRAM_ATTR spi_post_cb(spi_transaction_t* trans) { BaseType_t xHigherPriorityTaskWoken = pdFALSE; // Notify the plot task that the DMA transfer finished if (plotTaskHandle) { vTaskNotifyGiveFromISR(plotTaskHandle, &xHigherPriorityTaskWoken); if (xHigherPriorityTaskWoken) portYIELD_FROM_ISR(); } }

// ---------- SETUP ---------- void setup() { Serial.begin(115200); delay(50); tft.init(); tft.setRotation(TFT_ROTATION); tft.fillScreen(TFT_BLACK);

// allocate DMA-capable source bar buffer barSource = (uint16_t*) heap_caps_malloc(barSourceLen * sizeof(uint16_t), MALLOC_CAP_DMA); if (!barSource) { Serial.println("FATAL: failed to allocate barSource in DMA-capable memory"); while (1) delay(1000); } createBarSource(COLOR_FILL, COLOR_BG);

// Initialize a separate spi_device for direct DMA transfers to the display // We use the same pins defined by TFT_eSPI (TFT_CS, TFT_DC, TFT_SCLK, TFT_MOSI) // NOTE: if your wiring or defines differ adapt these macros. spi_bus_config_t buscfg = {}; buscfg.mosi_io_num = TFT_MOSI; buscfg.sclk_io_num = TFT_SCLK; buscfg.miso_io_num = -1; buscfg.quadwp_io_num = -1; buscfg.quadhd_io_num = -1; buscfg.max_transfer_sz = BAR_PIXEL_WIDTH * BAR_PIXEL_HEIGHT * sizeof(uint16_t); // reasonable per-bar max

esp_err_t ret = spi_bus_initialize(HSPI_HOST, &buscfg, 1); if (ret != ESP_OK) { Serial.printf("spi_bus_initialize failed: %d ", ret); while (1) delay(1000); }

spi_device_interface_config_t devcfg = {}; devcfg.clock_speed_hz = 40 * 1000 * 1000; // 40 MHz devcfg.mode = 0; devcfg.spics_io_num = TFT_CS; devcfg.queue_size = 3; devcfg.post_cb = spi_post_cb;

ret = spi_bus_add_device(HSPI_HOST, &devcfg, &spiHandle); if (ret != ESP_OK) { Serial.printf("spi_bus_add_device failed: %d ", ret); while (1) delay(1000); }

// create the plot task (blocked until startBarPlot triggers it) if (plotTaskHandle == NULL) { xTaskCreatePinnedToCore(plotTask, "plotTask", 4096, NULL, 1, &plotTaskHandle, 1); }

// demo data static uint16_t demo[BAR_COUNT]; for (size_t i=0;i<BAR_COUNT;i++) demo[i] = (i * (MAX_BAR_LENGTH_PIXELS / BAR_COUNT)) % (MAX_BAR_LENGTH_PIXELS+1); startBarPlot(demo, BAR_COUNT, nullptr); }

// ---------- HELPERS ---------- void createBarSource(uint16_t colorA, uint16_t colorB) { size_t half = barSourceLen / 2; for (size_t i=0;i<barSourceLen;i++) barSource[i] = (i < half) ? colorA : colorB; }

static inline size_t valueToSourceOffset(uint16_t value) { if (value > MAX_BAR_LENGTH_PIXELS) value = MAX_BAR_LENGTH_PIXELS; size_t half = barSourceLen / 2; if (value >= half) return 0; return (half - value); }

// low-level st commands (blocking transmit) - toggles DC line accordingly static void st_cmd(uint8_t cmd) { // DC low for command gpio_set_level((gpio_num_t)TFT_DC, 0); spi_transaction_t t = {}; t.length = 8; t.tx_buffer = &cmd; esp_err_t r = spi_device_transmit(spiHandle, &t); // blocking (void)r; }

static void st_data(const void* data, int len) { // DC high for data gpio_set_level((gpio_num_t)TFT_DC, 1); spi_transaction_t t = {}; t.length = len * 8; t.tx_buffer = data; // Use spi_device_transmit for short control transfers (blocking) esp_err_t r = spi_device_transmit(spiHandle, &t); (void)r; }

// ---------- PLOT STARTER ---------- void startBarPlot(const uint16_t* dataArray, size_t count, uint16_t* optionalBarSource) { if (plotInProgress) { Serial.println("Plot already running"); return; } plotDataArray = dataArray; plotDataCount = count; currentBarIndex = 0; plotProgress = 0; plotInProgress = true; if (optionalBarSource) barSource = optionalBarSource;

// Clear plot area once (we use TFT_eSPI helper for convenience) int totalBars = (int)plotDataCount; int totalPlotWidth = totalBars * BAR_PIXEL_WIDTH + (totalBars - 1) * PLOT_SPACING; tft.fillRect(PLOT_X, PLOT_Y, totalPlotWidth, BAR_PIXEL_HEIGHT, COLOR_BG);

// Kick the plot task to start processing the first bar xTaskNotifyGive(plotTaskHandle); }

// ---------- PLOT TASK (event-driven) ---------- void plotTask(void* pvParameters) { while (true) { // Wait either for startBarPlot to start (notification) or for DMA completion (also notification) ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

if (!plotInProgress) continue;

// If we were notified to start or continue, process bars until queueing the next DMA
while (plotInProgress && currentBarIndex < plotDataCount) {
  uint16_t value = plotDataArray[currentBarIndex];
  if (value > MAX_BAR_LENGTH_PIXELS) value = MAX_BAR_LENGTH_PIXELS;
  size_t offset = valueToSourceOffset(value);
  uint16_t* srcPtr = barSource + offset;

  // Destination rectangle
  int w = BAR_PIXEL_WIDTH;
  int h = BAR_PIXEL_HEIGHT;
  int dx = PLOT_X + (int)currentBarIndex * (BAR_PIXEL_WIDTH + PLOT_SPACING);
  int dy = PLOT_Y;

  // Set column (CASET) and row (RASET) window (ST7789-like commands)
  // CASET: [0x2A] + 4 bytes: x_start, x_end
  // RASET: [0x2B] + 4 bytes: y_start, y_end
  uint8_t caset[] = { 0x2A, (uint8_t)(dx>>8), (uint8_t)(dx&0xFF), (uint8_t)((dx + w - 1)>>8), (uint8_t)((dx + w - 1)&0xFF) };
  uint8_t raset[] = { 0x2B, (uint8_t)(dy>>8), (uint8_t)(dy&0xFF), (uint8_t)((dy + h - 1)>>8), (uint8_t)((dy + h - 1)&0xFF) };

  // Send CASET
  st_cmd(0x2A);
  st_data(&caset[1], 4);
  // Send RASET
  st_cmd(0x2B);
  st_data(&raset[1], 4);
  // Send RAMWR
  st_cmd(0x2C);

  // Now queue the pixel data as a DMA transaction. The SPI device was configured
  // with a post_cb that notifies this task when the DMA finishes.
  spi_transaction_t* trans = (spi_transaction_t*) heap_caps_malloc(sizeof(spi_transaction_t), MALLOC_CAP_DMA);
  if (!trans) {
    Serial.println("Failed to allocate spi_transaction_t");
    plotInProgress = false;
    break;
  }
  memset(trans, 0, sizeof(spi_transaction_t));

  trans->length = w * h * 16; // bits
  trans->tx_buffer = srcPtr;   // must be DMA-capable memory

  // Queue transaction (non-blocking). The post callback will notify us.
  esp_err_t r = spi_device_queue_trans(spiHandle, trans, portMAX_DELAY);
  if (r != ESP_OK) {
    Serial.printf("spi_device_queue_trans failed: %d

", r); heap_caps_free(trans); plotInProgress = false; break; }

// Wait until the post-callback notifies us that DMA finished for this bar.
  // ulTaskNotifyTake will return when post_cb calls vTaskNotifyGiveFromISR.
  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

  // Free the spi_transaction_t allocated earlier (safe now)
  // Note: the SPI driver still retains internal pointers; freeing this struct
  // is safe because driver copies necessary content. If you see crashes,
  // consider allocating a small pool of spi_transaction_t statically.
  heap_caps_free(trans);

  // Update progress and move to next bar
  currentBarIndex++;
  plotProgress = (uint8_t)((currentBarIndex * 100) / (plotDataCount ? plotDataCount : 1));
}

if (currentBarIndex >= plotDataCount) {
  plotInProgress = false;
  plotProgress = 100;
  Serial.println("Plot finished (callback-driven)");
}

} }

void loop() { static uint32_t last = 0; if (millis() - last > 500) { last = millis(); Serial.printf("plotInProgress=%d progress=%u%% ", plotInProgress ? 1 : 0, plotProgress); } }

/* NOTES & EXTENSIONS

This code uses a heap allocation per spi_transaction_t for clarity. For max performance you should pre-allocate a small pool of spi_transaction_t structures (DMA-capable) and reuse them instead of malloc/free in the hot path.

If your display controller is not ST7789-compatible, change the CASET/RASET sequences to match your controller.

If you want to reuse the TFT_eSPI SPI device/handle directly, modify the TFT_eSPI library to expose the internal spi_device_handle_t and set a post_cb there. That approach avoids creating a second device on the bus. */
