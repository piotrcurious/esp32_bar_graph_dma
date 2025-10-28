/* esp32_tft_dma_bar_plot.ino

Highly-optimized non-blocking bar-plot for ESP32 + TFT_eSPI using DMA.

Uses a single reusable DMA-capable "bar" source buffer (larger than drawn width) so only one bar buffer needs to exist in RAM.

Each plotted bar is drawn by issuing a single DMA blit (pushImageDMA) of a horizontal strip. By offsetting the start pointer into the source bar buffer we get an arbitrary-length visible filled area without changing the buffer.

Non-blocking: startBarPlot(...) returns immediately and a background FreeRTOS task drives the DMA sequence. Progress is visible through a global variable.

Uses MALLOC_CAP_DMA to allocate DMA-safe memory for the bar buffer.


IMPORTANT NOTES / REQUIREMENTS

Requires recent TFT_eSPI with DMA support (pushImageDMA, dmaBusy(), startWrite()) and that DMA is enabled in User_Setup.h (TFT_SPI_TRANSFER_MODE and DMA flags where appropriate). This sketch documents a safe, portable approach.

If you want a true interrupt callback on DMA completion, you'll need either:

A TFT_eSPI build that exposes a DMA completion callback or

Drop down to the ESP-IDF spi_master API and queue transactions with user callbacks (advanced). The provided code uses a low-overhead task that waits for dmaBusy() to clear and then continues automatically.



Usage: * configure USER section below for your display and layout * call startBarPlot(dataArray, nBars, barSourcePointerOptional); * check plotInProgress and plotProgress globals to monitor.

Author: assistant (code-quality focused) */

#include <Arduino.h> #include <TFT_eSPI.h> #include "esp_heap_caps.h" // for heap_caps_malloc on ESP32

// ---------- USER CONFIG ---------- #define TFT_ROTATION 1           // choose rotation that makes bars horizontal #define BAR_COUNT 60             // number of bars to draw #define BAR_PIXEL_WIDTH 4        // horizontal pixels per bar (destination width) #define BAR_PIXEL_HEIGHT 6       // vertical pixels per bar (bar thickness) #define PLOT_X 8                 // left margin #define PLOT_Y 40                // top margin #define PLOT_SPACING 1           // pixels between bars #define MAX_BAR_LENGTH_PIXELS 200 // maximum logical bar length in pixels

// Colors (565) #define COLOR_FILL 0x07E0  // green #define COLOR_BG   0xFFFF  // white

// DMA-safe buffer length: make it slightly larger than the drawn width so offsets // can produce different "fill lengths" by shifting start pointer inside the source. // We'll draw exactly (BAR_PIXEL_WIDTH) pixels per bar horizontally, but we issue // a DMA transfer for BAR_MAX_DRAW_WIDTH pixels so the destination column is filled // from the source start address. #define BAR_SOURCE_LEN (MAX_BAR_LENGTH_PIXELS + BAR_PIXEL_WIDTH)

// ---------- GLOBALS ---------- TFT_eSPI tft = TFT_eSPI();

// DMA-source bar buffer (16-bit color). Allocated at runtime in DMA-capable RAM. static uint16_t* barSource = nullptr; static size_t barSourceLen = BAR_SOURCE_LEN;

// A small scratch DMA buffer is sometimes required by TFT_eSPI pushImageDMA, but // many configurations allow passing nullptr. Keep a pointer in case it's needed. static uint16_t* dmaTempBuffer = nullptr; // optional, allocated if needed

// Plot progress globals (user-visible). plotProgress is percentage 0..100. volatile bool plotInProgress = false; volatile uint8_t plotProgress = 0;

// Internal state for plot task static const uint16_t* plotDataArray = nullptr; // pointer to source values (0..MAX_BAR_LENGTH_PIXELS) static size_t plotDataCount = 0; static size_t currentBarIndex = 0;

// FreeRTOS task handle for DMA worker static TaskHandle_t plotTaskHandle = NULL;

// Forward void createBarSource(uint16_t colorA, uint16_t colorB); void startBarPlot(const uint16_t* dataArray, size_t count, uint16_t* optionalBarSource = nullptr); void plotTask(void* pvParameters);

// ---------- SETUP ---------- void setup() { Serial.begin(115200); delay(50); tft.init(); tft.setRotation(TFT_ROTATION); tft.fillScreen(TFT_BLACK);

// Allocate DMA-safe source bar buffer barSource = (uint16_t*) heap_caps_malloc(barSourceLen * sizeof(uint16_t), MALLOC_CAP_DMA); if (!barSource) { Serial.println("FATAL: failed to allocate barSource in DMA-capable memory"); while (1) delay(1000); }

// Optionally allocate a small DMA temp buffer if pushImageDMA requires it. // If you encounter issues remove this allocation and pass nullptr as the second // parameter to pushImageDMA (TFT_eSPI may handle internally). // dmaTempBuffer = (uint16_t*) heap_caps_malloc(32 * 1024, MALLOC_CAP_DMA);

// Prepare default two-color bar source: first half colorA, second half colorB. createBarSource(COLOR_FILL, COLOR_BG);

// Example data (sanitized assumed by user). Values in [0..MAX_BAR_LENGTH_PIXELS] static uint16_t demo[BAR_COUNT]; for (size_t i=0;i<BAR_COUNT;i++) demo[i] = (i * (MAX_BAR_LENGTH_PIXELS / BAR_COUNT)) % (MAX_BAR_LENGTH_PIXELS+1);

// Start non-blocking plot using the demo array. startBarPlot(demo, BAR_COUNT, nullptr); }

// ---------- HELPERS ---------- void createBarSource(uint16_t colorA, uint16_t colorB) { // Fill first half with colorA, second half with colorB. The barSourceLen is // intentionally larger than the visible area so we can offset the start pointer // to show different filled lengths. size_t half = barSourceLen / 2; for (size_t i = 0; i < barSourceLen; ++i) { barSource[i] = (i < half) ? colorA : colorB; } }

// Convert logical value (0..MAX_BAR_LENGTH_PIXELS) to start offset in source buffer. // We choose an offset so that the drawn region contains 'value' pixels of the fill // color at the left side. This math assumes the source buffer layout created by // createBarSource() (fill color first, background second). static inline size_t valueToSourceOffset(uint16_t value) { // Clamp if (value > MAX_BAR_LENGTH_PIXELS) value = MAX_BAR_LENGTH_PIXELS; // We will request DMA to transfer exactly BAR_PIXEL_WIDTH pixels to the display // (for the bar position). To make 'value' appear, we start the source pointer // at (half - value) so that the next 'value' pixels are colorA. size_t half = barSourceLen / 2; if (value >= half) return 0; // saturate return (half - value); }

// ---------- PLOT STARTER (call-and-forget) ---------- // dataArray: pointer to array of uint16_t values in range [0..MAX_BAR_LENGTH_PIXELS] // count: number of bars to draw // optionalBarSource: if provided, must be DMA-capable and match barSourceLen void startBarPlot(const uint16_t* dataArray, size_t count, uint16_t* optionalBarSource) { if (plotInProgress) { Serial.println("Plot already running"); return; } // store pointers plotDataArray = dataArray; plotDataCount = count; currentBarIndex = 0; plotProgress = 0; plotInProgress = true;

// If user provided bar source pointer, use it (must be DMA-capable). Otherwise use default barSource. if (optionalBarSource) { barSource = optionalBarSource; }

// Start background task to drive DMA transfers if (plotTaskHandle == NULL) { xTaskCreatePinnedToCore(plotTask, "plotTask", 4096, NULL, 1, &plotTaskHandle, 1); } }

// ---------- PLOT TASK ---------- // Drives a sequence of DMA blits, one bar per DMA transfer. Uses non-blocking // pushImageDMA and waits for dmaBusy() to clear before launching next transfer. void plotTask(void* pvParameters) { // Pre-calculate geometry const int totalBars = (int)plotDataCount; int x = PLOT_X; int y = PLOT_Y;

// Clear plot area once // compute total plot width int totalPlotWidth = totalBars * BAR_PIXEL_WIDTH + (totalBars - 1) * PLOT_SPACING; tft.fillRect(x, y, totalPlotWidth, BAR_PIXEL_HEIGHT, COLOR_BG);

while (true) { if (!plotInProgress) { // nothing to do, yield CPU vTaskDelay(pdMS_TO_TICKS(50)); continue; }

// draw currentBarIndex
if (currentBarIndex >= plotDataCount) {
  // finished
  plotInProgress = false;
  plotProgress = 100;
  Serial.println("Plot finished");
  continue;
}

uint16_t value = plotDataArray[currentBarIndex];
if (value > MAX_BAR_LENGTH_PIXELS) value = MAX_BAR_LENGTH_PIXELS;

// Calculate source offset and pointer for DMA
size_t offset = valueToSourceOffset(value);
uint16_t* srcPtr = barSource + offset; // pointer into DMA-capable memory

// Destination rectangle for this bar
int w = BAR_PIXEL_WIDTH;
int h = BAR_PIXEL_HEIGHT;
int dx = x + (int)currentBarIndex * (BAR_PIXEL_WIDTH + PLOT_SPACING);
int dy = y;

// Start DMA blit (non-blocking). We call startWrite to keep CS low during DMA.
// The second parameter (dmaTempBuffer) may be nullptr depending on TFT_eSPI config.
tft.startWrite();
tft.pushImageDMA(dx, dy, w, h, srcPtr, dmaTempBuffer);
// pushImageDMA returns immediately (non-blocking) on supported TFT_eSPI builds.

// Wait until DMA finishes. We use a short yield loop that checks dmaBusy().
// This loop intentionally yields the freeRTOS CPU so other tasks (including
// WiFi) continue to run; it's low-overhead compared to per-pixel writes.
while (tft.dmaBusy()) {
  // Sleep briefly (1 tick) — this is cooperative, not a busy spin.
  vTaskDelay(pdMS_TO_TICKS(1));
}

// Ensure frame/transaction ended
tft.endWrite();

// Update progress
currentBarIndex++;
plotProgress = (uint8_t)((currentBarIndex * 100) / (plotDataCount ? plotDataCount : 1));

// Continue to next bar automatically

} }

// ---------- OPTIONAL: function to change bar colors at runtime ---------- // The user wanted the ability to choose source bar address so colors can be swapped fast. // If you want to swap colors quickly, prepare alternative bar sources in DMA-capable // memory and pass their pointer into startBarPlot(..., optionalBarSource).

// ---------- LOOP ---------- void loop() { // The main loop is free to do other stuff. plot runs in background. static uint32_t last = 0; if (millis() - last > 500) { last = millis(); Serial.printf("plotInProgress=%d progress=%u%%\n", plotInProgress ? 1 : 0, plotProgress); } }

/* Final notes & extension ideas:

To make the DMA completion truly interrupt-driven (no polling at all), modify the TFT_eSPI processor-specific source (Processors/...) to expose the DMA completion callback, or implement your own spi_device_queue_trans call with a user callback (advanced, ESP-IDF API). The rest of the code (bar buffer layout, offset logic) remains the same.

If your TFT_eSPI implementation complains about passing arbitrary memory to pushImageDMA, allocate barSource with heap_caps_malloc(..., MALLOC_CAP_DMA) as shown.

Many displays like ST7789 have 16-bit color but some controllers require byte swapping. TFT_eSPI usually handles that; if colors look swapped, use tft.setSwapBytes(true). */
