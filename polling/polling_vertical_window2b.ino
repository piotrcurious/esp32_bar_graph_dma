/* esp32_tft_dma_bar_plot_window_spacing.ino
   Single tall barSource (2*H rows) used as moving window.
   Does NOT clear the full plot area. Only clears spacing areas (fillRect).
   After each bar blit the spacing area to the right of that bar is cleared.
*/

#include <Arduino.h>
#include <TFT_eSPI.h>
#include "esp_heap_caps.h" // MALLOC_CAP_DMA

// for demo
#define SCREEN_UPDATE_RATE_MS 33     // Update interval (approx 30 FPS)

// ---------- USER CONFIG ----------
#define USE_DMA               1    // 1 = attempt DMA, 0 = always fillRect()
#define TFT_ROTATION          1    // keep your display rotation
#define BAR_COUNT             64   // number of bars across the screen
#define BAR_PIXEL_WIDTH       4    // width of each vertical bar in pixels
#define BAR_MAX_HEIGHT_PIXELS 240  // maximum bar height in pixels (vertical)
#define PLOT_X                0   // left margin
#define PLOT_Y                0    // top margin
#define PLOT_SPACING          1    // horizontal spacing between bars (pixels)

// Colors (565)
#define COLOR_FILL  0x07E0   // green
#define COLOR_EXTRA  0x0700   // for debug

#define COLOR_FG    0xFFFF   // white
#define SCREEN_BG   0x0000   // black screen background

// ---------- GLOBALS ----------
TFT_eSPI tft = TFT_eSPI();

volatile bool plotInProgress = false;
volatile uint8_t plotProgress = 0; // 0..100

static const uint16_t* plotDataArray = nullptr; // heights 0..BAR_MAX_HEIGHT_PIXELS
static size_t plotDataCount = 0;
static size_t currentBarIndex = 0;
static TaskHandle_t plotTaskHandle = NULL;

// Single tall precomputed source buffer: 2*H rows, each row has BAR_PIXEL_WIDTH pixels.
static uint16_t* barSource = nullptr;
static size_t barSourcePixels = 0; // number of uint16_t pixels in barSource
static size_t barSourceRows = 0;   // 2 * H
static size_t rowWidth = 0;        // BAR_PIXEL_WIDTH

// Fallback per-bar DMA buffer (w × H)
static uint16_t* dmaBarBuffer = nullptr;
static size_t dmaBarBufferPixels = 0;

// ---------- HELPERS ----------
static inline void memset16(uint16_t* dst, uint16_t val, size_t count) {
    for (size_t i = 0; i < count; ++i) dst[i] = val;
}

// Build single tall barSource with top H rows = BG and bottom H rows = FILL.
// barSource layout row-major, each row has 'rowWidth' pixels.
bool createTallBarSource() {
    const size_t w = (size_t)BAR_PIXEL_WIDTH;
    const size_t h = (size_t)BAR_MAX_HEIGHT_PIXELS;
    barSourceRows = 2 * h;
    rowWidth = w;

    uint64_t totalPixels = (uint64_t)rowWidth * (uint64_t)barSourceRows;
    if (totalPixels == 0 || totalPixels > SIZE_MAX / sizeof(uint16_t)) {
        Serial.println("Precompute: size check failed");
        return false;
    }

    barSourcePixels = (size_t) totalPixels;
    size_t bytes = barSourcePixels * sizeof(uint16_t);

    Serial.printf("Precompute (tall): allocating %u bytes for %u rows (w=%u, rows=%u)\n",
                  (unsigned)bytes, (unsigned)barSourcePixels, (unsigned)w, (unsigned)barSourceRows);

    barSource = (uint16_t*) heap_caps_malloc(bytes, MALLOC_CAP_DMA);
    if (!barSource) {
        Serial.println("Precompute: allocation failed (no DMA-capable memory)");
        return false;
    }

    // Fill top H rows with BG, bottom H rows with FILL
    for (size_t r = 0; r < barSourceRows; ++r) {
        uint16_t color = (r < h) ? COLOR_FG : COLOR_FILL;
        size_t base = r * rowWidth;
        for (size_t c = 0; c < rowWidth; ++c) {
            barSource[base + c] = color;
        }
    }

    return true;
}

// Prepare fallback per-bar DMA buffer (w × H)
bool createPerBarDmaBuffer() {
    const size_t w = (size_t)BAR_PIXEL_WIDTH;
    const size_t h = (size_t)BAR_MAX_HEIGHT_PIXELS;
    dmaBarBufferPixels = w * h;
    size_t bytes = dmaBarBufferPixels * sizeof(uint16_t);
    dmaBarBuffer = (uint16_t*) heap_caps_malloc(bytes, MALLOC_CAP_DMA);
    if (!dmaBarBuffer) {
        Serial.println("Fallback DMA: allocation failed");
        return false;
    }
    // initialize to BG
    memset16(dmaBarBuffer, COLOR_FG, dmaBarBufferPixels);
    return true;
}

// Prepare the fallback per-bar DMA buffer for value v (bottom v rows fill)
static inline void prepareDmaBarBufferForValue(uint16_t v) {
    if (!dmaBarBuffer) return;
    const size_t w = (size_t)BAR_PIXEL_WIDTH;
    const size_t h = (size_t)BAR_MAX_HEIGHT_PIXELS;
    if (v > (uint16_t)h) v = (uint16_t)h;
    for (size_t r = 0; r < h; ++r) {
        bool fillRow = (r >= (h - v)); // bottom v rows
        uint16_t color = fillRow ? COLOR_FILL : COLOR_FG;
        size_t base = r * w;
        for (size_t c = 0; c < w; ++c) dmaBarBuffer[base + c] = color;
    }
}

// Get pointer to the window start row index v inside barSource (clamped)
static inline uint16_t* windowPtrForValue(uint16_t v) {
    if (!barSource) return nullptr;
    if (v > (uint16_t)BAR_MAX_HEIGHT_PIXELS) v = (uint16_t)BAR_MAX_HEIGHT_PIXELS;
    // pointer to pixel at row 'v', column 0
    // offset (in pixels) = v * rowWidth
    return barSource + ((size_t)v * rowWidth);
}

// Clear spacing columns across the full plot area (only the gap columns between bars).
// This is called once before plotting to ensure the gaps are background.
void clearSpacingAreasOnce(size_t count) {
    if (PLOT_SPACING == 0) return;
    const int baseX = PLOT_X;
    const int baseY = PLOT_Y;
    const int w = BAR_PIXEL_WIDTH;
    const int h = BAR_MAX_HEIGHT_PIXELS;

    // Use startWrite/endWrite for grouped SPI ops
    tft.startWrite();
    for (size_t i = 0; i < count; ++i) {
        int dx = baseX + (int)i * (w + PLOT_SPACING);
        int gapX = dx + w;
        // For last bar, gap may be beyond plotting area; still safe to call with width 0 check
        if (PLOT_SPACING > 0) {
            tft.fillRect(gapX, baseY, PLOT_SPACING, h, COLOR_EXTRA);
        }
    }
    tft.endWrite();
}

// ---------- PLOT STARTER ----------
void startBarPlot(const uint16_t* dataArray, size_t count) {
    if (plotInProgress) {
        Serial.println("Plot already running");
        return;
    }
    if (!dataArray || count == 0) {
        Serial.println("Invalid data input");
        return;
    }

    plotDataArray = dataArray;
    plotDataCount = count;
    currentBarIndex = 0;
    plotProgress = 0;
    plotInProgress = true;

    // Clear spacing columns only (not whole area)
  //  clearSpacingAreasOnce(count); // for debug

    // Create plot task if needed
    if (plotTaskHandle == NULL) {
        const uint32_t stackSize = 8192;
        BaseType_t ok = xTaskCreatePinnedToCore(plotTask, "plotTask", stackSize, NULL, 2, &plotTaskHandle, 1);
        if (ok != pdPASS) {
            Serial.println("Failed to create plotTask");
            plotInProgress = false;
            plotTaskHandle = NULL;
        }
    }
}

// ---------- PLOT TASK ----------
void plotTask(void* pvParameters) {
    const int totalBars = (int)plotDataCount;
    const int baseX = PLOT_X;
    const int baseY = PLOT_Y;
    const int w = BAR_PIXEL_WIDTH;
    const int h = BAR_MAX_HEIGHT_PIXELS;

    while (1) {
        if (!plotInProgress) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        if (currentBarIndex >= plotDataCount) {
            plotInProgress = false;
            plotProgress = 100;
            //Serial.println("Plot finished");
            // delete self and clear handle
            TaskHandle_t me = plotTaskHandle;
            plotTaskHandle = NULL;
            vTaskDelete(me);
            return;
        }

        uint16_t value = plotDataArray[currentBarIndex];
        if (value > (uint16_t)h) value = (uint16_t)h;

        int dx = baseX + (int)currentBarIndex * (w + PLOT_SPACING);
        int dy = baseY;

        // Preferred: use single tall barSource and blit window at row 'value'
#if USE_DMA
        if (barSource) {
            uint16_t* src = windowPtrForValue(value); // pointer to start row in barSource
            // Blit full bar rectangle (w x h) so it both draws fill and clears remainder.
            tft.startWrite();
            tft.pushImageDMA(dx, dy, w, h, src, nullptr);

            // Wait for DMA completion
            while (tft.dmaBusy()) vTaskDelay(pdMS_TO_TICKS(0));

            // Immediately clear the spacing region to the right of this bar (single fillRect).
            if (PLOT_SPACING > 0 && currentBarIndex + 1 < plotDataCount) {
                int gapX = dx + w;
                tft.fillRect(gapX, dy, PLOT_SPACING, h, SCREEN_BG);
            }
            tft.endWrite();
        }
        else if (dmaBarBuffer) {
            prepareDmaBarBufferForValue(value);
            tft.startWrite();
            tft.pushImageDMA(dx, dy, w, h, dmaBarBuffer, nullptr);
            while (tft.dmaBusy()) vTaskDelay(pdMS_TO_TICKS(1));
            if (PLOT_SPACING > 0 && currentBarIndex + 1 < plotDataCount) {
                int gapX = dx + w;
                tft.fillRect(gapX, dy, PLOT_SPACING, h, SCREEN_BG);
            }
            tft.endWrite();
        }
        else
#endif
        {
            // Final fallback CPU rectangle drawing (reliable)
            if (value > 0) {
                int fillY = dy + (h - (int)value);
                tft.fillRect(dx, fillY, w, (int)value, COLOR_FILL);
            }
            if (value < (uint16_t)h) {
                int clearH = h - (int)value;
                tft.fillRect(dx, dy, w, clearH, COLOR_FG);
            }
            // clear spacing right away
            if (PLOT_SPACING > 0 && currentBarIndex + 1 < plotDataCount) {
                int gapX = dx + w;
                tft.fillRect(gapX, dy, PLOT_SPACING, h, SCREEN_BG);
            }
        }

        currentBarIndex++;
//        plotProgress = (uint8_t)((currentBarIndex * 100) / (plotDataCount ? plotDataCount : 1));

        // tiny yield to keep system responsive
        vTaskDelay(pdMS_TO_TICKS(0));
    }

    vTaskDelete(NULL);
}

// ---------- SETUP ----------
void setup() {
    Serial.begin(115200);
    delay(50);

    tft.init();
#if USE_DMA
    tft.initDMA();
#endif
    tft.setRotation(TFT_ROTATION);
    tft.fillScreen(SCREEN_BG);

#if USE_DMA
    // Try single tall precompute (preferred)
    if (createTallBarSource()) {
        Serial.println("Using tall precomputed barSource (moving window)");
    } else {
        Serial.println("Tall precompute failed; attempting per-bar DMA buffer");
        if (createPerBarDmaBuffer()) {
            Serial.println("Using per-bar DMA buffer fallback");
        } else {
            Serial.println("No DMA buffers available; will use fillRect fallback");
        }
    }
#endif

    // Example demo data (heights 0..BAR_MAX_HEIGHT_PIXELS)
    static uint16_t demo[BAR_COUNT];
    for (size_t i = 0; i < BAR_COUNT; ++i) {
        demo[i] = (uint16_t)((sin((float)i / (float)BAR_COUNT * 3.14159f * 2.0f) * 0.5f + 0.5f) * BAR_MAX_HEIGHT_PIXELS);
    }

    startBarPlot(demo, BAR_COUNT);
}

// ---------- LOOP ----------

void loop() {
    // Variable to track update rate (e.g., 30 FPS)
    static uint32_t last_update_ms = 0;
    
    // Only update the graph if the desired interval has passed
    if (millis() - last_update_ms < SCREEN_UPDATE_RATE_MS) {
        // Skip if not time to update yet, but allow yield() for RTOS tasks
        yield(); 
        return; 
    }
    last_update_ms = millis();
    
    // Time variable that controls the phase shift and movement speed
    float time_factor = (float)millis() * 0.001f; // Time in seconds

    // Array to hold the bar heights
    static uint16_t demo[BAR_COUNT];
    
    for (size_t i = 0; i < BAR_COUNT; ++i) {
        // Normalized position of the current bar (0.0 to 1.0)
        float bar_norm = (float)i / (float)BAR_COUNT; 

        // --- Multi-Sine Wave Calculation ---
        
        // Wave 1: Primary low-frequency sine wave (The main shape)
        // Frequency: 4 cycles across the screen. Speed: Slow evolution (0.8x time)
        float wave1 = sin(bar_norm * 3.14159f * 4.0f + time_factor * 0.8f);

        // Wave 2: Secondary medium-frequency sine wave (Adds complexity/wobble)
        // Frequency: 8 cycles. Speed: Medium evolution (1.5x time)
        float wave2 = sin(bar_norm * 3.14159f * 8.0f + time_factor * 1.5f);

        // Wave 3: Tertiary high-frequency wave (Adds fine detail/noise)
        // Frequency: 16 cycles. Speed: Faster evolution (2.5x time)
        float wave3 = sin(bar_norm * 3.14159f * 16.0f + time_factor * 2.5f);

        // Combine the waves with different weightings
        // This gives the plot an organic, non-uniform look
        float combined_wave = (wave1 * 0.5f) + (wave2 * 0.3f) + (wave3 * 0.2f);
        
        // --- Scaling and Normalization ---
        
        // combined_wave is now between -1.0 and 1.0
        
        // Normalize to 0.0 to 1.0 (Mapping -1 to 0 and 1 to 1)
        float normalized_output = (combined_wave * 0.5f) + 0.5f; 
        
        // Scale to a final height, ensuring a minimum base height (e.g., 20% of max)
        // This prevents the graph from completely disappearing when waves dip low
        float final_height = (normalized_output * 0.8f + 0.2f) * BAR_MAX_HEIGHT_PIXELS; 

        // Apply a gentle peak-limiting based on time (adds a breathing/pulsing effect)
        float breath_factor = (sin(time_factor * 0.5f) * 0.1f) + 0.9f; // Varies 0.8 to 1.0
        final_height *= breath_factor;

        // Clip to maximum height
        if (final_height > BAR_MAX_HEIGHT_PIXELS) {
            final_height = BAR_MAX_HEIGHT_PIXELS;
        }

        demo[i] = (uint16_t)final_height;
    }

    // --- Plotting and Benchmarking ---
    
    uint32_t benchmark_start = millis();
    startBarPlot(demo, BAR_COUNT);
    // Wait for the plotting function to complete (if it's non-blocking)
    while(plotInProgress) { 
        yield();
    }
    uint32_t benchmark_end = millis();
    
    // The original serial print logic
    Serial.print("took:");
    Serial.println(benchmark_end - benchmark_start); 
}
