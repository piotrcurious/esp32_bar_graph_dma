/* esp32_tft_dma_bar_plot_window_spacing_with_pattern_array.ino
   Modified to support repeating patterns (e.g. 4x4 grid) embedded into each copy.
   Pattern is defined as a small 2D array (patternH x patternW) with values 0/1.
   Each copy is built by tiling the pattern across rows and columns, then applying
   a vertical shift per-copy so that repeating patterns align differently per column.
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
#define PLOT_X                0     // left margin
#define PLOT_Y                0     // top margin
#define PLOT_SPACING          1     // horizontal spacing between bars (pixels)

// Pattern repeat: number of copies of the source pattern (affects periodicity)
#define PATTERN_REPEAT        4     // <-- change this for different periodicities

// ---------- PATTERN DEFINITION (example: 4x4 grid) ----------
/*
   Define any pattern here as patternH x patternW array.
   Use 1 = fill (COLOR_FILL), 0 = background/top (COLOR_FG).
   The pattern will be tiled horizontally to fill the bar width,
   and vertically to fill the 2*H rows; each copy may have a vertical
   shift to produce the spatial periodic effect.
*/
static const uint8_t patternW = 4;
static const uint8_t patternH = 4;
static const uint8_t pattern[patternH][patternW] = {
    // a simple 4x4 grid-like pattern: horizontal lines every 4 pixels and vertical lines every 4 pixels
    // 1 marks the filled pixel.
    {1,0,0,1},
    {0,0,0,0},
    {0,0,0,0},
    {1,0,0,1}
};

// Colors (565)
#define COLOR_FILL  0x07E0   // green
#define COLOR_EXTRA 0x0700   // for debug (spacing clear)
#define COLOR_FG    0xFFFF   // white (background of bar area)
#define SCREEN_BG   0x0000   // black screen background
#define GRID_COLOR  0x5502 // kolor używany dla wzoru (kratki). 
// ---------- GLOBALS ----------
TFT_eSPI tft = TFT_eSPI();

volatile bool plotInProgress = false;
volatile uint8_t plotProgress = 0; // 0..100

static const uint16_t* plotDataArray = nullptr; // heights 0..BAR_MAX_HEIGHT_PIXELS
static size_t plotDataCount = 0;
static size_t currentBarIndex = 0;
static TaskHandle_t plotTaskHandle = NULL;

// Single tall precomputed source buffer copies: copies × (2*H rows) × rowWidth pixels.
// layout: [copy0 rows 0..(2h-1)] [copy1 rows ...] ...
static uint16_t* barSourceCopies = nullptr;
static size_t barSourcePixelsSingle = 0; // pixels in single copy (rowWidth * rowsPerCopy)
static size_t barSourceCopiesCount = 0;  // number of copies (PATTERN_REPEAT)
static size_t barSourceRowsSingle = 0;   // rows per single copy (2*H)
static size_t rowWidth = 0;              // BAR_PIXEL_WIDTH

// Fallback per-bar DMA buffer (w × H)
static uint16_t* dmaBarBuffer = nullptr;
static size_t dmaBarBufferPixels = 0;

// ---------- HELPERS ----------
static inline void memset16(uint16_t* dst, uint16_t val, size_t count) {
    for (size_t i = 0; i < count; ++i) dst[i] = val;
}


// Zastąp swoją funkcję tą implementacją.
// Tworzymy dokładnie patternH kopii; każda kopia k ma pattern przesunięty o k wierszy:
// patternRow for physical row r = (r + k) % patternH
bool createTallBarSourceCopiesWithPattern(size_t requestedCopies) {
    // ensure we have at least patternH copies to achieve full staticness
    const size_t copies = (size_t)patternH;
    if (copies == 0) return false;

    const size_t w = (size_t)BAR_PIXEL_WIDTH;
    const size_t h = (size_t)BAR_MAX_HEIGHT_PIXELS;
    const size_t rowsPerCopy = 2 * h;
    rowWidth = w;
    barSourceRowsSingle = rowsPerCopy;

    uint64_t pixelsSingle = (uint64_t)rowWidth * (uint64_t)rowsPerCopy;
    if (pixelsSingle == 0 || pixelsSingle > SIZE_MAX / sizeof(uint16_t)) {
        Serial.println("Precompute copies: size check failed");
        return false;
    }
    barSourcePixelsSingle = (size_t)pixelsSingle;

    uint64_t totalPixels = (uint64_t)barSourcePixelsSingle * (uint64_t)copies;
    if (totalPixels == 0 || totalPixels > SIZE_MAX / sizeof(uint16_t)) {
        Serial.println("Precompute copies: total size check failed");
        return false;
    }

    size_t totalBytes = (size_t) totalPixels * sizeof(uint16_t);
    Serial.printf("Precompute (tall copies with pattern, for static pattern): allocating %u bytes for %u pixels (w=%u, rowsPerCopy=%u, copies=%u)\n",
                  (unsigned)totalBytes, (unsigned)totalPixels, (unsigned)w, (unsigned)rowsPerCopy, (unsigned)copies);

    barSourceCopies = (uint16_t*) heap_caps_malloc(totalBytes, MALLOC_CAP_DMA);
    if (!barSourceCopies) {
        Serial.println("Precompute copies: allocation failed (no DMA-capable memory)");
        return false;
    }

    // Build copies: copy k has pattern shifted by +k rows (patternRow = (r + k) % patternH)
    for (size_t k = 0; k < copies; ++k) {
        uint16_t* basePtr = barSourceCopies + (k * barSourcePixelsSingle);

        for (size_t r = 0; r < rowsPerCopy; ++r) {
            // two-tone base: top H rows = COLOR_FG, bottom H rows = COLOR_FILL
            uint16_t baseColor = (r < h) ? COLOR_FG : COLOR_FILL;
            size_t base = r * rowWidth;

            // pattern row uses +k shift
            size_t patternRow = (r + k) % patternH;

            for (size_t c = 0; c < rowWidth; ++c) {
                size_t patternCol = c % patternW;
                uint16_t color = baseColor;
                if (pattern[patternRow][patternCol]) {
                    color = GRID_COLOR; // overlay kratki
                }
                basePtr[base + c] = color;
            }
        }
    }

    barSourceCopiesCount = copies;
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
    // initialize to BG (COLOR_FG used as background color for top)
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

// Get pointer to the window start row index v inside a specific copy of barSource (clamped).
// copyIndex selects which copy (0..barSourceCopiesCount-1)
static inline uint16_t* windowPtrForValueAndCopy(uint16_t v, size_t copyIndex) {
    if (!barSourceCopies) return nullptr;
    if (copyIndex >= barSourceCopiesCount) copyIndex = 0;
    if (v > (uint16_t)BAR_MAX_HEIGHT_PIXELS) v = (uint16_t)BAR_MAX_HEIGHT_PIXELS;
    // offset in pixels: copyIndex*barSourcePixelsSingle + v*rowWidth
    return barSourceCopies + (copyIndex * barSourcePixelsSingle) + ((size_t)v * rowWidth);
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

    // Optionally clear spacing columns only (not whole area)
    // clearSpacingAreasOnce(count);

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

     // Determine copyIndex to keep pattern static
size_t copyIndex = 0;
if (barSourceCopiesCount >= (size_t)patternH) {
    size_t anchor = (size_t)(PLOT_Y % patternH);
    size_t v_mod = (size_t)(value % (uint16_t)patternH);
    copyIndex = (anchor + patternH - v_mod) % patternH;
} else if (barSourceCopiesCount > 0) {
    copyIndex = currentBarIndex % barSourceCopiesCount;
}

        // Preferred: use tall barSource copies and blit window at row 'value' from chosen copy
#if USE_DMA
        if (barSourceCopies) {
            uint16_t* src = windowPtrForValueAndCopy(value, copyIndex); // pointer into chosen copy
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
                tft.fillRect(gapX, dy, PLOT_SPACING,h,  SCREEN_BG);
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
                tft.fillRect(gapX, dy, PLOT_SPACING,h,  SCREEN_BG);
            }
        }

        currentBarIndex++;
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
    // Try tall precompute with multiple copies embedding the pattern (preferred)
    if (createTallBarSourceCopiesWithPattern(PATTERN_REPEAT)) {
        Serial.printf("Using tall precomputed barSource copies with pattern (%u copies)\n", (unsigned)barSourceCopiesCount);
    } else {
        Serial.println("Tall precompute copies with pattern failed; attempting per-bar DMA buffer");
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
        float wave1 = sin(bar_norm * 3.14159f * 4.0f + time_factor * 0.8f);
        float wave2 = sin(bar_norm * 3.14159f * 8.0f + time_factor * 1.5f);
        float wave3 = sin(bar_norm * 3.14159f * 16.0f + time_factor * 2.5f);

        float combined_wave = (wave1 * 0.5f) + (wave2 * 0.3f) + (wave3 * 0.2f);
        
        float normalized_output = (combined_wave * 0.5f) + 0.5f; 
        float final_height = (normalized_output * 0.8f + 0.2f) * BAR_MAX_HEIGHT_PIXELS; 
        float breath_factor = (sin(time_factor * 0.5f) * 0.1f) + 0.9f; // Varies 0.8 to 1.0
        final_height *= breath_factor;
        if (final_height > BAR_MAX_HEIGHT_PIXELS) final_height = BAR_MAX_HEIGHT_PIXELS;

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
    
    Serial.print("took:");
    Serial.println(benchmark_end - benchmark_start); 
}
