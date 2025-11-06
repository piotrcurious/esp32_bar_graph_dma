#include "BarPlotter.h"

// for demo
#define SCREEN_UPDATE_RATE_MS 33     // Update interval (approx 30 FPS)

// ---------- USER CONFIG ----------
#define TFT_ROTATION          1    // keep your display rotation
#define BAR_COUNT             320   // number of bars across the screen
#define BAR_PIXEL_WIDTH       1    // width of each vertical bar in pixels
#define BAR_MAX_HEIGHT_PIXELS 240  // maximum bar height in pixels (vertical)
#define PLOT_X                0     // left margin
#define PLOT_Y                0     // top margin
#define PLOT_SPACING          0     // horizontal spacing between bars (pixels)

// ---------- GLOBALS ----------
TFT_eSPI tft = TFT_eSPI();
BarPlotter plotter(tft);

// ---------- SETUP ----------
void setup() {
    Serial.begin(115200);
    delay(50);

    tft.init();
    tft.initDMA();
    tft.setRotation(TFT_ROTATION);
    tft.fillScreen(0x0000);

    plotter.begin(BAR_COUNT, BAR_PIXEL_WIDTH, BAR_MAX_HEIGHT_PIXELS, PLOT_X, PLOT_Y, PLOT_SPACING);
}

// ---------- LOOP ----------
void loop() {
    static uint32_t last_update_ms = 0;

    if (millis() - last_update_ms < SCREEN_UPDATE_RATE_MS) {
        yield();
        return;
    }
    last_update_ms = millis();

    float time_factor = (float)millis() * 0.001f;

    static uint16_t demo[BAR_COUNT];

    for (size_t i = 0; i < BAR_COUNT; ++i) {
        float bar_norm = (float)i / (float)BAR_COUNT;

        float wave1 = sin(bar_norm * 3.14159f * 4.0f + time_factor * 0.8f);
        float wave2 = sin(bar_norm * 3.14159f * 8.0f + time_factor * 1.5f);
        float wave3 = sin(bar_norm * 3.14159f * 16.0f + time_factor * 2.5f);

        float combined_wave = (wave1 * 0.5f) + (wave2 * 0.3f) + (wave3 * 0.2f);

        float normalized_output = (combined_wave * 0.5f) + 0.5f;
        float final_height = (normalized_output * 0.8f + 0.2f) * BAR_MAX_HEIGHT_PIXELS;
        float breath_factor = (sin(time_factor * 0.5f) * 0.1f) + 0.9f;
        final_height *= breath_factor;
        if (final_height > BAR_MAX_HEIGHT_PIXELS) final_height = BAR_MAX_HEIGHT_PIXELS;

        demo[i] = (uint16_t)final_height;
    }

    uint32_t benchmark_start = millis();
    plotter.startBarPlot(demo, BAR_COUNT);
    while(!plotter.isQueueEmpty()) {
        yield();
    }
    uint32_t benchmark_queue_end = millis();
    while(plotter.isPlotting()) {
        yield();
    }

    uint32_t benchmark_end = millis();

    Serial.print("queue:");
    Serial.print(benchmark_queue_end - benchmark_start);
    Serial.print(",DMA:");
    Serial.println(benchmark_end - benchmark_start);
}
