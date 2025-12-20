// ADC DMA -> BarPlotter example for ESP32 (Arduino core 3.3.2)
#include "BarPlotter.h"
#include <driver/i2s.h>
#include <driver/adc.h>
#include <esp_adc_cal.h>

// for demo
#define SCREEN_UPDATE_RATE_MS 33     // Update interval (approx 30 FPS)

// ---------- USER CONFIG ----------
#define TFT_ROTATION          1    // keep your display rotation
#define BAR_COUNT             320  // number of bars across the screen
#define BAR_PIXEL_WIDTH       1    // width of each vertical bar in pixels
#define BAR_MAX_HEIGHT_PIXELS 240  // maximum bar height in pixels (vertical)
#define PLOT_X                0     // left margin
#define PLOT_Y                0     // top margin
#define PLOT_SPACING          0     // horizontal spacing between bars (pixels)

// ---------- ADC / I2S CONFIG ----------
#define I2S_PORT              I2S_NUM_0
// Which ADC1 channel to use (ADC1 channels can be used with I2S ADC)
// ADC1_CHANNEL_0 = GPIO36, ADC1_CHANNEL_1 = GPIO37, ADC1_CHANNEL_2 = GPIO38, ADC1_CHANNEL_3 = GPIO39
#define ADC_CHANNEL           ADC1_CHANNEL_0

// I2S sampling parameters (these affect DMA behaviour)
#define I2S_SAMPLE_RATE       9600   // not too important for single-shot reads; keep reasonable
#define I2S_DMA_BUF_COUNT     4
#define I2S_DMA_BUF_LEN       128     // in samples (per buffer). must be > BAR_COUNT sometimes, but we'll read exact bytes

// ---------- GLOBALS ----------
TFT_eSPI tft = TFT_eSPI();
BarPlotter plotter(tft);

// optional calibration (unused here but kept for reference)
static esp_adc_cal_characteristics_t adc_chars;

// ---------- ADC/I2S init ----------
void init_adc_dma()
{
    // 1) Configure I2S in ADC-DMA (RX) mode
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN),
        .sample_rate = I2S_SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT, // 16-bit container for 12-bit ADC
        .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT, // single channel
        .communication_format = I2S_COMM_FORMAT_I2S_MSB,
        .intr_alloc_flags = 0, // default interrupt priority
        .dma_buf_count = I2S_DMA_BUF_COUNT,
        .dma_buf_len = I2S_DMA_BUF_LEN,
        .use_apll = false,
        .tx_desc_auto_clear = false,
        .fixed_mclk = 0
    };

    // install driver
    esp_err_t err = i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
    if (err != ESP_OK) {
        Serial.print("i2s_driver_install failed: ");
        Serial.println((int)err);
        return;
    }

    // 2) Configure ADC hardware and connect to I2S
    // ADC width and attenuation configuration
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC_CHANNEL, ADC_ATTEN_DB_11); // adjust attenuation to match input voltage range

    // Tell I2S which ADC unit/channel to use
    // For ADC1 channels:
    i2s_set_adc_mode(ADC_UNIT_1, ADC_CHANNEL);

    // enable the ADC for the I2S driver
    i2s_adc_enable(I2S_PORT);

    // Optional: initialize esp_adc_cal for more accurate conversions (not strictly necessary for plot)
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_chars);
}

// ---------- SETUP ----------
void setup() {
    Serial.begin(115200);
    delay(50);

    tft.init();
    tft.initDMA();
    tft.setRotation(TFT_ROTATION);
    tft.fillScreen(0x0000);

    plotter.begin(BAR_COUNT, BAR_PIXEL_WIDTH, BAR_MAX_HEIGHT_PIXELS, PLOT_X, PLOT_Y, PLOT_SPACING);

    // init ADC DMA via I2S
    init_adc_dma();
}

// ---------- LOOP ----------
void loop() {
    static uint32_t last_update_ms = 0;

    if (millis() - last_update_ms < SCREEN_UPDATE_RATE_MS) {
        yield();
        return;
    }
    last_update_ms = millis();

    // buffer to receive raw i2s samples (16-bit signed container)
    static int16_t i2s_raw[BAR_COUNT];
    size_t bytes_to_read = BAR_COUNT * sizeof(int16_t);
    size_t bytes_read = 0;

    // read samples from I2S (this uses DMA)
    esp_err_t r = i2s_read(I2S_PORT, (void*)i2s_raw, bytes_to_read, &bytes_read, 100 / portTICK_PERIOD_MS);
    if (r != ESP_OK || bytes_read != bytes_to_read) {
        Serial.print("ADC DMA read failed! err=");
        Serial.print((int)r);
        Serial.print(" read=");
        Serial.println(bytes_read);
        // If read fails, don't crash: fallback to zeros
        static uint16_t fallback[BAR_COUNT];
        for (size_t i = 0; i < BAR_COUNT; ++i) fallback[i] = 0;
        plotter.startBarPlot(fallback, BAR_COUNT);
        // continue so loop timing remains stable
        return;
    }

    // Convert raw samples to bar heights
    static uint16_t bars[BAR_COUNT];
    for (size_t i = 0; i < BAR_COUNT; ++i) {
        // i2s ADC 12-bit value is in the lower 12 bits of the 16-bit container
        int raw = i2s_raw[i] & 0x0FFF; // mask lower 12 bits
        // scale 0..4095 to 0..BAR_MAX_HEIGHT_PIXELS
        uint32_t h = ((uint32_t)raw * (uint32_t)BAR_MAX_HEIGHT_PIXELS) / 4095U;
        if (h > BAR_MAX_HEIGHT_PIXELS) h = BAR_MAX_HEIGHT_PIXELS;
        bars[i] = (uint16_t)h;
    }

    uint32_t benchmark_start = millis();
    plotter.startBarPlot(bars, BAR_COUNT);
    uint32_t benchmark_init_end = millis();

    while(!plotter.isQueueEmpty()) {
        yield();
    }
    uint32_t benchmark_queue_end = millis();
    while(plotter.isPlotting()) {
        yield();
    }

    uint32_t benchmark_end = millis();

    Serial.print("init:");
    Serial.print(benchmark_init_end - benchmark_start);
    Serial.print(".queue:");
    Serial.print(benchmark_queue_end - benchmark_start);
    Serial.print(",DMA:");
    Serial.println(benchmark_end - benchmark_start);
}
