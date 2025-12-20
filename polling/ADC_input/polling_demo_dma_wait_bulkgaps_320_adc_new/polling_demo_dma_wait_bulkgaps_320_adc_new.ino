// ADC-DMA Bar plot example for ESP32 (Arduino core 3.3.2)
// Uses I2S built-in ADC DMA (I2S0) to stream ADC samples via DMA and feed BarPlotter.
//
// Important:
// - Select an ADC1 pin (GPIO 32-39 for ADC1). ADC2 pins are problematic when WiFi is used.
// - This config assumes 12-bit ADC, 16-bit samples from I2S (packed).
//
// References: I2S ADC built-in mode examples and docs for ESP-IDF (I2S0 -> ADC path).
// (If your Arduino core has the newer adc_continuous driver and you prefer that API
//  I can provide an alternate implementation.)

#include "BarPlotter.h"
#include "driver/i2s.h"
#include "driver/adc.h"

#define SCREEN_UPDATE_RATE_MS 35     // approx 30 FPS

// ---------- USER CONFIG ----------
#define TFT_ROTATION          1
#define BAR_COUNT             320
#define BAR_PIXEL_WIDTH       1
#define BAR_MAX_HEIGHT_PIXELS 240
#define PLOT_X                0
#define PLOT_Y                0
#define PLOT_SPACING          0

// ADC / I2S config
// Use ADC1 channel on a GPIO that maps to ADC1 (32..39).
// Example: ADC1_CHANNEL_4 is GPIO32
#define ADC_UNIT              ADC_UNIT_1
#define ADC_CHANNEL           ADC1_CHANNEL_4   // change to your input (ADC1 channel)
#define ADC_ATTEN             ADC_ATTEN_DB_11  // attenuation (0..11dB) ; choose per expected input voltage

// Sampling rate and DMA buffer sizing
#define SAMPLE_RATE_HZ        155000           // requested samples per second (adjust)
#define I2S_DMA_BUF_LEN       512            // length of each I2S DMA buffer (in samples)
#define I2S_DMA_BUF_COUNT     4               // number of DMA buffers
// How many raw ADC samples to average for each rendered bar (SAMPLES_PER_BAR * BAR_COUNT <= throughput)
#define SAMPLES_PER_BAR       16

// ---------- GLOBALS ----------
TFT_eSPI tft = TFT_eSPI();
BarPlotter plotter(tft);

static uint16_t demo[BAR_COUNT];

// ---------- I2S / ADC helper functions ----------
static i2s_port_t i2s_port = I2S_NUM_0;

void i2s_adc_init()
{
    // Configure ADC attenuation & width
    adc1_config_width(ADC_WIDTH_BIT_12);         // 12-bit ADC
    adc1_config_channel_atten((adc1_channel_t)ADC_CHANNEL, ADC_ATTEN);

    // I2S config for ADC mode (RX + ADC built-in)
    i2s_config_t i2s_config;
    memset(&i2s_config, 0, sizeof(i2s_config));

    i2s_config.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN);
    i2s_config.sample_rate = SAMPLE_RATE_HZ;
    i2s_config.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT; // I2S returns 16-bit words containing ADC data
    i2s_config.channel_format = I2S_CHANNEL_FMT_ONLY_LEFT;  // single channel
    i2s_config.communication_format = I2S_COMM_FORMAT_I2S_MSB;
    i2s_config.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
    i2s_config.dma_buf_count = I2S_DMA_BUF_COUNT;
    i2s_config.dma_buf_len = I2S_DMA_BUF_LEN;
    i2s_config.use_apll = false;
    i2s_config.tx_desc_auto_clear = false; // not used (RX-only)
    i2s_config.fixed_mclk = 0;

    // Install driver
    esp_err_t err = i2s_driver_install(i2s_port, &i2s_config, 0, NULL);
    if (err != ESP_OK) {
        Serial.print("i2s_driver_install failed: ");
        Serial.println((int)err);
    }

    // No pin config needed for built-in ADC route, but API requires call (keeps consistent)
    i2s_pin_config_t pin_config;
    memset(&pin_config, 0, sizeof(pin_config));
    pin_config.bck_io_num = -1;
    pin_config.ws_io_num  = -1;
    pin_config.data_out_num = -1;
    pin_config.data_in_num  = -1;
    i2s_set_pin(i2s_port, &pin_config);

    // Select ADC unit/channel for I2S
    i2s_set_adc_mode(ADC_UNIT, (adc1_channel_t)ADC_CHANNEL); // route ADC channel to I2S

    // Enable ADC for I2S
    i2s_adc_enable(i2s_port);

    Serial.println("I2S ADC DMA initialized.");
}

void i2s_adc_deinit()
{
    i2s_adc_disable(i2s_port);
    i2s_driver_uninstall(i2s_port);
}

// Read N samples from I2S DMA into buffer (blocking until bytes read)
size_t i2s_read_samples(uint16_t* out_buf, size_t samples_to_read, TickType_t ticks_to_wait = portMAX_DELAY)
{
    // Each I2S sample is 16 bits (2 bytes) containing ADC reading in lower bits (12-bit)
    size_t bytes_needed = samples_to_read * sizeof(uint16_t);
    size_t bytes_read = 0;
    esp_err_t res = i2s_read(i2s_port, (void*)out_buf, bytes_needed, &bytes_read, ticks_to_wait);
    if (res != ESP_OK) {
        // i2s_read may fail if RX not enabled or driver not installed
        Serial.print("i2s_read error: ");
        Serial.println((int)res);
        return 0;
    }
    // bytes_read should be multiple of 2
    return bytes_read / sizeof(uint16_t);
}

// Convert raw 16-bit I2S words to 12-bit ADC reading (0..4095)
// On many ESP32s the 16-bit word contains the 12-bit ADC data in lower bits.
static inline uint16_t raw_i2s_to_adc12(uint16_t raw)
{
    // On many platforms the ADC value is left-aligned in the 16-bit word (or right-aligned).
    // Observed common layout: lower 12 bits = ADC reading. Mask to be safe.
    return raw & 0x0FFF;
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

    // Init I2S ADC DMA
    i2s_adc_init();
}

// ---------- LOOP ----------
void loop() {
    static uint32_t last_update_ms = 0;

    if (millis() - last_update_ms < SCREEN_UPDATE_RATE_MS) {
        yield();
        return;
    }
    last_update_ms = millis();

    // We'll read SAMPLE_RATE_HZ / (1000/SCREEN_UPDATE_RATE_MS) samples per update.
    // But simpler: read SAMPLES_PER_BAR * BAR_COUNT raw samples and aggregate per bar.
    const size_t total_samples_needed = (size_t)SAMPLES_PER_BAR * (size_t)BAR_COUNT;
    // To avoid huge single reads, stream in chunks using a modest temp buffer.
    const size_t chunk_samples = 1024; // must be <= I2S_DMA_BUF_LEN * dma_buf_count maybe large; keep modest
    uint16_t *rx_buf = (uint16_t*)malloc(chunk_samples * sizeof(uint16_t));
    if (!rx_buf) {
        Serial.println("Allocation failed");
        return;
    }

    // accumulators
    // Use 32-bit to accumulate sums safely
    static uint32_t sums[BAR_COUNT];
    for (size_t i = 0; i < BAR_COUNT; ++i) sums[i] = 0;

    size_t samples_collected = 0;
    size_t bar_index = 0;
    size_t samples_in_current_bar = 0;

    while (samples_collected < total_samples_needed) {
        size_t to_read = chunk_samples;
        if (total_samples_needed - samples_collected < to_read) to_read = total_samples_needed - samples_collected;

        size_t read = i2s_read_samples(rx_buf, to_read, 100 / portTICK_PERIOD_MS);
        if (read == 0) {
            // If zero samples returned, try briefly yielding and retrying; don't block forever.
            yield();
            continue;
        }

        for (size_t i = 0; i < read; ++i) {
            uint16_t raw = rx_buf[i];
            uint16_t val12 = raw_i2s_to_adc12(raw);
            // sum into current bar
            sums[bar_index] += val12;
            samples_in_current_bar++;
            samples_collected++;

            if (samples_in_current_bar >= SAMPLES_PER_BAR) {
                // move to next bar
                bar_index++;
                if (bar_index >= BAR_COUNT) {
                    // safety cap
                    bar_index = BAR_COUNT - 1;
                }
                samples_in_current_bar = 0;
            }
        }
    }

    free(rx_buf);

    // Convert sums -> bar heights (map 0..4095 -> 0..BAR_MAX_HEIGHT_PIXELS)
    for (size_t i = 0; i < BAR_COUNT; ++i) {
        // average value per bar: sums[i] / SAMPLES_PER_BAR
        uint32_t avg = sums[i] / SAMPLES_PER_BAR;
        // normalize to final pixel height (apply optional scaling)
        float normalized = (float)avg / 4095.0f; // ADC 12-bit
        // apply same breathing and scaling as original demo to keep visual interest
        //float breath_factor = (sin((float)millis() * 0.001f * 0.5f) * 0.1f) + 0.9f;
        //float final_height = (normalized * 0.8f + 0.2f) * (float)BAR_MAX_HEIGHT_PIXELS * breath_factor;
        //if (final_height > BAR_MAX_HEIGHT_PIXELS) final_height = BAR_MAX_HEIGHT_PIXELS;
        //demo[i] = (uint16_t)final_height;
        demo[i] = (uint16_t)((float)normalized*(float)BAR_MAX_HEIGHT_PIXELS);
    
    }

    uint32_t benchmark_start = millis();
    plotter.startBarPlot(demo, BAR_COUNT);
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
