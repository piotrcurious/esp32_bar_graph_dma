// BarPlotter.cpp (merged + new features)
#include "BarPlotter.h"
#include "esp_heap_caps.h" // MALLOC_CAP_DMA
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

const uint8_t BarPlotter::_pattern[BarPlotter::_patternH][BarPlotter::_patternW] = {
    {1,1,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0},
    {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0},
    {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0},
    {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}
};

BarPlotter::BarPlotter(TFT_eSPI& tft) :
    _tft(tft),
    _barCount(0),
    _barWidth(0),
    _barMaxHeight(0),
    _plotX(0),
    _plotY(0),
    _plotSpacing(0),
    _plotInProgress(false),
    _plotQueueInProgress(false),
    _plotDataArray(nullptr),
    _plotDataCount(0),
    _currentBarIndex(0),
    _plotTaskHandle(NULL),
    _barSourceCopies(nullptr),
    _barSourcePixelsSingle(0),
    _barSourceCopiesCount(0),
    _barSourceRowsSingle(0),
    _rowWidth(0),
    _dmaBarBuffer(nullptr),
    _dmaBarBufferPixels(0),
    _tftMutex(NULL),
    _frameMutex(NULL),
    _timingTaskHandle(NULL),
    _frameBuf(nullptr),
    _frameReady(false)
{
    // Default colors
    _colorFill = 0x07E0;   // green
    _colorExtra = 0x0700;   // for debug (spacing clear)
    _colorFg = 0xFFFF;   // white (background of bar area)
    _screenBg = 0x0000;   // black screen background
    _gridColor = 0x5502; // color used for the pattern (grid)
}

BarPlotter::~BarPlotter() {
    // Stop timing task if present
    stopTearFree();

    if (_barSourceCopies) {
        heap_caps_free(_barSourceCopies);
        _barSourceCopies = nullptr;
    }
    if (_dmaBarBuffer) {
        heap_caps_free(_dmaBarBuffer);
        _dmaBarBuffer = nullptr;
    }
    if (_frameBuf) {
        heap_caps_free(_frameBuf);
        _frameBuf = nullptr;
    }
    if (_plotTaskHandle) {
        vTaskDelete(_plotTaskHandle);
        _plotTaskHandle = NULL;
    }
    if (_tftMutex) {
        vSemaphoreDelete(_tftMutex);
        _tftMutex = NULL;
    }
    if (_frameMutex) {
        vSemaphoreDelete(_frameMutex);
        _frameMutex = NULL;
    }
}

// --- Existing begin, createTallBarSourceCopiesWithPattern, createPerBarDmaBuffer, etc.
// Keep those functions unchanged. (They still exist above or below - unchanged.)

bool BarPlotter::createTallBarSourceCopiesWithPattern() {
    const size_t copies = (size_t)_patternH;
    if (copies == 0) return false;

    const size_t w = (size_t)_barWidth;
    const size_t h = (size_t)_barMaxHeight;
    const size_t rowsPerCopy = 2 * h;
    _rowWidth = w;
    _barSourceRowsSingle = rowsPerCopy;

    uint64_t pixelsSingle = (uint64_t)_rowWidth * (uint64_t)rowsPerCopy;
    if (pixelsSingle == 0 || pixelsSingle > SIZE_MAX / sizeof(uint16_t)) {
        Serial.println("Precompute copies: size check failed");
        return false;
    }
    _barSourcePixelsSingle = (size_t)pixelsSingle;

    uint64_t totalPixels = (uint64_t)_barSourcePixelsSingle * (uint64_t)copies;
    if (totalPixels == 0 || totalPixels > SIZE_MAX / sizeof(uint16_t)) {
        Serial.println("Precompute copies: total size check failed");
        return false;
    }

    size_t totalBytes = (size_t) totalPixels * sizeof(uint16_t);

    _barSourceCopies = (uint16_t*) heap_caps_malloc(totalBytes, MALLOC_CAP_DMA);
    if (!_barSourceCopies) {
        Serial.println("Precompute copies: allocation failed (no DMA-capable memory)");
        return false;
    }

    for (size_t k = 0; k < copies; ++k) {
        uint16_t* basePtr = _barSourceCopies + (k * _barSourcePixelsSingle);

        for (size_t r = 0; r < rowsPerCopy; ++r) {
            uint16_t baseColor = (r < h) ? _colorFg : _colorFill;
            size_t base = r * _rowWidth;
            size_t patternRow = (r + k) % _patternH;

            for (size_t c = 0; c < _rowWidth; ++c) {
                size_t patternCol = c % _patternW;
                uint16_t color = baseColor;
                if (_pattern[patternRow][patternCol]) {
                    color = _gridColor;
                }
                basePtr[base + c] = color;
            }
        }
    }

    _barSourceCopiesCount = copies;
    return true;
}

bool BarPlotter::createPerBarDmaBuffer() {
    const size_t w = (size_t)_barWidth;
    const size_t h = (size_t)_barMaxHeight;
    _dmaBarBufferPixels = w * h;
    size_t bytes = _dmaBarBufferPixels * sizeof(uint16_t);
    _dmaBarBuffer = (uint16_t*) heap_caps_malloc(bytes, MALLOC_CAP_DMA);
    if (!_dmaBarBuffer) {
        Serial.println("Fallback DMA: allocation failed");
        return false;
    }
    for (size_t i = 0; i < _dmaBarBufferPixels; ++i) _dmaBarBuffer[i] = _colorFg;
    return true;
}

void BarPlotter::prepareDmaBarBufferForValue(uint16_t v) {
    if (!_dmaBarBuffer) return;
    const size_t w = (size_t)_barWidth;
    const size_t h = (size_t)_barMaxHeight;
    if (v > (uint16_t)h) v = (uint16_t)h;
    for (size_t r = 0; r < h; ++r) {
        bool fillRow = (r >= (h - v));
        uint16_t color = fillRow ? _colorFill : _colorFg;
        size_t base = r * w;
        for (size_t c = 0; c < w; ++c) _dmaBarBuffer[base + c] = color;
    }
}

uint16_t* BarPlotter::windowPtrForValueAndCopy(uint16_t v, size_t copyIndex) {
    if (!_barSourceCopies) return nullptr;
    if (copyIndex >= _barSourceCopiesCount) copyIndex = 0;
    if (v > (uint16_t)_barMaxHeight) v = (uint16_t)_barMaxHeight;
    return _barSourceCopies + (copyIndex * _barSourcePixelsSingle) + ((size_t)v * _rowWidth);
}



bool BarPlotter::begin(int barCount, int barWidth, int barMaxHeight, int plotX, int plotY, int plotSpacing) {
    _barCount = barCount;
    _barWidth = barWidth;
    _barMaxHeight = barMaxHeight;
    _plotX = plotX;
    _plotY = plotY;
    _plotSpacing = plotSpacing;

    // allocate internal frame buffer (DMA-capable) for offered frames
    size_t fbBytes = (size_t)_barCount * sizeof(uint16_t);
    _frameBuf = (uint16_t*) heap_caps_malloc(fbBytes, MALLOC_CAP_DMA);
    if (!_frameBuf) {
        Serial.println("BarPlotter.begin: failed to allocate internal frame buffer (DMA)");
        return false;
    }

    // create per-frame mutex
    _frameMutex = xSemaphoreCreateMutex();
    if (!_frameMutex) {
        Serial.println("BarPlotter.begin: failed to create frame mutex");
        heap_caps_free(_frameBuf);
        _frameBuf = nullptr;
        return false;
    }

    if (createTallBarSourceCopiesWithPattern()) {
        Serial.printf("Using tall precomputed barSource copies with pattern (%u copies)\n", (unsigned)_barSourceCopiesCount);
        return true;
    } else {
        Serial.println("Tall precompute copies with pattern failed; attempting per-bar DMA buffer");
        if (createPerBarDmaBuffer()) {
            Serial.println("Using per-bar DMA buffer fallback");
            return true;
        } else {
            Serial.println("No DMA buffers available; will use fillRect fallback");
            return true;
        }
    }
}

// ----------------- New: tear-free timing subsystem -----------------

// timing task (pinned to core 0)
void BarPlotter::timingTask(void* pv) {
    BarPlotter* self = (BarPlotter*)pv;
    const TickType_t frameTicks = pdMS_TO_TICKS((uint32_t)(1000.0f / self->_tf_refreshRate));
    TickType_t lastWake = xTaskGetTickCount();

    while (1) {
        // sleep until next frame tick
        vTaskDelayUntil(&lastWake, frameTicks);

        // 1) do even-frame register writes (short atomic block)
        if (self->_tftMutex) {
            if (xSemaphoreTake(self->_tftMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                self->_tft.startWrite();
                self->_tft.writecommand(ST7789_FRCTR2);
                self->_tft.writedata(self->_tf_var1);
                self->_tft.writecommand(ST7789_PORCTRL);
                self->_tft.writedata(self->_tf_var2);
                self->_tft.writedata(self->_tf_var3);
                self->_tft.writedata(0x01);
                self->_tft.writedata(0x33);
                self->_tft.writedata(0x33);
                self->_tft.endWrite();
                xSemaphoreGive(self->_tftMutex);
            } else {
                // if mutex contention, skip register write this frame (best-effort)
            }
        } else {
            // fallback, no mutex present (shouldn't happen if initTearFree used)
            self->_tft.startWrite();
            self->_tft.writecommand(ST7789_FRCTR2);
            self->_tft.writedata(self->_tf_var1);
            self->_tft.writecommand(ST7789_PORCTRL);
            self->_tft.writedata(self->_tf_var2);
            self->_tft.writedata(self->_tf_var3);
            self->_tft.writedata(0x01);
            self->_tft.writedata(0x33);
            self->_tft.writedata(0x33);
            self->_tft.endWrite();
        }

        // 2) after register writes, if a new frame is available and plotter idle -> start plot
        bool haveFrame = false;
        if (self->_frameMutex) {
            if (xSemaphoreTake(self->_frameMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                haveFrame = self->_frameReady;
                xSemaphoreGive(self->_frameMutex);
            }
        }
        if (haveFrame) {
            // ensure no plot currently in progress
            if (!self->isPlotting() && self->isQueueEmpty()) {
                // start a new plot using internal frame buffer as data source
                self->startBarPlot(self->_frameBuf, (size_t)self->_barCount);

                // consume the frame flag
                if (self->_frameMutex) {
                    if (xSemaphoreTake(self->_frameMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                        self->_frameReady = false;
                        xSemaphoreGive(self->_frameMutex);
                    }
                }
            }
        }

        // Mid-frame odd-frame adjustment will be handled by plotter if desired using the same mutex.
        // For simplicity the second half adjust can be implemented here if required.
    }
}

bool BarPlotter::initTearFree(uint8_t var1, uint8_t var2, uint8_t var3,
                              uint8_t var4, uint8_t var5, uint8_t var6,
                              float refreshRate)
{
    // store params
    _tf_var1 = var1; _tf_var2 = var2; _tf_var3 = var3;
    _tf_var4 = var4; _tf_var5 = var5; _tf_var6 = var6;
    _tf_refreshRate = refreshRate;

    // create tft mutex
    if (_tftMutex) {
        // already initialized
        return true;
    }
    _tftMutex = xSemaphoreCreateMutex();
    if (!_tftMutex) {
        Serial.println("initTearFree: failed to create _tftMutex");
        _tftMutex = NULL;
        return false;
    }

    // create/reset frameReady flag (already created in begin)
    _frameReady = false;

    // create timing task (pinned to core 0)
    const uint32_t stackSize = 8192;
    //BaseType_t ok = xTaskCreatePinnedToCore(timingTask, "BarPlotterTiming", stackSize, this, 2, &_timingTaskHandle, 0);
    //BaseType_t ok = xTaskCreatePinnedToCore(BarPlotter::timingTask, "BarPlotterTiming", stackSize, this, 2, &_timingTaskHandle, 0);
      BaseType_t ok = xTaskCreate(timingTask, "BarPlotterTiming", stackSize, this, 2, &_timingTaskHandle);

    if (ok != pdPASS) {
        Serial.println("initTearFree: failed to create timing task");
        vSemaphoreDelete(_tftMutex);
        _tftMutex = NULL;
        return false;
    }

    Serial.println("BarPlotter: tear-free timing initialized");
    return true;
}

void BarPlotter::stopTearFree() {
    if (_timingTaskHandle) {
        vTaskDelete(_timingTaskHandle);
        _timingTaskHandle = NULL;
    }
    if (_tftMutex) {
        vSemaphoreDelete(_tftMutex);
        _tftMutex = NULL;
    }
    // keep frameMutex and frameBuf (they are freed in destructor)
}

// Offer a frame: copies data into internal DMA buffer and marks it ready.
// Non-blocking (short mutex). Returns true if copied.
bool BarPlotter::offerFrame(const uint16_t* data, size_t count) {
    if (!data) return false;
    if (count != (size_t)_barCount) return false;
    if (!_frameBuf || !_frameMutex) return false;

    if (xSemaphoreTake(_frameMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        memcpy(_frameBuf, data, count * sizeof(uint16_t));
        _frameReady = true;
        xSemaphoreGive(_frameMutex);
       // Serial.println("BarPlotter: offerFrame accepted");

        return true;
    } else {
        // couldn't acquire quickly; skip this offered frame to avoid blocking ADC
        return false;
    }
}

void BarPlotter::startBarPlot(const uint16_t* dataArray, size_t count) {
    if (_plotInProgress) {
        Serial.println("Plot already running");
        return;
    }
    if (!dataArray || count == 0) {
        Serial.println("Invalid data input");
        return;
    }

    // Acquire the TFT mutex and open a startWrite() transaction for the entire plot run.
    // This mirrors original BarPlotter behaviour: a single startWrite() surrounds pushImageDMA calls.
    bool haveLock = false;
    if (_tftMutex) {
        // block here until we can get the mutex - starting a plot must be atomic
        if (xSemaphoreTake(_tftMutex, portMAX_DELAY) == pdTRUE) {
            haveLock = true;
        }
    }

    if (haveLock) {
        _tft.startWrite(); // keep the write transaction open during the whole plot run
    } else {
        // fallback: try best-effort startWrite (not recommended)
        _tft.startWrite();
    }

    _plotDataArray = dataArray;
    _plotDataCount = count;
    _currentBarIndex = 0;
    _plotInProgress = true;
    _plotQueueInProgress = true;

    // clear spacing now (we already hold the _tftMutex and startWrite)
    clearSpacingAreas();

    if (_plotTaskHandle == NULL) {
        const uint32_t stackSize = 8192;
        BaseType_t ok = xTaskCreatePinnedToCore(plotTask, "plotTask", stackSize, this, 2, &_plotTaskHandle, 0);
        if (ok != pdPASS) {
            Serial.println("Failed to create plotTask");
            _plotInProgress = false;
            _plotTaskHandle = NULL;
            // if plotTask creation failed, make sure to close transaction and release mutex
            _tft.endWrite();
            if (haveLock) xSemaphoreGive(_tftMutex);
        } else {
            Serial.println("BarPlotter: startBarPlot() called");
        }
    }
}


void BarPlotter::plotTask(void* pvParameters) {
    BarPlotter* self = (BarPlotter*)pvParameters;
    const int totalBars = (int)self->_plotDataCount;
    const int baseX = self->_plotX;
    const int baseY = self->_plotY;
    const int w = self->_barWidth;
    const int h = self->_barMaxHeight;

    Serial.printf("BarPlotter: plotTask running, bars to draw: %d\n", totalBars);

    while (1) {
        if (!self->_plotInProgress) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        if (self->_currentBarIndex >= self->_plotDataCount) {
            self->_plotQueueInProgress = false;
            // Wait for any outstanding DMA transfers to finish
            while (self->_tft.dmaBusy()) vTaskDelay(pdMS_TO_TICKS(1));
            self->_plotInProgress = false;

            // Close the long-running startWrite() transaction and release the TFT mutex so timingTask can run.
            self->_tft.endWrite();
            if (self->_tftMutex) {
                xSemaphoreGive(self->_tftMutex);
            }

            TaskHandle_t me = self->_plotTaskHandle;
            self->_plotTaskHandle = NULL;
            vTaskDelete(NULL);
            return;
        }

        uint16_t value = self->_plotDataArray[self->_currentBarIndex];
        if (value > (uint16_t)h) value = (uint16_t)h;

        int dx = baseX + (int)self->_currentBarIndex * (w + self->_plotSpacing);
        int dy = baseY;

        size_t copyIndex = 0;
        if (self->_barSourceCopiesCount >= (size_t)self->_patternH) {
            size_t anchor = (size_t)(self->_plotY % self->_patternH);
            size_t v_mod = (size_t)(value % (uint16_t)self->_patternH);
            copyIndex = (anchor + self->_patternH - v_mod) % self->_patternH;
        } else if (self->_barSourceCopiesCount > 0) {
            copyIndex = self->_currentBarIndex % self->_barSourceCopiesCount;
        }

        // We DO NOT take _tftMutex per-bar here because startBarPlot already acquired it and started startWrite().
        if (self->_barSourceCopies) {
            uint16_t* src = self->windowPtrForValueAndCopy(value, copyIndex);
            self->_tft.pushImageDMA(dx, dy, w, h, src, nullptr);
        } else if (self->_dmaBarBuffer) {
            self->prepareDmaBarBufferForValue(value);
            self->_tft.pushImageDMA(dx, dy, w, h, self->_dmaBarBuffer, nullptr);
        } else {
            if (value > 0) {
                int fillY = dy + (h - (int)value);
                self->_tft.fillRect(dx, fillY, w, (int)value, self->_colorFill);
            }
            if (value < (uint16_t)h) {
                int clearH = h - (int)value;
                self->_tft.fillRect(dx, dy, w, clearH, self->_colorFg);
            }
        }

        self->_currentBarIndex++;

        // small yield to let DMA engine proceed and keep other tasks responsive
        taskYIELD();
    }

    vTaskDelete(NULL);
}


// clearSpacingAreas uses _tft; ensure callers have taken _tftMutex if necessary.
// startBarPlot already takes the _tftMutex briefly before calling clearSpacingAreas(),
// so this function can assume it's safe to call _tft.fillRect().
void BarPlotter::clearSpacingAreas() {
    if (_plotSpacing == 0) return;
    for (size_t i = 0; i < _barCount; ++i) {
        int dx = _plotX + (int)i * (_barWidth + _plotSpacing);
        int gapX = dx + _barWidth;
        if (_plotSpacing > 0) {
            _tft.fillRect(gapX, _plotY, _plotSpacing, _barMaxHeight, _screenBg);
        }
    }
}

bool BarPlotter::isPlotting() const {
    return _plotInProgress;
}

bool BarPlotter::isQueueEmpty() const {
    return !_plotQueueInProgress;
}

void BarPlotter::endWrite() {
    _tft.endWrite();
}


/* The rest of the file continues with any other unchanged helper methods from your original file. */
