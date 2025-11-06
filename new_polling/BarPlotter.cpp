#include "BarPlotter.h"
#include "esp_heap_caps.h" // MALLOC_CAP_DMA

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
    _dmaBarBufferPixels(0)
{
    // Default colors
    _colorFill = 0x07E0;   // green
    _colorExtra = 0x0700;   // for debug (spacing clear)
    _colorFg = 0xFFFF;   // white (background of bar area)
    _screenBg = 0x0000;   // black screen background
    _gridColor = 0x5502; // color used for the pattern (grid)
}

BarPlotter::~BarPlotter() {
    if (_barSourceCopies) {
        heap_caps_free(_barSourceCopies);
    }
    if (_dmaBarBuffer) {
        heap_caps_free(_dmaBarBuffer);
    }
    if (_plotTaskHandle) {
        vTaskDelete(_plotTaskHandle);
    }
}

bool BarPlotter::begin(int barCount, int barWidth, int barMaxHeight, int plotX, int plotY, int plotSpacing) {
    _barCount = barCount;
    _barWidth = barWidth;
    _barMaxHeight = barMaxHeight;
    _plotX = plotX;
    _plotY = plotY;
    _plotSpacing = plotSpacing;

    if (createTallBarSourceCopiesWithPattern()) {
        Serial.printf("Using tall precomputed barSource copies with pattern (%u copies)\\n", (unsigned)_barSourceCopiesCount);
        return true;
    } else {
        Serial.println("Tall precompute copies with pattern failed; attempting per-bar DMA buffer");
        if (createPerBarDmaBuffer()) {
            Serial.println("Using per-bar DMA buffer fallback");
            return true;
        } else {
            Serial.println("No DMA buffers available; will use fillRect fallback");
            return false;
        }
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

    _plotDataArray = dataArray;
    _plotDataCount = count;
    _currentBarIndex = 0;
    _plotInProgress = true;
    _plotQueueInProgress = true;

    clearSpacingAreas();

    if (_plotTaskHandle == NULL) {
        const uint32_t stackSize = 8192;
        BaseType_t ok = xTaskCreatePinnedToCore(plotTask, "plotTask", stackSize, this, 2, &_plotTaskHandle, 0);
        if (ok != pdPASS) {
            Serial.println("Failed to create plotTask");
            _plotInProgress = false;
            _plotTaskHandle = NULL;
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
    Serial.printf("Precompute (tall copies with pattern, for static pattern): allocating %u bytes for %u pixels (w=%u, rowsPerCopy=%u, copies=%u)\\n",
                  (unsigned)totalBytes, (unsigned)totalPixels, (unsigned)w, (unsigned)rowsPerCopy, (unsigned)copies);

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

void BarPlotter::plotTask(void* pvParameters) {
    BarPlotter* self = (BarPlotter*)pvParameters;
    const int totalBars = (int)self->_plotDataCount;
    const int baseX = self->_plotX;
    const int baseY = self->_plotY;
    const int w = self->_barWidth;
    const int h = self->_barMaxHeight;

    while (1) {
        if (!self->_plotInProgress) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        if (self->_currentBarIndex >= self->_plotDataCount) {
            self->_plotQueueInProgress = false;
            while (self->_tft.dmaBusy()) vTaskDelay(pdMS_TO_TICKS(0));
            self->_plotInProgress = false;
            TaskHandle_t me = self->_plotTaskHandle;
            self->_plotTaskHandle = NULL;
            vTaskDelete(me);
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

        if (self->_barSourceCopies) {
            uint16_t* src = self->windowPtrForValueAndCopy(value, copyIndex);
            self->_tft.pushImageDMA(dx, dy, w, h, src, nullptr);
        } else if (self->_dmaBarBuffer) {
            self->prepareDmaBarBufferForValue(value);
            self->_tft.pushImageDMA(dx, dy, w, h, self->_dmaBarBuffer, nullptr);
            while (self->_tft.dmaBusy()) vTaskDelay(pdMS_TO_TICKS(1));
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
        vTaskDelay(pdMS_TO_TICKS(0));
    }

    vTaskDelete(NULL);
}

void BarPlotter::clearSpacingAreas() {
    if (_plotSpacing == 0) return;

    _tft.startWrite();
    for (size_t i = 0; i < _barCount; ++i) {
        int dx = _plotX + (int)i * (_barWidth + _plotSpacing);
        int gapX = dx + _barWidth;
        if (_plotSpacing > 0) {
            _tft.fillRect(gapX, _plotY, _plotSpacing, _barMaxHeight, _screenBg);
        }
    }
    _tft.endWrite();
}
