#ifndef BAR_PLOTTER_H
#define BAR_PLOTTER_H

#include <Arduino.h>
#include <TFT_eSPI.h>

class BarPlotter {
public:
    BarPlotter(TFT_eSPI& tft);
    ~BarPlotter();

    bool begin(int barCount, int barWidth, int barMaxHeight, int plotX, int plotY, int plotSpacing);
    void startBarPlot(const uint16_t* dataArray, size_t count);
    bool isPlotting() const;
    bool isQueueEmpty() const;
    void endWrite();

private:
    static void plotTask(void* pvParameters);

    TFT_eSPI& _tft;

    int _barCount;
    int _barWidth;
    int _barMaxHeight;
    int _plotX;
    int _plotY;
    int _plotSpacing;

    volatile bool _plotInProgress;
    volatile bool _plotQueueInProgress;

    const uint16_t* _plotDataArray;
    size_t _plotDataCount;
    size_t _currentBarIndex;
    TaskHandle_t _plotTaskHandle;

    uint16_t* _barSourceCopies;
    size_t _barSourcePixelsSingle;
    size_t _barSourceCopiesCount;
    size_t _barSourceRowsSingle;
    size_t _rowWidth;

    uint16_t* _dmaBarBuffer;
    size_t _dmaBarBufferPixels;

    // Pattern definition
    static const uint8_t _patternW = 4;
    static const uint8_t _patternH = 16;
    static const uint8_t _pattern[_patternH][_patternW];

    // Colors
    uint16_t _colorFill;
    uint16_t _colorExtra;
    uint16_t _colorFg;
    uint16_t _screenBg;
    uint16_t _gridColor;

    bool createTallBarSourceCopiesWithPattern();
    bool createPerBarDmaBuffer();
    void prepareDmaBarBufferForValue(uint16_t v);
    uint16_t* windowPtrForValueAndCopy(uint16_t v, size_t copyIndex);
};

#endif // BAR_PLOTTER_H
