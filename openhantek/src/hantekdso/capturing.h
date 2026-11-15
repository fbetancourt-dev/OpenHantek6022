// SPDX-License-Identifier: GPL-2.0-or-later

#pragma once

#include "hantekdsocontrol.h"

class CapturingThread : public QThread {
    Q_OBJECT

  public:
    CapturingThread( HantekDsoControl *hdc );
    void quitCapturing() { hdc->capturing = false; }
    void setFifoPath( const QString &path ) { fifoPath = path; }

  private:
    void run() override;
    void capture();
    unsigned getRealSamples( ScopeDevice *device );
    unsigned getDemoSamples();
    unsigned getFifoSamples();
    void xferSamples();
    HantekDsoControl *hdc;
    QString fifoPath;
    int fifoFd = -1;
    uint8_t lastSampleCh1 = 0x80;
    uint8_t lastSampleCh2 = 0x80;
    std::vector< uint8_t > fifoBufferCh1;
    std::vector< uint8_t > fifoBufferCh2;
    std::vector< uint8_t > fifoHistoryCh1;
    std::vector< uint8_t > fifoHistoryCh2;
    unsigned channels = 0;
    double effectiveSamplerate = 0;
    bool realSlow = false;
    double samplerate = 0;
    unsigned oversampling = 0;
    unsigned rawSamplesize = 0;
    unsigned received = 0;
    unsigned gainValue[ 2 ] = { 0, 0 }; // 1,2,5,10,..
    unsigned gainIndex[ 2 ] = { 0, 0 }; // index 0..7
    unsigned tag = 0;
    bool valid = true;
    bool freeRun = false;
    std::vector< unsigned char > data;
    std::vector< unsigned char > *dp = &data;
};
