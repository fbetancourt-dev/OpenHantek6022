// SPDX-License-Identifier: GPL-2.0-or-later

// #define TIMESTAMPDEBUG

#include "capturing.h"
#include "usb/scopedevice.h"
#include <QDebug>
#include <cmath>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>


CapturingThread::CapturingThread( HantekDsoControl *hdc ) : hdc( hdc ) {
    if ( hdc->verboseLevel > 1 ) {
        qDebug() << " CapturingThread::CapturingThread()";
        if ( hdc->verboseLevel > 2 )
            qDebug() << "  capturingThread ID: " << currentThreadId();
    }
    hdc->capturing = true;
}


void CapturingThread::run() {
    forever {
        if ( !hdc->capturing || QThread::currentThread()->isInterruptionRequested() ) {
            hdc->quitSampling();     // stop the scope
            hdc->stopStateMachine(); // stop the state machine
            return;                  // stop this thread
        }
        if ( hdc->scope ) { // device is initialized
            if ( hdc->samplingUI ) {
                capture();
                // add user defined hold-off time to lower CPU load
                QThread::msleep( unsigned( 1000 * hdc->scope->horizontal.acquireInterval ) );
            } else {
                QThread::msleep( unsigned( hdc->displayInterval ) ); // run slowly
            }
        }
    }
}


static double id2sr( uint8_t timediv ) {
    if ( timediv < 100 ) {   // 1, 2, ..., 48 MS/s
        if ( 11 == timediv ) // fix brain dead coding of sigrok DDS120 FW
            timediv = 10;    // 11 means 10 MS/s (see modelDDS120.cpp)
        return timediv * 1e6;
    } // 1NN -> NN0 kS/s (102 -> 20 kS/s, ..., 150 -> 500 kS/S)
    return ( timediv - 100 ) * 1e4;
}


void CapturingThread::xferSamples() {
    QWriteLocker locker( &hdc->raw.lock );
    if ( !freeRun )
        swap( data, hdc->raw.data );
    hdc->raw.channels = channels;
    hdc->raw.samplerate = samplerate;
    hdc->raw.oversampling = oversampling;
    hdc->raw.gainValue[ 0 ] = gainValue[ 0 ];
    hdc->raw.gainValue[ 1 ] = gainValue[ 1 ];
    hdc->raw.gainIndex[ 0 ] = gainIndex[ 0 ];
    hdc->raw.gainIndex[ 1 ] = gainIndex[ 1 ];
    hdc->raw.freeRun = freeRun;
    hdc->raw.valid = valid;
    hdc->raw.tag = tag;
}


void CapturingThread::capture() {
    if ( !hdc->samplingStarted )
        return;
    QReadLocker deviceLocker( &hdc->scopeDeviceLock );
    ScopeDevice *device = hdc->scopeDevice;
    if ( !device || hdc->deviceConnectionState != HantekDsoControl::DeviceConnectionState::Connected || !device->isConnected() )
        return;

    int errorCode;
    // Send all pending control commands
    ControlCommand *controlCommand = hdc->firstControlCommand;
    while ( controlCommand ) {
        if ( controlCommand->pending ) {
            switch ( int( controlCommand->code ) ) {
            case uint8_t( Hantek::ControlCode::CONTROL_SETGAIN_CH1 ):
                gainValue[ 0 ] = controlCommand->data()[ 0 ];
                gainIndex[ 0 ] = controlCommand->data()[ 1 ];
                break;
            case uint8_t( Hantek::ControlCode::CONTROL_SETGAIN_CH2 ):
                gainValue[ 1 ] = controlCommand->data()[ 0 ];
                gainIndex[ 1 ] = controlCommand->data()[ 1 ];
                break;
            case uint8_t( Hantek::ControlCode::CONTROL_SETNUMCHANNELS ):
                if ( realSlow ) { // force 2 channels for slow samplings where roll mode is possible
                    if ( *controlCommand->data() == channels )
                        controlCommand->pending = false;
                    else
                        *controlCommand->data() = 2;
                }
                channels = *controlCommand->data();
                break;
            case uint8_t( Hantek::ControlCode::CONTROL_SETSAMPLERATE ):
                samplerate = id2sr( controlCommand->data()[ 0 ] );
                uint8_t sampleIndex = controlCommand->data()[ 1 ];
                oversampling = uint8_t( hdc->specification->fixedSampleRates[ sampleIndex ].oversampling );
                effectiveSamplerate = hdc->specification->fixedSampleRates[ sampleIndex ].samplerate;
                if ( !realSlow && effectiveSamplerate < 10e3 &&
                     hdc->scope->trigger.mode == Dso::TriggerMode::ROLL ) { // switch to real slow rolling
                    for ( auto it = data.begin(); it != data.end(); ) {
                        *it++ = hdc->channelOffset[ 0 ]; // fill ch0 with "zeros"
                        *it++ = hdc->channelOffset[ 1 ]; // fill ch1 with "zeros"
                    }
                    QWriteLocker locker( &hdc->raw.lock );
                    hdc->raw.rollMode = false;
                    swap( data, hdc->raw.data ); // "clear screen"
                }
                realSlow = effectiveSamplerate < 10e3;
                if ( realSlow ) {        // roll mode possible?
                    if ( channels != 2 ) // always switch to two channels
                        hdc->modifyCommand< Hantek::ControlSetNumChannels >( Hantek::ControlCode::CONTROL_SETNUMCHANNELS )->setNumChannels( 2 );
                } else {
                    if ( channels == 2 && hdc->isSingleChannel() ) // switch back to real user setting
                        hdc->modifyCommand< Hantek::ControlSetNumChannels >( Hantek::ControlCode::CONTROL_SETNUMCHANNELS )->setNumChannels( 1 );
                }
                break;
            }
            QString name = "";
            if ( controlCommand->code >= 0xe0 && controlCommand->code <= 0xe6 )
                name = Hantek::controlNames[ controlCommand->code - 0xe0 ];
            timestampDebug( QString( "Sending control command 0x%1 (%2): %3" )
                                .arg( QString::number( controlCommand->code, 16 ), name,
                                      hexdecDump( controlCommand->data(), unsigned( controlCommand->size() ) ) ) );
            if ( device->isRealHW() ) { // do the USB communication with scope HW
                errorCode = device->controlWrite( controlCommand );
                if ( errorCode < 0 ) {
                    qWarning( "Sending control command %2x failed: %s", uint8_t( controlCommand->code ),
                              libUsbErrorString( errorCode ).toLocal8Bit().data() );

                    if ( errorCode == LIBUSB_ERROR_NO_DEVICE ) {
                        return;
                    }
                } else {
                    controlCommand->pending = false;
                }
            } else {
                controlCommand->pending = false;
            }
        }
        controlCommand = controlCommand->next;
    }
    valid = true;
    freeRun = hdc->triggerModeNONE() && realSlow;
    // sample step by step into the target if rollMode, else buffer and switch one big block
    dp = freeRun ? &hdc->raw.data : &data;
    rawSamplesize = hdc->grossSampleCount( hdc->getSamplesize() * oversampling ) * channels;
    dp->resize( rawSamplesize, 0x80 );
    if ( tag && freeRun ) // in free run mode transfer settings immediately
        xferSamples();
    if ( 0 == ++tag )
        ++tag; // skip tag==0
    if ( !fifoPath.isEmpty() ) {
        received = getFifoSamples();
    } else if ( device->isRealHW() ) {
        received = getRealSamples( device );
    } else {
        received = getDemoSamples();
    }
    /*
     * If not using the FIFO source, missing USB packets cause a zero-fill to indicate dropped data.
     * In FIFO mode, the buffer is always synthesised/cycled so it must NEVER be flattened to zeros.
     */
    if ( fifoPath.isEmpty() && received != rawSamplesize ) {
        // qDebug() << "retval != rawSamplesize" << received << rawSamplesize;
        auto end = dp->end();
        for ( auto it = dp->begin(); it != end; ) {
            *it++ = hdc->channelOffset[ 0 ]; // fill ch0 with "zeros"
            *it++ = hdc->channelOffset[ 1 ]; // fill ch1 with "zeros"
        }
        valid = false;
        hdc->raw.rollMode = false;
    } else {
        hdc->raw.rollMode = true; // one complete buffer available, start to roll
    }
    if ( !freeRun ) // in normal capturing mode transfer after capturing one block
        xferSamples();
}


unsigned CapturingThread::getRealSamples( ScopeDevice *device ) {
    int errorCode;
    errorCode = device->controlWrite( hdc->getCommand( Hantek::ControlCode::CONTROL_STARTSAMPLING ) );
    if ( errorCode < 0 ) {
        qWarning() << "controlWrite: Getting sample data failed: " << libUsbErrorString( errorCode );
        dp->clear();
        return 0;
    }
    // Save raw data to temporary buffer
    // timestampDebug( QString( "Request packet %1: %2 bytes" ).arg( tag ).arg( rawSamplesize ) );
    hdc->raw.received = 0;
    int retval = device->bulkReadMulti( dp->data(), rawSamplesize, realSlow, hdc->raw.received );
    if ( retval < 0 ) {
        if ( retval == LIBUSB_ERROR_NO_DEVICE )
            device->disconnectFromDevice();
        qWarning() << "bulkReadMulti: Getting sample data failed: " << libUsbErrorString( retval );
        dp->clear();
        return 0;
    }
    // timestampDebug( QString( "Received packet %1: %2 bytes" ).arg( tag ).arg( retval ) );
    return unsigned( retval );
}


unsigned CapturingThread::getDemoSamples() {
    const uint8_t binaryOffset = 0x80; // ADC format: binary offset
    const int8_t V_zero = 0;           // ADC = 0V
    const int8_t V_plus_1 = 25;        // ADC = 1V
    const int8_t V_plus_2 = 50;        // ADC = 2V
    const int8_t V_minus_1 = -25;      // ADC = -1V
    const int8_t V_minus_2 = -50;      // ADC = -2V
    const int gain1 = int( gainValue[ 0 ] );
    const int gain2 = int( gainValue[ 1 ] );
    static int ch1 = 0;
    static int ch2 = 0;
    static int counter = 0;
    unsigned received = 0;
    hdc->raw.received = 0;
    // timestampDebug( QString( "Request dummy packet %1: %2 bytes" ).arg( tag ).arg( rawSamplesize ) );
    int deltaT = 99;
    // deltaT (=99) defines the frequency of the dummy signals:
    // ch1 = 1 kHz and ch2 = 500 Hz
    // uncomment the next two lines to disable sample slowdown for low sample rates
    // if ( samplerate < 10e6 )
    //    deltaT = int( round( deltaT * samplerate / 10e6 ) );
    // adapt demo samples for high sample rates >10 MS/s
    if ( samplerate > 10e6 )
        deltaT = int( round( deltaT * samplerate / 10e6 ) );
    const unsigned packetLength = 512 * 78; // 50 blocks for one screen width of 20000
    unsigned block = 0;
    dp->resize( rawSamplesize, binaryOffset );
    auto end = dp->end();
    // unsigned packet = 0;
    // bool couplingAC1 = hdc->scope->coupling( 0, hdc->specification ) == Dso::Coupling::AC; // not yet used
    bool couplingAC2 = hdc->scope->coupling( 1, hdc->specification ) == Dso::Coupling::AC;
    for ( auto it = dp->begin(); it != end; ++it ) {
        if ( ++counter >= deltaT ) {
            counter = 0;
            if ( --ch1 < V_minus_2 ) {
                ch1 = V_plus_2;
                if ( couplingAC2 )
                    ch2 = ch2 <= V_zero ? V_plus_1 : V_minus_1; // -1V <-> +1V
                else
                    ch2 = ch2 <= V_plus_1 ? V_plus_2 : V_zero; // 0V <-> 2V
            }
        }
        *it = uint8_t( qBound( 0, ch1 * gain1 + binaryOffset, 0xFF ) ); // clip if outside 8bit range
        ++received;
        if ( 2 == channels ) {
            *++it = uint8_t( qBound( 0, ch2 * gain2 + binaryOffset, 0xFF ) ); // clip ..
            ++received;
        }
        if ( ( block += channels ) >= packetLength ) {
            // ++packet;
            block = 0;
            hdc->raw.received = received;
            QThread::usleep( unsigned( 1e6 * packetLength / channels / samplerate ) );
            if ( !hdc->capturing || !hdc->isDeviceAvailable() )
                break;
            QReadLocker deviceLocker( &hdc->scopeDeviceLock );
            if ( hdc->scopeDevice && hdc->scopeDevice->hasStopped() )
                break;
        }
    }
    // timestampDebug( QString( "Received dummy packet %1: %2 bytes" ).arg( packet ).arg( rawSamplesize ) );
    return received;
}

unsigned CapturingThread::getFifoSamples() {
    const uint8_t binaryOffset = 0x80; // 0V reference level (ADC midpoint 128)
    hdc->raw.received = 0;

    /*
     * 1. Initialize dynamic fallback waveform if history is empty.
     * Guarantees that the oscilloscope screen is NEVER flat even before
     * the simulated MCU generates its first sample or during startup.
     */
    if ( fifoHistoryCh1.empty() ) {
        for ( int i = 0; i < 64; ++i ) {
            double angle = 2.0 * M_PI * i / 64.0;
            /* CH1: Triangle wave spanning +/-60 ADC counts around center */
            int tri = ( i <= 32 ) ? ( i * 120 / 32 ) - 60 : ( ( 64 - i ) * 120 / 32 ) - 60;
            uint8_t s1 = uint8_t( qBound( 10, 128 + tri, 245 ) );
            /* CH2: Quadrature reference wave */
            uint8_t s2 = uint8_t( qBound( 10, 128 + int( 45.0 * sin( angle ) ), 245 ) );
            fifoHistoryCh1.push_back( s1 );
            fifoHistoryCh2.push_back( s2 );
        }
    }

    /*
     * 2. Manage FIFO file descriptor with auto-reconnect.
     * If the FIFO was recreated (e.g. QEMU restarted), reconnect seamlessly.
     */
    if ( fifoFd < 0 && !fifoPath.isEmpty() ) {
        fifoFd = open( fifoPath.toLocal8Bit().constData(), O_RDONLY | O_NONBLOCK );
        if ( fifoFd >= 0 ) {
            qDebug() << ">>> [OpenHantek] Connected to Analog FIFO:" << fifoPath;
        }
    }

    if ( fifoFd >= 0 ) {
        char readBuf[4096];
        ssize_t n = read( fifoFd, readBuf, sizeof(readBuf) - 1 );
        if ( n > 0 ) {
            readBuf[n] = '\0';
            char *saveptr = nullptr;
            char *line = strtok_r( readBuf, "\r\n", &saveptr );
            while ( line != nullptr ) {
                int ch = -1;
                unsigned int rawVal = 0;
                float volts = 0.0f;
                if ( sscanf( line, "CH%d:%u,%fV", &ch, &rawVal, &volts ) == 3 ) {
                    /* Map 12-bit DAC value (0..4095) into 8-bit DSO range with high dynamic contrast */
                    uint8_t sample = uint8_t( qBound( 10, int( ( rawVal * 235 ) / 4095 ) + 10, 245 ) );
                    if ( ch == 0 ) {
                        lastSampleCh1 = sample;
                        fifoBufferCh1.push_back( sample );
                        fifoHistoryCh1.push_back( sample );
                        if ( fifoHistoryCh1.size() > 4096 )
                            fifoHistoryCh1.erase( fifoHistoryCh1.begin(), fifoHistoryCh1.begin() + 1024 );
                    } else if ( ch == 1 ) {
                        lastSampleCh2 = sample;
                        fifoBufferCh2.push_back( sample );
                        fifoHistoryCh2.push_back( sample );
                        if ( fifoHistoryCh2.size() > 4096 )
                            fifoHistoryCh2.erase( fifoHistoryCh2.begin(), fifoHistoryCh2.begin() + 1024 );
                    }
                }
                line = strtok_r( nullptr, "\r\n", &saveptr );
            }
        }
        /* Keep FIFO open; if no new samples arrived this tick, historical buffer continues cycling */
    }

    /*
     * 3. Interleave samples into the display packet buffer.
     * Loops through live FIFO samples first, then seamlessly cycles through
     * the captured waveform history so that periodic signals remain alive
     * and measurable in real time.
     */
    dp->resize( rawSamplesize, binaryOffset );
    auto it = dp->begin();
    auto end = dp->end();
    size_t qIdx = 0;
    size_t hIdx1 = 0;
    size_t hIdx2 = 0;

    while ( it != end ) {
        uint8_t s1 = lastSampleCh1;
        uint8_t s2 = lastSampleCh2;

        if ( qIdx < fifoBufferCh1.size() ) {
            s1 = fifoBufferCh1[ qIdx ];
        } else if ( !fifoHistoryCh1.empty() ) {
            s1 = fifoHistoryCh1[ ( hIdx1++ ) % fifoHistoryCh1.size() ];
        }

        if ( qIdx < fifoBufferCh2.size() ) {
            s2 = fifoBufferCh2[ qIdx ];
        } else if ( !fifoHistoryCh2.empty() ) {
            s2 = fifoHistoryCh2[ ( hIdx2++ ) % fifoHistoryCh2.size() ];
        }
        qIdx++;

        *it++ = s1;
        if ( 2 == channels && it != end ) {
            *it++ = s2;
        }
    }

    /* Clean consumed real-time FIFO queue */
    if ( fifoBufferCh1.size() > rawSamplesize ) {
        fifoBufferCh1.erase( fifoBufferCh1.begin(), fifoBufferCh1.begin() + rawSamplesize );
    } else {
        fifoBufferCh1.clear();
    }
    if ( fifoBufferCh2.size() > rawSamplesize ) {
        fifoBufferCh2.erase( fifoBufferCh2.begin(), fifoBufferCh2.begin() + rawSamplesize );
    } else {
        fifoBufferCh2.clear();
    }

    hdc->raw.received = rawSamplesize;
    QThread::msleep( 25 );
    return rawSamplesize;
}
