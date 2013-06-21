#ifndef SERIALPORTHANDLER_H
#define SERIALPORTHANDLER_H

#include "timemodel.h"
#include "ringbuffer.hpp"
#include "QString"
#include "QTimer"

class LagTestSerialPortComm;

class SerialPortHandler : public QObject
{
    Q_OBJECT
public:
    explicit SerialPortHandler(QString port, int requestPeriod, TimeModel* tm, RingBuffer<clockPair>* clock_storage, RingBuffer<adcMeasurement>* adc_storage);

signals:
    void sendDebugMsg(QString msg);
    void sendErrorMsg(QString msg);
    void sendArduinoTimeout();
    void sendFirmwareVersion(int version);
    void sendArduinoDetectionFailed();
    void sendVersionCheck();

public slots:
    void onThreadQuit();
    void start();
    void stop();
    void doVersionCheck();

private:
    QTimer* timer;
    QThread* thread;
    LagTestSerialPortComm* serial;

    TimeModel* tm;
    RingBuffer<clockPair>* clock_storage;
    RingBuffer<adcMeasurement>* adc_storage;

    QString port;
    int requestPeriod;
    void createSerialPortCommunicator();

};

#include "timemodel.h"
#include "ringbuffer.hpp"
#include <QString>

// lagtest\firmware\lagtestino\lagtest_dataypes.h
typedef uint32_t epoch_dtype;
typedef struct {
    uint8_t cmd;    //either 'H', 'P', 'L', 'V'
    uint8_t value;
    epoch_dtype epoch;
    uint16_t ticks;
    uint8_t checksum;   //bytewise ( value + epoch + ticks ) % 256
} timed_sample_t;


class LagTestSerialPortComm : public QObject
{
    Q_OBJECT
public:
    explicit LagTestSerialPortComm(QString port, int baudRate, TimeModel* tm, RingBuffer<clockPair>* clock_storage, RingBuffer<adcMeasurement>* adc_storage);
    static int getPortIdx(QString portName);

    class InvalidFrameException: public std::exception {};
    class ReadErrorException: public std::exception {};

signals:
    void finished();
    void sendDebugMsg(QString msg);
    void sendErrorMsg(QString msg);
    void sendArduinoTimeout();
    void sendFirmwareVersion(int version);
    void sendArduinoDetectionFailed();
    void sendCheckFirmwareVersion();

public slots:
    void startCommunication();
    void sendClockRequest();
    void recvStop();    
    void doVersionCheck();

private:
    void init();
    bool decode2Frame(uint8_t *buffer, timed_sample_t* frame);
    void closeSerialPort();
    void initSerialPort();    
    void sendArduinoTimeRequest();
    void sendArduinoVersionRequest();
    int readFrameFromSerial(uint8_t* buffer, int frameLength, int bufferSize);
    void getNextFrame(timed_sample_t& frame , double &timeRead);

    bool stopThread;

    int write(unsigned char *data, int size);
    int read(unsigned char *buffer, int max_size);

    //Get current PC time and store data
    TimeModel* tm;
    RingBuffer<clockPair>* clock_storage;
    RingBuffer<adcMeasurement>* adc_storage;

    //Needed for handling Arduino Clock requests
    bool sendRequest;
    const static int ntimeRequests = 20;
    double timeRequests[ntimeRequests];
    unsigned int tR;

    // Serial Port Config
    //QString port;
    int baudRate;
    int portN;

};

#endif // SERIALPORTHANDLER_H
