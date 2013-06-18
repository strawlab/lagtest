#ifndef LAGTEST_H
#define LAGTEST_H

#include <QObject>


#include <vector>

class QSettings;
class QNetworkReply;

class LatencyModel;
class SerialPortHandler;
class Window;
class QPlainTextEdit;
class TimeModel;

class LagTest : public QObject
{
    Q_OBJECT
public:
    explicit LagTest() : w(NULL), serial(NULL), lm(NULL) { }; //Only here to make Flasher easier
    explicit LagTest(int clockSyncPeriod, int latencyUpdate, int screenFlipPeriod, bool createLogWindow = true);
    virtual ~LagTest();

signals:
    void stopMeasurement();

public slots:    
    void recvVersionCheckFinished(QNetworkReply*reply);
    void generateReport();
    void recvFlashArduino();
    void recvShowLogWindow();
    void recvSelectPort();

    void recvSerialMsg(QString msg);
    void recvSerialError(QString msg);
    void recvArduinoTimeout();
    void recvArduinoFirmwareVersion(int version);

protected:
    QString getOS();
    bool loadSettings();
    bool testPort(QString port);
    void doNewVersionCheck();
    void setupLogWindow();

    std::vector<QString> discoverComPorts();
    int programArduino(QString avrDudePath, QString pathToFirmware, QString port=QString() );
    QString makeUserSelectPort();

    QSettings* settings;
    LatencyModel* lm;
    SerialPortHandler* serial;
    TimeModel* tm;
    Window* w;
};


class Flasher : LagTest
{
    Q_OBJECT
public:
    explicit Flasher(QString avrDudePath, QString pathToFirmware, QString port=QString()) : LagTest() {
        programArduino( avrDudePath, pathToFirmware, port );
    }
};

#endif // LAGTEST_H
