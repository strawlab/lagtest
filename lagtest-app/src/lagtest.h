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

class LagTest : public QObject
{    
    Q_OBJECT
public:
    explicit LagTest() { }; //Only here to make Flasher easier
    explicit LagTest(int clockSyncPeriod, int latencyUpdate, int screenFlipPeriod);
    virtual ~LagTest();

public slots:
    void receiveFlashAdruino();
    void recvVersionCheckFinished(QNetworkReply*reply);
    void generateReport();
    void recvFlashAdruino();
    void recvShowLogWindow();
    void recvSelectPort();

    void recvSerialMsg(QString msg);
    void recvSerialError(QString msg);

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
