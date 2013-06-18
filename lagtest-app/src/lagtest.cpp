#include "lagtest.h"

#include <QSettings>
#include <QString>
#include <QInputDialog>
#include <QProcess>

#include "timemodel.h"
#include "latencymodel.h"
#include "serialporthandler.h"
#include "window.h"
#include "rs232.h"
#include <QPlainTextEdit>
#include <qapplication.h>

#include <QNetworkAccessManager>
#include <QNetworkReply>
#include <QFileDialog>
#include <QMessageBox>


#include <QDesktopServices>
#include <QVariantMap>
#include <QAbstractButton>
#include <QFile>
#include <QDir>

LagTest::LagTest(int clockSyncPeriod, int latencyUpdate, int screenFlipPeriod, bool createLogWindow)
    : w(NULL), serial(NULL), lm(NULL)
{
    if( createLogWindow )
        this->setupLogWindow();

    this->doNewVersionCheck();

    TimeModel* tm = new TimeModel();
    //tm.testModelGenerator();

    RingBuffer<screenFlip>* screenFlips = new RingBuffer<screenFlip>(20);
    RingBuffer<clockPair>* arduinoClock = new RingBuffer<clockPair>(100);
    RingBuffer<adcMeasurement>* adcValues = new RingBuffer<adcMeasurement>(20000);

    this->loadSettings();

    //Make user define the serial port
    QString port = settings->value("Arduino/Port").toString();

    qDebug("Creating handler for serial port on %s...", port.toStdString().c_str() );
    // Setup Serial Port Reader
    this->serial = new SerialPortHandler(port, clockSyncPeriod, tm, arduinoClock, adcValues);
    this->lm = new LatencyModel(latencyUpdate, tm, screenFlips, arduinoClock, adcValues);
    this->w = new Window(tm, screenFlips);

    QObject::connect( w, SIGNAL(doReset()), lm, SLOT(reset()) );
    QObject::connect( w, SIGNAL(startMeasurement()), serial, SLOT(start()) );
    QObject::connect( w, SIGNAL(startMeasurement()), lm, SLOT(start()) );
    QObject::connect( w, SIGNAL(generateReport()), this, SLOT( generateReport() ) );
    QObject::connect( w, SIGNAL(flashArduino()), this, SLOT( recvFlashArduino() ) );
    QObject::connect( w, SIGNAL(showLogWindow()), this, SLOT( recvShowLogWindow() ) );
    QObject::connect( w, SIGNAL(selectPort()), this, SLOT( recvSelectPort() ) );

    //QObject::connect( &w, &Window::flashArduino, getArduinoPort );


    QObject::connect( lm, SIGNAL(signalStableLatency()),        w, SLOT(receiveStableLatency()) );
    QObject::connect( lm, SIGNAL(signalUnstableLatency()),      w, SLOT(receiveUnstableLatency()) );
    QObject::connect( lm, SIGNAL(signalInvalidLatency()),       w, SLOT(receiveInvalidLatency()) );
    QObject::connect( lm, SIGNAL(signalUpdate(LatencyModel*)),  w, SLOT(receiveLatencyUpdate(LatencyModel*)) );
    QObject::connect( lm, SIGNAL(signalNewMeasurementWindow(uint8_t*,double*,double*,flip_type)), w, SLOT(receiveNewMeasurementWindow(uint8_t*,double*,double*,flip_type)) );


    QObject::connect( serial, SIGNAL(sendDebugMsg(QString)),    this, SLOT(recvSerialMsg(QString)) );
    QObject::connect( serial, SIGNAL(sendErrorMsg(QString)),    this, SLOT(recvSerialError(QString)) );

    w->show();
}

LagTest::~LagTest()
{
}



QPlainTextEdit* logWindow = NULL;



#if QT_VERSION >= 0x050000
void myMessageOutput(QtMsgType type, const QMessageLogContext &context, const QString &msg)
#else
void myMessageOutput(QtMsgType type, const char *msg)
#endif
{
    if( logWindow !=  NULL) {
        logWindow->appendPlainText( msg );
    } else {
#if QT_VERSION >= 0x050000
        fprintf(stderr, "%s" , msg.toStdString().c_str() );
#else
        fprintf(stderr, "%s", msg );
#endif
    }
}

void LagTest::setupLogWindow()
{
    logWindow = new QPlainTextEdit(0);
    logWindow->setWindowTitle( "Log window ");
    logWindow->setCenterOnScroll(true);
    logWindow->setReadOnly(true);
    logWindow->resize( 700 , 500 );
    logWindow->hide();
#if QT_VERSION >= 0x050000
    qInstallMessageHandler( myMessageOutput );
#else
    qInstallMsgHandler( myMessageOutput );
#endif
}

void LagTest::recvShowLogWindow(){
    logWindow->show();
}

void LagTest::recvSerialMsg(QString msg)
{
    QString s;
    s = "SerialPort: ";
    s += msg;
    qDebug( "%s", s.toStdString().c_str() );
}

void LagTest::recvSerialError(QString msg)
{
    QString s;
    s = "Error SerialPort: ";
    s += msg;
    qDebug( "%s", s.toStdString().c_str() );
}

void LagTest::generateReport()
{
    QString text;
    QString s;
    text.append( "Lagtest - Latency Report\n" );
    text.append( "########################\n" );

    text.append( "\n" );
    text.append( s.sprintf("Average Latency:        %3.2f [ms]\n" , (this->lm->getAvgLatency()/1e6) ) );
    text.append( s.sprintf("Standard deviation:     %3.2f [ms]\n" , this->lm->getAvgLatencySD()/1e6 ) );
    text.append( s.sprintf("Measurement duration:   %3.2f [sec]\n" , this->lm->getMeasurementDuration() / 1e9 ) );

    text.append( "\n" );
    text.append( tr("Operating System:  %1 \n").arg(this->getOS()) );
    text.append( "\n" );
    text.append( tr("Display Vendor: XXXXXXX \n") );
    text.append( tr("Display Model:  XXXXXXX \n") );

    text.append( "\n" );
    text.append( "Notes:\n" );
    text.append( "\n" );

    text.append( "\n" );
    text.append( tr("Report generated with LagTest v%1 \n").arg( QCoreApplication::applicationVersion() ) );
    text.append( tr("Find out how slow YOUR display is. Check %1 \n").arg("http://lagtest.org") );

    qDebug("Report: \n%s", text.toStdString().c_str());

    QString fileName = QFileDialog::getSaveFileName(0, tr("Save Protocol"), "C:", tr("Text File (*.txt)"));
    QFile f(fileName);
    if( f.open( QIODevice::WriteOnly | QIODevice::Text ) )
    {
        f.write( text.toLocal8Bit() );
    } else {
        QMessageBox::warning(0, tr("Write Error"), tr("Writing Protocol failed!") , QMessageBox::Ok, QMessageBox::NoButton);
    }
    f.close();
}

QString LagTest::getOS()
{
    qDebug( "System gets %d" , QSysInfo::WordSize );
#ifdef Q_OS_WIN
    switch( QSysInfo::windowsVersion() )
    {
        case QSysInfo::WV_2003:
            //return ( QSysInfo::WordSize	== 32 ) ? "Windows 2003 (x86)" : "Windows 2003 (x64)" ;
            return "Windows 2003";
        break;

        case QSysInfo::WV_VISTA:
            //return ( QSysInfo::WordSize	== 32 ) ? "Windows Vista (x86)" : "Windows Vista (x64)" ;
            return "Windows Vista";
        break;

        case QSysInfo::WV_WINDOWS7:
            //return ( QSysInfo::WordSize	== 32 ) ? "Windows 7 (x86)" : "Windows 7 (x64)" ;
            return "Windows 7";
        break;

        case QSysInfo::WV_WINDOWS8:
            //return ( QSysInfo::WordSize	== 32 ) ? "Windows 8 (x86)" : "Windows 8 (x64)" ;
            return "Windows 8";
        break;
    }
#endif

    return "UNKNOWN OS";
}

void LagTest::recvFlashArduino()
{
    //qDebug("Current path %s" , QCoreApplication::applicationDirPath().toStdString().c_str() );
    programArduino( QCoreApplication::applicationDirPath().append("/tools/avrdude.exe"), QCoreApplication::applicationDirPath().append("/firmware.hex"), this->settings->value("Arduino/Port").toString());
}

void LagTest::recvSelectPort()
{
    QString port = makeUserSelectPort();
    QSettings* settings = new QSettings("lagtest.ini", QSettings::IniFormat);
    settings->setValue("Arduino/Port", port );
    settings->sync();
    delete(settings);
}

void LagTest::doNewVersionCheck()
{
    QNetworkAccessManager *manager = new QNetworkAccessManager(this);
    connect(manager, SIGNAL(finished(QNetworkReply*)), this, SLOT(recvVersionCheckFinished(QNetworkReply*)));
    manager->get(QNetworkRequest(QUrl("http://version.lagtest.org/latest")));
    //manager->get(QNetworkRequest(QUrl("http://www.google.de/index.html")));
}

void LagTest::recvVersionCheckFinished(QNetworkReply *reply)
{
    if( reply->error() == QNetworkReply::NoError )
    {
        QByteArray d = reply->readAll();

        QString this_version_json;
        this_version_json.sprintf("{\"version\"': \"%s\"}", QCoreApplication::applicationVersion().toStdString().c_str());
        QString available_version_json(d.data());

        if (this_version_json != available_version_json) {
            QMessageBox *box = new QMessageBox(QMessageBox::NoIcon, "Version update",
					     tr("New version available! %1 -> %2").arg(this_version_json).arg(available_version_json),
					     QMessageBox::Ignore | QMessageBox::Ok);
            box->button(QMessageBox::Ok)->setText("Update");
            int buttonPressed = box->exec();
            if( buttonPressed == QMessageBox::Ok ) {
                QDesktopServices::openUrl(QUrl("http://lagtest.org", QUrl::TolerantMode));
            }
        }

    } else {
        qDebug( "Lagtest version check failed! %s\n " , reply->errorString().toStdString().c_str() );
    }
    reply->deleteLater();
}

std::vector<QString> LagTest::discoverComPorts()
{
    std::vector<int> ports;
    std::vector<QString> portsNames;
    char cbuffer[10];

    qDebug("Discovering Serial Ports ...");

    for(int i = 0; i < 16; i++){
        qDebug("Trying %d",i);
        if( !RS232_OpenComport(i, 9600) ){
            ports.push_back(i);
            qDebug("  port %d worked",i);
            RS232_CloseComport( i );
        }
    }

    #ifdef __linux__
        for(std::vector<int>::iterator it = ports.begin(); it != ports.end(); ++it) {
            sprintf(cbuffer, "port %d", (*it));
            portsNames.push_back( QString( cbuffer ) );
        }
    #else
        for(std::vector<int>::iterator it = ports.begin(); it != ports.end(); ++it) {
            sprintf_s(cbuffer, 10, "COM%d", (*it)+1);
            portsNames.push_back( QString( cbuffer ) );
        }
    #endif

    return portsNames;
}

void LagTest::receiveFlashArduino()
{
    this->settings->beginGroup("Arduino");
    settings->value("port");
}

QString LagTest::makeUserSelectPort()
{
    QStringList items;

    std::vector<QString> ports = discoverComPorts();
    for(std::vector<QString>::iterator it = ports.begin(); it != ports.end(); ++it) {
        qDebug("Found Port %s" , (*it).toStdString().c_str() );
        //items << (*it).toStdString().c_str();
        items << (*it);
    }

    QString result;
    bool ok;

    result = QInputDialog::getItem( (this->w != NULL)? w : NULL,
				   QString("lagtest - select Arduino port"),
				   QString("Select the port arduino is connected to"),
				   items,
				   0, // current index
				   true, // editable
				   &ok);

    if (ok) {
      qWarning( "User selected Port %s " , result.toStdString().c_str() );
    } else {
      qDebug( "no selection" );
      result = QString("no selection");
    }
    return result;
}

int LagTest::programArduino(QString avrDudePath, QString pathToFirmware, QString port)
{
    qDebug("Trying to flash arduino ...");
    char buffer[300];
    if( port.isEmpty() ){
        port = makeUserSelectPort();
    }

    sprintf_s(buffer, 300, "-F -v -pm328p -c arduino -b 115200 -P\\\\.\\%s -D -Uflash:w:%s:i", port.toStdString().c_str(), pathToFirmware.toStdString().c_str() );

    QString param(QString::fromLocal8Bit(buffer));
    qDebug("Calling %s with %s" , avrDudePath.toStdString().c_str(), param.toStdString().c_str() );

    QProcess process;
    process.execute (avrDudePath, param.split(" "));

    if( process.exitCode() == 0){
        qDebug("Flashing Arduino was successful!");
    } else {
        qCritical("Flashing Arduino failed! Error code %d" , process.exitCode());
    }

    // Store this settings for the normal program execution
    QSettings* settings = new QSettings("lagtest.ini", QSettings::IniFormat);
    QDir dir;
    settings->setValue("Arduino/Port", port );
    settings->setValue("Arduino/avrDudePath", dir.relativeFilePath( avrDudePath ) );
    settings->setValue("Arduino/firmwarePath", dir.relativeFilePath( pathToFirmware ) );
    settings->sync();
    delete(settings);

    return process.exitCode();
}

bool LagTest::loadSettings()
{
    this->settings = new QSettings("lagtest.ini", QSettings::IniFormat);

    //If we dont have this setting, or the port is not valid, let the user define a new port
    if( !this->testPort( settings->value("Arduino/Port").toString() ))
    {
        qDebug("Port not valid, query user to specify new one.");
        QString nPort;
        do{
            nPort = makeUserSelectPort();
            if( !this->testPort(nPort) ){
                qCritical("Invalid Port!");
                nPort = "";
            }
        } while( nPort.isEmpty() );
        settings->setValue("Arduino/Port", nPort );
        settings->sync();
    }

    return true;

    //Create entries
    //    settings->beginGroup("Arduino");
    //        settings->setValue("Port", "COM11");
    //        settings->setValue("avrDudePath", "tools/avrdude.exe");
    //        settings->setValue("firmwarePath", "firmware.hex");
    //    settings->endGroup();
    //    settings->sync();
}

bool LagTest::testPort(QString port)
{
    int portIdx = LagTestSerialPortComm::getPortIdx(port);

    if( portIdx < 0)
        return false;

    if( !RS232_OpenComport(portIdx, 9600) ){
        RS232_CloseComport( portIdx );
        return true;
    }
    return false;
}
