#include "serialporthandler.h"

#include <QThread>
#include <QCoreApplication>
#include <QVariant>


#ifdef LINUX
#include <unistd.h>
#endif
#ifdef _WINDOWS
#include <windows.h>
#endif

void mySleep(int sleepMs)
{
  // From http://stackoverflow.com/questions/10918206/cross-platform-sleep-function-for-c
#ifdef LINUX
  usleep(sleepMs * 1000);   // usleep takes sleep time in us
#endif
#ifdef _WINDOWS
  Sleep(sleepMs);
#endif
}



SerialPortHandler::SerialPortHandler(QString port, int requestPeriod, TimeModel *tm, RingBuffer<clockPair> *clock_storage, RingBuffer<adcMeasurement> *adc_storage) :
    QObject(0),
    thread(NULL),
    port(port),
    requestPeriod(requestPeriod),
    tm(tm),
    clock_storage(clock_storage),
    adc_storage(adc_storage)
{
    //Start a thread and execute doProcessing()
    this->createSerialPortCommunicator();
}

void SerialPortHandler::createSerialPortCommunicator()
{
    try{
        this->serial = new LagTestSerialPortComm( this->port, 115200, this->tm, this->clock_storage, this->adc_storage );

        // Setup period query for arduino clock
        this->timer = new QTimer(this);
        connect(timer, SIGNAL(timeout()), serial, SLOT(sendClockRequest()));
        timer->setInterval(requestPeriod);

        this->thread = new QThread();
        serial->moveToThread(thread);
        connect(thread, SIGNAL(started()), serial, SLOT(startCommunication()));
        connect(serial, SIGNAL(finished()), this, SLOT(onThreadQuit()));
        connect(serial, SIGNAL(finished()), serial, SLOT(deleteLater()));

        //Forward signals
        connect(serial, SIGNAL(sendDebugMsg(QString)), this, SIGNAL(sendDebugMsg(QString)) );
        connect(serial, SIGNAL(sendErrorMsg(QString)), this, SIGNAL(sendErrorMsg(QString)) );
        connect(serial, SIGNAL(sendFirmwareVersion(int)), this, SIGNAL(sendFirmwareVersion(int)) );
        connect(serial, SIGNAL(sendArduinoTimeout()), this, SIGNAL(sendArduinoTimeout()) );



    } catch (exception &e){
        //qCritical("Connecting to the lagtest serial board failed!");
        this->sendErrorMsg("Connecting to the lagtest serial board failed!");
        throw e;
    }
}

void SerialPortHandler::onThreadQuit()
{
    //qDebug("Main: Serial Communicator finished ..." );
    //qDebug("Will restart serial port Communicator!");
    this->sendDebugMsg("Will restart serial port Communicator!");

    try
    {
        delete this->serial;
        delete this->thread;
        delete this->timer;

        this->createSerialPortCommunicator();
    } catch (exception &e){
        //qCritical("Restarting Serial Port Communicator failed!");
        this->sendErrorMsg("Restarting Serial Port Communicator failed!");
        throw e;
    }
}

void SerialPortHandler::start()
{
    if(this->thread){
        this->sendDebugMsg("Will start serial port Communicator!");
        this->thread->start();
        this->timer->start();
    }
}

void SerialPortHandler::stop(){
    this->serial->recvStop();
    this->timer->stop();
    this->thread->terminate();
}

#define _CRT_SECURE_NO_WARNINGS
#include "rs232.h"

LagTestSerialPortComm::LagTestSerialPortComm(QString port, int baudRate,TimeModel* tm, RingBuffer<clockPair>* clock_storage, RingBuffer<adcMeasurement>* adc_storage) :
    QObject(0),
    tm(tm),
    clock_storage(clock_storage),
    adc_storage(adc_storage),
    sendRequest(false),
    baudRate(baudRate)
{
    this->portN = this->getPortIdx(port);
    if( portN < 0)
    {
        //qCritical("Invalid Com port [%s]!", port.toStdString().c_str() );
        QString s;
        this->sendErrorMsg( s.sprintf("Invalid Com port [%s]!", port.toStdString().c_str()));
        throw std::exception();
    }   
}

void LagTestSerialPortComm::init()
{
    this->tR = 0;
    this->clock_storage->reset();
    this->adc_storage->reset();
}

int LagTestSerialPortComm::getPortIdx(QString portName)
{
    int idx = -1;

    if( portName.length() >= 4)
    {

#ifdef _WINDOWS
    if( !portName.contains("COM", Qt::CaseSensitive) )
        idx = -1;
    else
        idx = portName.right(portName.length()-3).toInt() - 1; //From Com11 , extract 11, convert it to int, rs232 starts counting from 0.
#endif
    
#ifdef LINUX
    bool success;
    //fprintf(stderr, "Trying to resolve port name %s", portName.toStdString().c_str());    
    success = RS232_comportName2Idx(portName.toStdString().c_str(), idx );
    if( !success ){
        fprintf(stderr, "Resolving Port name failed! ");
        idx = -1;
    }
#endif

    }
    return idx;
}

void LagTestSerialPortComm::closeSerialPort()
{
    RS232_CloseComport( portN );
}

void LagTestSerialPortComm::initSerialPort()
{
    QString s;
    this->sendDebugMsg(s.sprintf("Opening Com port %d ... ", portN+1));
    if(RS232_OpenComport(portN, this->baudRate))
    {
        this->sendErrorMsg("Can not open Serial port.");
        throw std::exception();
    }
}

int LagTestSerialPortComm::write(unsigned char* data, int size){
    return RS232_SendBuf(this->portN, data, size);
}

int  LagTestSerialPortComm::read(unsigned char* buffer, int max_size){
    return RS232_PollComport(this->portN, buffer, max_size);
}

void LagTestSerialPortComm::recvStop()
{
    this->stopThread = true;
}

void LagTestSerialPortComm::sendClockRequest()
{
    //sendDebugMsg("Received signal to send Time Request ...");
    this->sendRequest = true;
}

void LagTestSerialPortComm::startCommunication()
{   
    clockPair cp;
    adcMeasurement adcM; 
    timed_sample_t frame;    
    bool validFrame = false;    
    double sendTime, now, d1;
    int frameCnter = 0;

    this->stopThread = false;

    sendDebugMsg("Starting Arduino Serial communication ...");

    this->init();

    try{
        this->initSerialPort();
        mySleep( 10 ) ;
    } catch( ... ) {
        this->sendErrorMsg("Opening Serial Port failed!");
        emit finished();
    }

    while(!this->stopThread)
    {
        QCoreApplication::processEvents();  //If this is not called, no signals can be received by this thread

        //If ordered, send an request to the arduino to return its current clock value
        if( this->sendRequest )
        {
            this->sendArduinoTimeRequest();
            this->sendRequest = false;
        }

        if( frameCnter == 10 ){
            this->sendArduinoVersionRequest();
        }

        validFrame = this->getNextFrame( frame, now );
        frameCnter++;

        if( validFrame )
        {
            //qDebug( " Frame: %c;%d;%d;%d", frame.cmd, frame.value, frame.epoch, frame.ticks );

            switch(frame.cmd)
            {
                case 'H'://ADC measurement
                {
                    adcM.adc = frame.value;
                    adcM.arduino_epoch = frame.epoch;
                    adcM.arduino_ticks = frame.ticks;
                    this->adc_storage->put( &adcM );
                    //qDebug("Received a adc measurement");
                    break;
                }
                case 'P': //Clock response
                {
                //Get the time the request was send; assume the latency of receiving the reply is symetrical; store the time on this pc and the arduino clock
                    if( frame.value > this->ntimeRequests ){
                        //qCritical("Arduino returns invalid reference id!");
                        this->sendErrorMsg("Arduino returns invalid reference id!");
                    } else {
                        sendTime = this->timeRequests[ frame.value ];
                        d1 = (now - sendTime)/2.0;
                        if( d1 <= 0){
                            //qCritical("somethign strange happens here ... %g", d1 );
                            QString s;
                            this->sendErrorMsg(s.sprintf("somethign strange happens here ... %g", d1 ));
                            d1 = 0;
                        }
                        cp.local = sendTime + d1;
                        cp.arduino_epoch = frame.epoch;
                        cp.arduino_ticks = frame.ticks;
                        this->clock_storage->put( &cp );
                        //qDebug("Received a arduino clock msg for request %d from %g", frame.value, cp.local );
                    }
                    break;
                }

                case 'V':   //Version Response
                {
                    emit sendFirmwareVersion( frame.value );
                    break;
                }

                default:{
                    //qDebug( "Unknown msg from arduino: %c;%d;%d;%d", frame.cmd, frame.value, frame.epoch, frame.ticks );
                }
            }
        } else {
            //qDebug( " Invalid Frame ... ");
            emit sendErrorMsg("Invalid Frame ... ");
        }
    }

    sendDebugMsg("Stoping Arduino Serial communication ...");
    this->closeSerialPort();
}

void LagTestSerialPortComm::sendArduinoVersionRequest()
{
    unsigned char b[2];
    b[0] = 'V';
    b[1] = 0;

    this->write(b, 2);
}

void LagTestSerialPortComm::sendArduinoTimeRequest()
{
    unsigned char b[2];

    tR = (tR+1)%this->ntimeRequests;

    b[0] = 'P';
    b[1] = this->tR;

    this->write(b, 2);
    this->timeRequests[tR] = this->tm->getCurrentTime();
}

int LagTestSerialPortComm::readFrameFromSerial(uint8_t* buffer, int frameLength, int bufferSize)
{
    int nEmptyReads = 0;
    int nReadBytes = 0;
    int t;
    //Read from the serial port at least one complete message
    while( (nReadBytes < frameLength) && (nEmptyReads < 2000) )
    {
        t = this->read(&(buffer[nReadBytes]), (bufferSize-nReadBytes) );
        nReadBytes += t;
        if(t == 0){
            nEmptyReads ++;
            mySleep( 1 ) ;
        } else {
            nEmptyReads = 0;
        }
    }

    if( nEmptyReads >= 2000 ){
        emit sendArduinoTimeout();
        return 0;
    }

    return nReadBytes;
}


bool LagTestSerialPortComm::getNextFrame( timed_sample_t& frame, double& timeRead )
{
    const int frameLength = 9;
    const int bufferSize = 100;
    static uint8_t buffer[bufferSize];
    static int nBuffer = 0;
    int i;
    bool validFrame;

    nBuffer += this->readFrameFromSerial( &(buffer[nBuffer]), frameLength, bufferSize-nBuffer) ;

    //Try to read a new frame from the beginning of the buffer
    //If the frame checksum fails, do a frame shift and try again
    validFrame = false;
    i = 0;
    //do frame shifts untill there cannot be a full frame in the buffer
    while( !validFrame && (i+frameLength) <= nBuffer )
    {
        validFrame = decode2Frame(&buffer[i], &frame);

        if( validFrame ) {
           timeRead = this->tm->getCurrentTime();
           i += frameLength;
        } else {
            i++; //Shift the frame by one
        }
    }
    // i ... number of bytes read/garbage in buffer

    //Move read/garbage data to the beginning of the buffer
    assert ( i <= nBuffer );
    for(int j = 0; j < (nBuffer-i) ; j++){
        buffer[j] = buffer[j+i];
    }
    nBuffer -= i;
    assert( nBuffer >= 0 );

    return validFrame;
}


bool LagTestSerialPortComm::decode2Frame(uint8_t* buffer, timed_sample_t* frame)
{
    //Check if the checksum is valid and if so, fill the data structure
    uint8_t cs;
    cs = *(buffer+1)+ *(buffer+2)+ *(buffer+3)+ *(buffer+4)+ *(buffer+5) + *(buffer+6) + *(buffer+7);;
//    for(int i=0; i < 6; i++){
//        cs += *(buffer+1 + i);
//    }
    if( cs == *(buffer+8))
    {
        frame->cmd =    *(buffer+0);
        frame->value =  *(buffer+1);
        frame->epoch = (*(buffer+5) << 24) + (*(buffer+4) << 16) + (*(buffer+3) << 8) + *(buffer+2);
        frame->ticks = (*(buffer+7) << 8) + *(buffer+6);
        frame->checksum = *(buffer+8);
        return true;
    } else{
        //qDebug("Invalid Frame (%u) (%u) (%u;%u;%u;%u) (%u;%u;) (%u) | CS %u", *(buffer+0), *(buffer+1), *(buffer+2), *(buffer+3), *(buffer+4), *(buffer+5), *(buffer+6), *(buffer+7), *(buffer+8) , cs);
        return false;
    }
}
