#include "serialporthandler.h"

#include <QThread>
#include <QCoreApplication>
#include <QVariant>

#ifdef Q_OS_WIN
    #include <windows.h>
#elif defined( Q_OS_LINUX )
    #include <unistd.h>
#else
    ERROR UNDEFINED SYSTEM
#endif


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
        connect(serial, SIGNAL(sendArduinoDetectionFailed()), this, SIGNAL(sendArduinoDetectionFailed()) );

        connect(this, SIGNAL(setLed(bool)), serial, SLOT(onLedSet(bool)) );
        //This is not working, dont know why, cant get signal through to the slot
//        connect(this, SIGNAL(sendVersionCheck()), serial, SLOT(doVersionCheck()) );
//        connect(this, SIGNAL(sendVersionCheck()), serial, SLOT(doSomethingElse()) );
//        connect(this, SIGNAL(sendVersionCheck()), qApp, SLOT(aboutQt()));

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

void SerialPortHandler::doVersionCheck()
{
    //BUG: emited signal want get to slot
    //emit sendVersionCheck();

    this->serial->doVersionCheck();
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

    //sendDebugMsg("SerialPort Handler: Received signal to stop serial loop");
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
    baudRate(baudRate),
    stopThread(false),
    setLedState(0)
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

#ifdef Q_OS_WIN
        if( !portName.contains("COM", Qt::CaseSensitive) )
            idx = -1;
        else
            idx = portName.right(portName.length()-3).toInt() - 1; //From Com11 , extract 11, convert it to int, rs232 starts counting from 0.
#elif defined( Q_OS_LINUX )
        bool success;
        //fprintf(stderr, "Trying to resolve port name %s", portName.toStdString().c_str());
        success = RS232_comportName2Idx(portName.toStdString().c_str(), idx );
        if( !success ){
            fprintf(stderr, "Resolving Port name failed! ");
            idx = -1;
        }
#else
    ERROR UNDEFINED SYSTEM
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


bool LagTestSerialPortComm::blockingRead(unsigned char* data, int size, int maxTimeout)
{
	int nBytes, t;
	double endTime;

#ifdef Q_OS_LINUX
    struct timespec tw, tremain;
	tw.tv_sec = 0;
	tw.tv_nsec = 10 * 1000 * 1000;
#endif

    endTime = this->tm->getCurrentTime() + maxTimeout*1000000.0;
	nBytes = 0;

    while( (nBytes < size) && (endTime > this->tm->getCurrentTime() ) )
	{
		t = this->read(&(data[nBytes]), (size-nBytes) );
		nBytes += t;
		if(t == 0){
			#ifdef Q_OS_WIN
                Sleep( 10.0 );
			#elif defined( Q_OS_LINUX )
				nanosleep( &tw , &tremain);
			#else
				ERROR UNDEFINED SYSTEM
			#endif
		}
	}

	if( nBytes < size ){
		return false;
	} else {
		return true;
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
    //sendDebugMsg("LagTestSerialPortComm: Received signal to stop serial loop");
    this->stopThread = true;
}

void LagTestSerialPortComm::sendClockRequest()
{
    //sendDebugMsg("Received signal to send Time Request ...");
    this->sendRequest = true;
}

void LagTestSerialPortComm::onLedSet( bool ledOn )
{
    if( ledOn )
        this->setLedState = LED_ON;
    else
        this->setLedState = LED_OFF;
}

void LagTestSerialPortComm::startCommunication()
{   
    clockPair cp;
    adcMeasurement adcM; 
    timed_sample_t frame;    
    bool validFrame = false;    
    double sendTime, now, d1;
    int frameCnter = 0;

#ifdef Q_OS_LINUX
    struct timespec tw, tremain;
    tw.tv_sec = 0;
    tw.tv_nsec = 1 * 1000 * 1000;
#endif

    this->stopThread = false;

    sendDebugMsg("Starting Arduino Serial communication ...");

    this->doVersionCheck();
    this->init();

    try{
        this->initSerialPort();
        unsigned char buffer[100];
        if( this->blockingRead(buffer, 9, 2000) == false ) {
        	this->sendDebugMsg("Init read failed!");
        }
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
        if( this->setLedState != INACTIVE)
        {
            QString s;

            if(setLedState == LED_ON){
                //this->sendErrorMsg(s.sprintf("Led ON %g", this->tm->getCurrentTime() ));
                this->sendArduinoLedState(true);
            } else {
                //this->sendErrorMsg(s.sprintf("Led OFF %g", this->tm->getCurrentTime() ));
                this->sendArduinoLedState(false);
            }
            setLedState = INACTIVE;
        }
        try
        {
            if( this->getNextFrame( frame, now ) )
            {
                frameCnter++;

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
                            QString s;
                            //sendDebugMsg( s.sprintf("RTT %g ", (now - sendTime)) );
                            if( d1 <= 0){
                                //qCritical("somethign strange happens here ... %g", d1 );

                                this->sendErrorMsg(s.sprintf("somethign strange happens here ... %g", d1 ));
                                d1 = 0;
                            } else if ( d1 > 10000000 ){
                                sendErrorMsg( s.sprintf("Too high clock diff %g ", d1) );
                            }
                            cp.local = sendTime + d1;
                            cp.arduino_epoch = frame.epoch;
                            cp.arduino_ticks = frame.ticks;
                            this->clock_storage->put( &cp );
                            //qDebug("Received a arduino clock msg for request %d from %g", frame.value, cp.local );
                        }
                        break;
                    }

                    default:{
                        //qDebug( "Unknown msg from arduino: %c;%d;%d;%d", frame.cmd, frame.value, frame.epoch, frame.ticks );
                        break;
                    }
                }
            } else {
                #ifdef Q_OS_WIN
                    Sleep( 1.0 );
                #elif defined( Q_OS_LINUX )
                    nanosleep( &tw , &tremain);
                #else
                    ERROR UNDEFINED SYSTEM
                #endif
            }
        } catch ( InvalidFrameException ){
        	//Nothing special here, invalid frames can happen, specially at the beginning
        } catch ( ReadErrorException ) {
        	emit sendArduinoTimeout();
        }        
    }

    sendDebugMsg("Stoping Arduino Serial communication ...");
    this->closeSerialPort();  
}

void LagTestSerialPortComm::doVersionCheck()
{
    bool gotVersionReply;    
    int garbageCnt;
    timed_sample_t frame;
    double now;

#ifdef Q_OS_LINUX
    struct timespec tw, tremain;
    tw.tv_sec = 0;
    tw.tv_nsec = 1 * 1000 * 1000;
#endif

    sendDebugMsg("Starting Arduino Version Check ...");

    this->init();

    try{
        this->initSerialPort();
        unsigned char buffer[20];
        if( this->blockingRead(buffer, 9, 2000) == false ) {
            this->sendDebugMsg("Init read failed 2!");
        }
    } catch( ... ) {
        this->sendErrorMsg("Opening Serial Port failed!");
        emit finished();
    }

    garbageCnt = 0;
    gotVersionReply = false;
    while( (!gotVersionReply) && (garbageCnt < 100) )
    {
        this->sendArduinoVersionRequest();

        try
        {
            if( this->getNextFrame( frame, now ) )
            {
                switch(frame.cmd)
                {
                    case 'V':   //Version Response
                    {
                        gotVersionReply = true;
                        break;
                    }
                    default:{
                        garbageCnt++;
                        break;
                    }
                }
            } else {
                #ifdef Q_OS_WIN
                    Sleep( 1.0 );
                #elif defined( Q_OS_LINUX )
                    nanosleep( &tw , &tremain);
                #else
                    ERROR UNDEFINED SYSTEM
                #endif
            }
        } catch ( InvalidFrameException ){
        	garbageCnt++;
        } catch ( ReadErrorException ){
        	sendDebugMsg( "Stop version check, cant read from Arduino" );
			this->closeSerialPort();
			emit sendArduinoTimeout();
			return;
        }        
    }

    sendDebugMsg( "Finished version check" );
    this->closeSerialPort();

    if( !gotVersionReply ){
        emit sendArduinoDetectionFailed();
    } else {
        emit sendFirmwareVersion( frame.value );
    }
}

void LagTestSerialPortComm::sendArduinoLedState(bool ledOn)
{
    unsigned char b[2];
    b[0] = 'L';
    b[1] = (ledOn ? 1 : 0);

    this->write(b, 2);
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

bool LagTestSerialPortComm::getNextFrame( timed_sample_t& frame, double& timeRead )
{
    const int frameLength = 9;
    const int bufferSize = 100;
    static uint8_t buffer[bufferSize];
    static int nBuffer = 0;
    static int nEmptyReads = 0;
    int i;
    bool validFrame;
    int t;

    t = this->read(&(buffer[nBuffer]), bufferSize-nBuffer );

    if( t == 0)
    {
        nEmptyReads ++;
        if( nEmptyReads > 1000 ){
        	nEmptyReads = 0;
            throw ReadErrorException();
        }

    } else {
        nEmptyReads = 0;
        nBuffer += t;

        if( nBuffer >= frameLength )
        {
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

            if( !validFrame ){
                throw InvalidFrameException();
            }

            return true;
        }
    }

    return false; //No valid frame received
}


bool LagTestSerialPortComm::decode2Frame(uint8_t* buffer, timed_sample_t* frame)
{
    //Check if the checksum is valid and if so, fill the data structure
    uint8_t cs;
    cs = *(buffer+1)+ *(buffer+2)+ *(buffer+3)+ *(buffer+4)+ *(buffer+5) + *(buffer+6) + *(buffer+7);

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
