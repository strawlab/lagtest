#include "latencymodel.h"
#include <limits>
#include <stdlib.h>
#include <math.h>

LatencyModel::LatencyModel(int ms_updateRate, TimeModel *tm, RingBuffer<screenFlip> *screenFlips, RingBuffer<clockPair> *clock_storage, RingBuffer<adcMeasurement> *adc_storage)
    : QObject(0),
	tm(tm), screenFlips(screenFlips), clock(clock_storage), adc(adc_storage),
      timer(0),
      latencyCnt(0), flipCnt(0)
{
    qDebug("Creating LatencyModel ...");
    // Setup period update of the model
	this->timer = new QTimer();
    connect( timer, SIGNAL(timeout()), this, SLOT( update() ));
    this->timer->setInterval(ms_updateRate);
    this->resetHistory();
}

void LatencyModel::stop()
{
    this->timer->stop();    
}

void LatencyModel::start()
{
    this->resetHistory();
    QTimer* t = new QTimer();
    t->setSingleShot(true);
    connect(t, SIGNAL(timeout()), this, SLOT( realStart()) );
    t->start(2000); //start two seconds later
}

void LatencyModel::realStart()
{
    this->measurementStartTime = tm->getCurrentTime();
    qDebug("Starting latency model @%g" , this->measurementStartTime );
    this->screenFlips->reset();    
    this->resetHistory();
    this->timer->start();
}

void LatencyModel::resetHistory()
{
    qDebug("Reseting Latency Model History ...");
    //Initialize latency history
    for( int i=0; i < LatencyModel::latencyHistorySize; i++)  {
        this->latency[i] = -1.0;
    }

    for( int j=0; j < measurementWindowSize; j++ )
    {
        this->avgAdcWindow[WHITE_TO_BLACK][j] = 0;
        this->avgAdcWindow[BLACK_TO_WHITE][j] = 0;
        this->sampleTimes[j] = 0.0;
    }

    for( int i=0; i < LatencyModel::measurementHistoryLength; i++)
    {
        for( int j=0; j < measurementWindowSize; j++ )
        {
            this->adcData[WHITE_TO_BLACK][i][j] = -1;
            this->adcData[BLACK_TO_WHITE][i][j] = -1;
        }
        emit signalNewMeasurementWindow( this->adcData[WHITE_TO_BLACK][i], this->avgAdcWindow[WHITE_TO_BLACK], this->sampleTimes, WHITE_TO_BLACK );
        emit signalNewMeasurementWindow( this->adcData[BLACK_TO_WHITE][i], this->avgAdcWindow[BLACK_TO_WHITE], this->sampleTimes, BLACK_TO_WHITE );
    }
    emit signalUpdate(this);

    this->measurementCnter[BLACK_TO_WHITE] = -1;
    this->measurementCnter[WHITE_TO_BLACK] = -1;

    this->nMeasurements[BLACK_TO_WHITE] = 0;
    this->nMeasurements[WHITE_TO_BLACK] = 0;

    this->lastLatency = -1.0;
    this->avgLatency = -1.0;
    this->avgLatencySD = 0.0;
    this->allLatencies.clear();
    this->measurementStartTime = this->tm->getCurrentTime();

}

void LatencyModel::reset()
{
    this->resetHistory();
}

void LatencyModel::update()
{
    double latency;

    qDebug("Latency Update IN @%g: RingBuffers #Clocks %d , #Screen Fllips %d , #ADC %d" , this->tm->getCurrentTime() , clock->unread(), screenFlips->unread(), adc->unread() );

    //Read all new clock pairs and use them to update the time model
    clockPair cp;
    while(this->clock->canGet()){
        this->clock->get(&cp);
        this->tm->update(cp);
    }

    latency = -1.0;

    while( this->screenFlips->canGet() )
    {
        this->screenFlips->get(&(this->flips[this->flipCnt]) );

        if( this->findMeasurementWindow( this->flips[this->flipCnt] ) )
        {
            latency = this->calculateLatency( );
            //qDebug("Latency [%g]", latency);

            this->addLatency( latency );
            flipCnt = (flipCnt+1) % flipHistorySize;
        } else {
            //Leave this flip for the next time arround
            this->screenFlips->unget();
            break;
        }
    }

    if( latency != -1.0 )
    {
        if( this->isStable( this->latencyStableSize ) ){
            emit signalStableLatency();
        } else {
            emit signalUnstableLatency();
        }
        emit signalUpdate(this);
    }

    qDebug("Latency Update OUT @%g: RingBuffers #Clocks %d , #Screen Fllips %d , #ADC %d" , this->tm->getCurrentTime() , clock->unread(), screenFlips->unread(), adc->unread() );
}

void LatencyModel::addLatency(double newLatency)
{
    double t;
    latencyCnt = (latencyCnt+1) % latencyHistorySize;
    this->latency[this->latencyCnt] = newLatency;

    if( ( this->nMeasurements[BLACK_TO_WHITE]+this->nMeasurements[WHITE_TO_BLACK]) < latencyHistorySize )
        return;

    this->lastLatency = newLatency;
    t = 0;
    for(int i=0; i < latencyHistorySize; i++){
        t += latency[i];
    }
    this->avgLatency = t / latencyHistorySize;

    t = 0;
    for(int i=0; i < latencyHistorySize; i++){
        t += sqrt( (latency[i]-avgLatency) * (latency[i]-avgLatency) );
    }

    this->avgLatencySD = t / latencyHistorySize;

    this->allLatencies.push_back( newLatency );
}

double LatencyModel::calculateLatency()
{
    //Calculate mean values
    int j, idx;
    double latency;

    if( this->detectdisplacedSensor() )
    {
        emit signalInvalidLatency();
        return -1;
    }

    this->createAvgWindow();

    idx = -1;
    for( j=0; j < measurementWindowSize; j++ )
    {
        if( avgAdcWindow[WHITE_TO_BLACK][j] < avgAdcWindow[BLACK_TO_WHITE][j]){
            idx = j;
            //qDebug("Found a crossofer at idx %d", idx);
            break;
        }
    }

    if( (idx == 0) || (idx==measurementWindowSize) )
    {
        qWarning("Could not detect crossover!");
        latency = -1;
        emit signalInvalidLatency();
    } else {
        latency = this->sampleTimes[idx];
    }

    return latency;
}

void LatencyModel::createAvgWindow()
{
    int i,j,n[2];
    //char buffer[2000];

    n[WHITE_TO_BLACK] = nMeasurements[WHITE_TO_BLACK];
    n[BLACK_TO_WHITE] = nMeasurements[BLACK_TO_WHITE];

    if( n[WHITE_TO_BLACK] > measurementHistoryLength)
        n[WHITE_TO_BLACK] = measurementHistoryLength;

    if( n[BLACK_TO_WHITE] > measurementHistoryLength)
        n[BLACK_TO_WHITE] = measurementHistoryLength;

    for(i = 0; i < measurementWindowSize; i++){
        this->avgAdcWindow[WHITE_TO_BLACK][i] = 0.0;
        this->avgAdcWindow[BLACK_TO_WHITE][i] = 0.0;
    }

    //char buffer[2000];

    for( i=0; i < measurementHistoryLength; i++)
    {
        for( j=0; j < measurementWindowSize; j++ )
        {
            if(n[WHITE_TO_BLACK] > i) {
                avgAdcWindow[WHITE_TO_BLACK][j] += this->adcData[WHITE_TO_BLACK][i][j];
            }

            if(n[BLACK_TO_WHITE] > i) {
                avgAdcWindow[BLACK_TO_WHITE][j] += this->adcData[BLACK_TO_WHITE][i][j];
            }
        }

//        buffer[0] = 0;
//        sprintf(buffer,"W2B Sample [%d]\n", i);
//        for(int k = 0; k < measurementWindowSize; k++){
//            sprintf( &buffer[strlen(buffer)], " [%d] ", (this->adcData[WHITE_TO_BLACK][i][k]) );
//        }
//        qDebug(buffer);

//        buffer[0] = 0;
//        sprintf(buffer,"B2W Sample [%d]\n", i);
//        for(int k = 0; k < measurementWindowSize; k++){
//            sprintf( &buffer[strlen(buffer)], " [%d] ", (this->adcData[BLACK_TO_WHITE][i][k]) );
//        }
//        qDebug(buffer);
//        qDebug("#############");

    }

    for( j=0; j < measurementWindowSize; j++ )
    {
        avgAdcWindow[WHITE_TO_BLACK][j] = avgAdcWindow[WHITE_TO_BLACK][j] / n[WHITE_TO_BLACK];
        avgAdcWindow[BLACK_TO_WHITE][j] = avgAdcWindow[BLACK_TO_WHITE][j] / n[BLACK_TO_WHITE];
    }

//    buffer[0] = 0;
//    sprintf(buffer,"Avg Window n = [%d / %d]\n", n[0], n[1]);
//    for(i = 0; i < measurementWindowSize; i++){
//        sprintf( &buffer[strlen(buffer)], " [%g] ", (this->avgAdcWindow[WHITE_TO_BLACK][i]) );
//    }
//    qDebug(buffer);

}

static int cmpDouble(const void *p1, const void *p2)
{
	double dif;
	dif = (*(double*)p1) - (*(double*)p2) ;
 
	if( dif < 0.0)
		return -1;
	else if( dif > 0.0)
		return 1;
	else
		return 0;
}

//Calculate the avg and standart derivation for the last measurement window
//Average the 5 lowerst and highest values of the last measurement window.
//If min >= max than we assume the sensor is displaced
bool LatencyModel::detectdisplacedSensor()
{
    int j;
    double min[2] = {0,0};
    double max[2] = {0,0};
    double sorted[2][measurementWindowSize];    

    for( j=0; j < measurementWindowSize; j++ )
    {
    	sorted[WHITE_TO_BLACK][j] = this->adcData[WHITE_TO_BLACK][this->measurementCnter[WHITE_TO_BLACK]][j];
    	sorted[BLACK_TO_WHITE][j] = this->adcData[BLACK_TO_WHITE][this->measurementCnter[BLACK_TO_WHITE]][j];
    }

    qsort(sorted[WHITE_TO_BLACK], measurementWindowSize, sizeof(double), cmpDouble);
    qsort(sorted[BLACK_TO_WHITE], measurementWindowSize, sizeof(double), cmpDouble);
    
    
    for(j=0; j < 5; j++){
    	min[WHITE_TO_BLACK] += sorted[WHITE_TO_BLACK][j];
    	max[WHITE_TO_BLACK] += sorted[WHITE_TO_BLACK][measurementWindowSize-1-j];
    	
    	min[BLACK_TO_WHITE] += sorted[BLACK_TO_WHITE][j];
    	max[BLACK_TO_WHITE] += sorted[BLACK_TO_WHITE][measurementWindowSize-1-j];
    }
    min[WHITE_TO_BLACK] = ceil ( min[WHITE_TO_BLACK] / 5.0 );
    max[WHITE_TO_BLACK] = floor ( max[WHITE_TO_BLACK] / 5.0 );
	
	min[BLACK_TO_WHITE] = ceil ( min[BLACK_TO_WHITE] / 5.0 );
	max[BLACK_TO_WHITE] = floor ( max[BLACK_TO_WHITE] / 5.0 );
	
	if( min[WHITE_TO_BLACK] >= max[WHITE_TO_BLACK] ){
		qDebug("Measurement Window seems to flat. Mins [%f/%f] , Max [%f/%f]", min[WHITE_TO_BLACK] , min[BLACK_TO_WHITE], max[WHITE_TO_BLACK], max[BLACK_TO_WHITE]);
		return true;
	}
	else if( min[BLACK_TO_WHITE] >= max[BLACK_TO_WHITE] ){
		qDebug("Measurement Window seems to flat. Mins [%f/%f] , Max [%f/%f]", min[WHITE_TO_BLACK] , min[BLACK_TO_WHITE], max[WHITE_TO_BLACK], max[BLACK_TO_WHITE]);
		return true;
	} else {
		return false;
	}
}

bool LatencyModel::findMeasurementWindow(screenFlip sf )
{
    bool found = false;
    adcMeasurement s;

    //qDebug("Trying to find a flip at %g", sf.local);
    //Consume all adc sample values till we get one taken after the flip
    while( !found && this->adc->canGet())
    {
        this->adc->get(&s);

        //last entry in the ringbuffer is the sample closest in time to the screen flip
        if( this->tm->toLocalTime(s) > sf.local ){
            found = true;
            qDebug("For flip at %g using measurements starting from %g. Following %d unread elements" , sf.local, this->tm->toLocalTime(s), this->adc->unread() );
        }
    }

    if( !found || (this->adc->unread() < measurementWindowSize) )
    {   //Could not find a sample close to the screen flip
        qWarning("Not enought adc sample data!");
        found = false;
    } else {
        //The WindowSize Previous elements in the adc ringbuffer are part of this sample window
        int i;
        adcMeasurement tsample;

        this->nMeasurements[sf.type] ++;
        this->measurementCnter[ sf.type ] = (this->measurementCnter[ sf.type ]+1) % measurementHistoryLength;

        this->adcData[sf.type][this->measurementCnter[sf.type]][0] = s.adc;
        this->sampleTimes[0] = this->tm->toLocalTime( s ) - sf.local;

        for(i=1; i < measurementWindowSize; i++){
            if( this->adc->get( &tsample  ) )
            {
                this->adcData[sf.type][this->measurementCnter[sf.type]][i] = tsample.adc;
                this->sampleTimes[i] = this->tm->toLocalTime( tsample ) - sf.local;
                //qDebug("E: %g, T: %g, D: %d", tsample.arduino_epoch, tsample.arduino_ticks, tsample.adc);
            } else {
                qCritical("Not enough data in the ADC ringbuffer");
            }

        }
        //if( nMeasurements[sf.type] > measurementHistoryLength ){
        emit signalNewMeasurementWindow( this->adcData[sf.type][this->measurementCnter[sf.type]], this->avgAdcWindow[sf.type], this->sampleTimes, sf.type );
        //}
    }

//    char buffer[10000];
//    buffer[0] = 0;
//    sprintf(buffer," %s Measurement Window for flip %g\n" , (sf.type == BLACK_TO_WHITE)?"B2W":"W2B", sf.local);
//    for(int k = 0; k < measurementWindowSize; k++){
//        sprintf( &buffer[strlen(buffer)], " [%d] ", (adcData[sf.type][this->measurementCnter[sf.type]][k]) );
//    }
//    qDebug(buffer);

//    char buffer[10000];
//    buffer[0] = 0;
//    sprintf(buffer,"Timetable of adc Sample\n");
//    for(int k = 0; k < measurementWindowSize; k++){
//        sprintf( &buffer[strlen(buffer)], " [%g] ", (this->sampleTimes[k]) );
//    }
//    qDebug(buffer);

    return found;
}

bool LatencyModel::isStable(int stablePeriod)
{
    double diff;
    double t;
    double mean;
    bool isStable = false;
    t = 0;
    int prev;

    //If the last 3 latency values dont differ in more than 10% the result is stable
    mean = 0;
    prev = this->latencyCnt;
    isStable = true;
    for(int i = 0; i < stablePeriod; i++)
    {
        prev = (prev == 0) ? latencyHistorySize-1 : prev-1 ;
        t = this->latency[prev];

        if( mean != 0)
        {
            diff = sqrt( (t-mean)*(t-mean) );
            if( diff > 0.1*mean ){
                isStable = false;
                break;
            } else {
                mean = ( mean + t ) / 2.0;
            }
        } else {
            mean = t;
        }
    }

    return isStable;
}
