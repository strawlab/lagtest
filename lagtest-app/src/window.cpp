//#include "glwidget.h"
#include <QWidget>
#include "window.h"

#include <QGridLayout>
#include <QLabel>
#include <QTimer>

#include <qwt_plot.h>
#include <qwt_plot_curve.h>
#include <qwt_legend.h>
#include <QMenuBar>

#include <QCoreApplication>
#include "latencymodel.h"

#include <QUrl>
#include <QDesktopServices>
#include <QMessageBox>
#include <QPainter>
#include <QFont>

//Window::Window(enum drawingType drawing, TimeModel *tm, RingBuffer<screenFlip> *screenFlips)
Window::Window(TimeModel *tm, RingBuffer<screenFlip> *screenFlips) :
    showPlot(false) , isRunning(false)
{
    qDebug("Creating main window ...");
    QWidget* flipWindow;
	enum drawingType drawing = Window::QPAINT;
    setWindowTitle("Lagtest - How fast is your display");
    this->resize(640, 480);

    QVBoxLayout *layout = new QVBoxLayout;
    this->msg = new QLabel("Put the light sensor ontop of the white area. When ready press the space bar.");
    this->latency = new QLabel("");

    msg->setAlignment(Qt::AlignHCenter);
    latency->setAlignment(Qt::AlignHCenter);

    QHBoxLayout* buttonLayout = new QHBoxLayout;

    layout->addWidget(msg, 0);
    layout->addWidget(latency, 0);

    flipWindow = new flashingBGQPaint(500, tm, screenFlips);

    connect( this, SIGNAL(startMeasurement()), flipWindow, SLOT(receiveStart()) );
    connect( this, SIGNAL(startMeasurement()), this, SLOT(recvStartMeasurement()) );
    connect( this, SIGNAL(stopMeasurement()), this, SLOT(recvStopMeasurement()) );
    connect( this, SIGNAL(stop()), flipWindow, SLOT(receiveStop()) );


    flipWindow->resize(150, 140);
    flipWindow->show();
    layout->addWidget(flipWindow, 2);

    layout->setMenuBar( this->createMenu() );
    this->createPlots();

    this->setLayout(layout);

    qDebug("Main Window done!");
}


QMenuBar* Window::createMenu()
{
    //Create Menu
    QMenuBar* menuBar = new QMenuBar;

    QMenu* fileMenu = new QMenu(tr("&File"), this);
    QAction* writeProtocolAction = fileMenu->addAction(tr("&Create Report"));
    QAction* exitAction = fileMenu->addAction(tr("E&xit"));
    menuBar->addMenu(fileMenu);

    QMenu* optionsMenu = new QMenu(tr("&Options"), this);
    QAction* selectPortAction = optionsMenu->addAction(tr("Select &Port"));
    QAction* flashAction = optionsMenu->addAction(tr("Fl&ash Arduino"));
    QAction* plotAction = optionsMenu->addAction(tr("Show &graph"));
    QAction* showLogAction = optionsMenu->addAction(tr("Show &Logs"));
    menuBar->addMenu(optionsMenu);

    QMenu* helpMenu = new QMenu(tr("&Help"), this);
    QAction* helpPageAction = helpMenu->addAction(tr("Goto &Introduction Page"));
    QAction* aboutAction = helpMenu->addAction(tr("&About"));
    menuBar->addMenu(helpMenu);

    connect(exitAction, SIGNAL(triggered()), this, SLOT(quit()));
    connect(flashAction, SIGNAL(triggered()), this, SIGNAL(flashArduino()));
    connect(writeProtocolAction, SIGNAL(triggered()), this, SIGNAL(generateReport()));
    connect(helpPageAction, SIGNAL(triggered()), this, SLOT(recvOpenHelpPage()));
    connect(plotAction, SIGNAL(triggered()), this, SLOT(rcvTogglePlot()));
    connect(aboutAction, SIGNAL(triggered()), this, SLOT( rcvShowAbout()) );
    connect(showLogAction, SIGNAL(triggered()), this, SIGNAL(showLogWindow()));
    connect(selectPortAction, SIGNAL(triggered()), this, SIGNAL(selectPort()));


    return menuBar;
}


void Window::createPlots()
{

    qDebug("Preparing plotting");
    this->xData = (double*) malloc(sizeof(double) * LatencyModel::measurementWindowSize );
    this->yData = (double*) malloc(sizeof(double) * LatencyModel::measurementWindowSize);

    this->plot = new SubWindow(this);
    QVBoxLayout *plotLayout = new QVBoxLayout;

    this->cPlots[BLACK_TO_WHITE] = new QwtPlot( QwtText("Black to White") );
    this->cPlots[WHITE_TO_BLACK] = new QwtPlot( QwtText("White to Black") );

    cPlots[BLACK_TO_WHITE]->setCanvasBackground(QBrush(Qt::lightGray));
    cPlots[WHITE_TO_BLACK]->setCanvasBackground(QBrush(Qt::lightGray));

    cPlots[WHITE_TO_BLACK]->setAxisScale(QwtPlot::xBottom, 0, 100); cPlots[WHITE_TO_BLACK]->replot();
	cPlots[BLACK_TO_WHITE]->setAxisScale(QwtPlot::xBottom, 0, 100); cPlots[BLACK_TO_WHITE]->replot();
    //cPlots[BLACK_TO_WHITE]->setFooter( "Latency [ms]" );
    cPlots[BLACK_TO_WHITE]->setAxisTitle( QwtPlot::xBottom, "Latency [ms]" );
    cPlots[WHITE_TO_BLACK]->setAxisTitle( QwtPlot::xBottom, "Latency [ms]" );
    //cPlots[WHITE_TO_BLACK]->setFooter( "Latency [ms]" );

    //QwtLegend* l = new QwtLegend();
    //QwtLegend* l2 = new QwtLegend();
    //l->setWindowTitle( "ADC" );
    //l2->setWindowTitle( "ADC" );
    //cPlots[BLACK_TO_WHITE]->insertLegend( l );
    //cPlots[WHITE_TO_BLACK]->insertLegend( l2 );


    updateCurveIdx[BLACK_TO_WHITE] = 0;
    updateCurveIdx[WHITE_TO_BLACK] = 0;

    this->nCurves = LatencyModel::measurementHistoryLength;
    int colors[] = { Qt::yellow, Qt::darkYellow, Qt::gray, Qt::darkGray, Qt::black, Qt::green, Qt::darkGreen, Qt::cyan, Qt::darkCyan, Qt::magenta, Qt::darkMagenta };
    int nColors = sizeof(colors)/sizeof(int);
    std::vector<QColor> colorObj;

    for( int i=0; i < nColors; i++){
        QColor c((Qt::GlobalColor) colors[i%nColors]);
        c.setAlpha( 100 );
        colorObj.push_back( c );
    }
    //qDebug("nColors %d", nColors);

    QPen p;

    p.setStyle(Qt::DashLine);
    p.setWidth( 3 );
    p.setColor( Qt::blue );
    this->vLine[WHITE_TO_BLACK] = new QwtPlotCurve();
    this->vLine[BLACK_TO_WHITE] = new QwtPlotCurve();
    this->vLine[WHITE_TO_BLACK]->setPen( p );
    this->vLine[BLACK_TO_WHITE]->setPen( p );
    this->vLine[WHITE_TO_BLACK]->hide();
    this->vLine[BLACK_TO_WHITE]->hide();
    this->vLine[WHITE_TO_BLACK]->attach( cPlots[WHITE_TO_BLACK] );
    this->vLine[BLACK_TO_WHITE]->attach( cPlots[BLACK_TO_WHITE] );

    p.setWidth( 3 );
    p.setStyle(Qt::SolidLine);
    p.setColor( Qt::red );
    this->meanCurves[WHITE_TO_BLACK] = new QwtPlotCurve();
    this->meanCurves[BLACK_TO_WHITE] = new QwtPlotCurve();
    this->meanCurves[WHITE_TO_BLACK]->setPen( p );
    this->meanCurves[BLACK_TO_WHITE]->setPen( p );
    this->meanCurves[WHITE_TO_BLACK]->attach( cPlots[WHITE_TO_BLACK] );
    this->meanCurves[BLACK_TO_WHITE]->attach( cPlots[BLACK_TO_WHITE] );

    p.setStyle(Qt::SolidLine);
    p.setWidth( 1 );

    for(int i=0; i < nCurves; i++)
    {
        p.setColor( (QColor) colorObj[i%nColors] );

        curves[BLACK_TO_WHITE].push_back( new QwtPlotCurve() );
        curves[WHITE_TO_BLACK].push_back( new QwtPlotCurve() );

        curves[BLACK_TO_WHITE][i]->setPen( p );
        curves[WHITE_TO_BLACK][i]->setPen( p );

        curves[BLACK_TO_WHITE][i]->attach( cPlots[BLACK_TO_WHITE] );
        curves[WHITE_TO_BLACK][i]->attach( cPlots[WHITE_TO_BLACK] );
    }

    plotLayout->addWidget( cPlots[BLACK_TO_WHITE] );
    plotLayout->addWidget( cPlots[WHITE_TO_BLACK] );

    plot->setLayout( plotLayout );

    if( this->showPlot ) {
        this->plot->show();
    } else {
        this->plot->hide();
    }
    showPlot = !showPlot;

    plot->resize( 700 , 500);
    qDebug("Preparing plots done ...");
}

void Window::keyPressEvent(QKeyEvent *event)
{   
    //qDebug("Pressed key %c", event->key() );

    switch (event->key()) {
        case Qt::Key_D:{
                this->rcvTogglePlot();
                break;
            }
        case Qt::Key_Q:{
                this->quit();
                break;
            }
        case Qt::Key_Space:
        {
            if( event->isAutoRepeat() )
                break;


            if( this->isRunning ){
                this->msg->setText( "" );
                emit stopMeasurement();
            }
            else{
                this->msg->setText( "Synchronizing clocks ..." );
                emit startMeasurement();
            }
            break;
        }
        case Qt::Key_L:{
			emit showLogWindow();
			break;
		}
        case Qt::Key_C:{
            emit doReset();
            break;
        }
    }
}

void Window::recvStartMeasurement()
{
    this->isRunning = true;
}

void Window::recvStopMeasurement()
{
    this->isRunning = false;
    emit stop();
}

void Window::rcvShowAbout()
{
    QString text;
    text += tr("Lagtest is designed by Strawlab <br>at the Research Institute of Molecular Pathology in Vienna<br><br>");
    text += tr("For more information go to <a href='http://lagtest.org'>lagtest.org</a><br><br>");
    text += tr("You are running lagtest version %1<br>").arg(QCoreApplication::applicationVersion());

    //QMessageBox::about(this, tr("Lagtest - How slow is your display?"), text );
    QMessageBox b(QMessageBox::NoIcon, tr("Lagtest - How slow is your display?"), text, QMessageBox::NoButton, this, Qt::Dialog );
    b.setTextFormat(Qt::RichText);
    b.exec();
}

void Window::rcvTogglePlot()
{
    if( this->showPlot ) {
        this->plot->show();
    } else {
        this->plot->hide();
    }
    this->raise();
    this->setFocus();
    showPlot = !showPlot;
}

void Window::receiveInvalidLatency()
{
    this->msg->setText( "Invalid Latency. Adjust the position of the light sensor to be ontop of the blinking area." );
    this->latency->clear();

    this->vLine[WHITE_TO_BLACK]->hide();
    this->vLine[BLACK_TO_WHITE]->hide();
}

void Window::receiveUnstableLatency()
{
    this->msg->setText( "Unstable Latency. Remain still. Calculating Latency ..." );
    this->latency->clear();

    this->vLine[WHITE_TO_BLACK]->hide();
    this->vLine[BLACK_TO_WHITE]->hide();
}

void Window::receiveStableLatency()
{
    this->msg->setText( "Found a Latency of" );
}

void Window::receiveNewMeasurementWindow(uint8_t* window, double *avgWindow, double *time, flip_type type)
{
    //qDebug("Updateing curve %d",updateCurveIdx[type]);
    for(int j=0; j < LatencyModel::measurementWindowSize; j++)
    {
        this->yData[j] = window[j];
        this->xData[j] = time[j] / 1000000.0;
    }
    updateCurveIdx[type] = (updateCurveIdx[type]+1)%nCurves;
    this->curves[type][this->updateCurveIdx[type]]->setSamples( xData, yData, LatencyModel::measurementWindowSize );

    //Update Mean curve
    this->meanCurves[type]->setSamples( xData, avgWindow, LatencyModel::measurementWindowSize );

    this->cPlots[type]->replot();
}

void Window::receiveLatencyUpdate(LatencyModel* lm)
{
    QString str;
    double ll,al;
    ll = lm->getLastLatency();
    al = lm->getAvgLatency();

    this->msg->setText( "Found a Latency of" );
    this->latency->setText( str.sprintf("Last Latency %.2f ms , Avg. Latency %.2f|%.2f ms", ll/1000000.0, al/1000000.0, lm->getAvgLatencySD()/1000000.0) );

    double x[2], y[2];
    x[0] = al / 1000000.0; y[0] = -5.0;
    x[1] = al / 1000000.0; y[1] = this->meanCurves[WHITE_TO_BLACK]->maxYValue() + 10;
    this->vLine[WHITE_TO_BLACK]->setSamples( x, y, 2 );
    this->vLine[BLACK_TO_WHITE]->setSamples( x, y, 2 );

    this->cPlots[WHITE_TO_BLACK]->setAxisScale(QwtPlot::xBottom, 0, (al / 1000000.0) * 2); this->cPlots[WHITE_TO_BLACK]->replot();
    this->cPlots[BLACK_TO_WHITE]->setAxisScale(QwtPlot::xBottom, 0, (al / 1000000.0) * 2); this->cPlots[BLACK_TO_WHITE]->replot();

    this->vLine[WHITE_TO_BLACK]->show();
    this->vLine[BLACK_TO_WHITE]->show();

}


void Window::recvOpenHelpPage(){
    QDesktopServices::openUrl(QUrl("http://lagtest.org/app-help", QUrl::TolerantMode));
}

void Window::quit(){
    emit QCoreApplication::quit();
}

SubWindow::SubWindow(QWidget* parent) : QWidget( parent , Qt::Window ) {}

// ########################################################################################################
// ####             Flashing Background Using QPaint                                                    ###
// ########################################################################################################


flashingBGQPaint::flashingBGQPaint(int flipRate, TimeModel* clock, RingBuffer<screenFlip> *store) :
    QWidget(0),
    r(this->rect()),
    drawWhiteBG(false),
    clock(clock),
    store(store)
{
    this->timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(flipColor()));
    timer->setInterval(flipRate);
}


void flashingBGQPaint::flipColor()
{
    emit update(); //Will produce a paint event, and make the screen update
    this->drawWhiteBG = !this->drawWhiteBG;
}

void flashingBGQPaint::paintEvent(QPaintEvent *event)
{
    QPainter painter(this);
    painter.fillRect(r, (this->drawWhiteBG) ? Qt::white : Qt::black );

    screenFlip sf;
    sf.type = this->drawWhiteBG ? BLACK_TO_WHITE : WHITE_TO_BLACK;
    sf.local = this->clock->getCurrentTime();

    if( this->timer->isActive() )
    {
        painter.setFont( QFont("Times", 10, QFont::Bold) );
        painter.setPen( Qt::red );
        painter.drawText(r, Qt::AlignTop | Qt::AlignHCenter, tr("Stop by pressing Space Bar"));
    } else {
        painter.setFont( QFont("Times", 20, QFont::Bold) );
        painter.setPen( Qt::red );
        painter.drawText(r, Qt::AlignCenter, tr("Start by pressing Space Bar"));
    }

    this->store->put( &sf );
}

void flashingBGQPaint::receiveStart(){
    qDebug("Start screen blinking ...");
    this->timer->start();    
}

void flashingBGQPaint::receiveStop(){
    qDebug("Stop screen blinking ...");
    this->timer->stop();
    this->drawWhiteBG = false;
    emit update();
}
