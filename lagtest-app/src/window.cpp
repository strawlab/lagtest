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
#include <QPolygon>
#include <QPoint>
#include <QApplication>
#include <QDesktopWidget>

Window::Window(TimeModel *tm, RingBuffer<screenFlip> *screenFlips) :
    showPlot(false) , isRunning(false)
{
    qDebug("Creating main window ...");
    QWidget* flipWindow;	
    setWindowTitle("Lagtest - How slow is your display?");
    this->resize(640, 480);

    QVBoxLayout *layout = new QVBoxLayout;
    this->msg = new QLabel("Put the light sensor on the black area, then press the space bar.");
    this->latency = new QLabel("");

    msg->setAlignment(Qt::AlignHCenter);
    latency->setAlignment(Qt::AlignHCenter);

    QHBoxLayout* buttonLayout = new QHBoxLayout;

    layout->addWidget(msg, 0);
    layout->addWidget(latency, 0);

    QGLFormat fmt = QGLFormat::defaultFormat();
    fmt.setSwapInterval(0); //Dissable Vsync
    QGLFormat::setDefaultFormat(fmt);

    //flipWindow = new flashingBGQPaint(500, tm, screenFlips);
    flipWindow = new flashingBGQPaint(500, tm, screenFlips, fmt, this);

    connect( this, SIGNAL(startMeasurement()), flipWindow, SLOT(receiveStart()) );
    connect( this, SIGNAL(startMeasurement()), this, SLOT(recvStartMeasurement()) );
    connect( this, SIGNAL(stopMeasurement()), this, SLOT(recvStopMeasurement()) );
    connect( this, SIGNAL(stop()), flipWindow, SLOT(receiveStop()) );

    connect( flipWindow, SIGNAL(setLed(bool)), this, SIGNAL(setLed(bool)) );

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
    QAction* writeReportAction = fileMenu->addAction(tr("create &Report"));
    QAction* exitAction = fileMenu->addAction(tr("E&xit"));
    menuBar->addMenu(fileMenu);

    QMenu* optionsMenu = new QMenu(tr("&Options"), this);
    QAction* selectPortAction = optionsMenu->addAction(tr("select &Port"));
    QAction* flashAction = optionsMenu->addAction(tr("upload new &Firmware to Arduino"));
    QAction* plotAction = optionsMenu->addAction(tr("show &Graph"));
    QAction* showLogAction = optionsMenu->addAction(tr("show debug &Log"));
    QAction* setSystemLatencyAction = optionsMenu->addAction(tr("Set &system latency"));
    menuBar->addMenu(optionsMenu);

    QMenu* helpMenu = new QMenu(tr("&Help"), this);
    QAction* helpPageAction = helpMenu->addAction(tr("open &Help webpage"));
    QAction* aboutAction = helpMenu->addAction(tr("&About"));
    menuBar->addMenu(helpMenu);

    connect(exitAction, SIGNAL(triggered()), this, SLOT(quit()));
    connect(flashAction, SIGNAL(triggered()), this, SIGNAL(flashArduino()));
    connect(writeReportAction, SIGNAL(triggered()), this, SIGNAL(generateReport()));
    connect(helpPageAction, SIGNAL(triggered()), this, SLOT(recvOpenHelpPage()));
    connect(plotAction, SIGNAL(triggered()), this, SLOT(rcvTogglePlot()));
    connect(aboutAction, SIGNAL(triggered()), this, SLOT( rcvShowAbout()) );
    connect(showLogAction, SIGNAL(triggered()), this, SIGNAL(showLogWindow()));
    connect(selectPortAction, SIGNAL(triggered()), this, SIGNAL(selectPort()));
    connect(setSystemLatencyAction, SIGNAL(triggered()), this, SIGNAL(setSystemLatency()));


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
    text += tr("<h2>Lagtest version %1</h2>").arg(QCoreApplication::applicationVersion());
    text += tr("Written by:");
    text += tr("<ul><li>Manuel Pasieka, Campus Support Facilities, Vienna</li>");
    text += tr(" <li>Andrew D. Straw, Research Institute of Molecular Pathology (IMP), Vienna<br></li></ul>");
    text += tr("For more information, see <a href='http://lagtest.org'>lagtest.org</a><br><br>");

    text += tr("The source code written by the lagtest authors is licensed under the ");
    text += tr("MIT license. The application also includes ");
    text += tr("<a href='http://www.teuniz.net/RS-232/'>RS-232</a>, ");
    text += tr("which is GPL-2 licensed. Therefore, the compiled binary is ");
    text += tr("available under the GPL-2 license. ");

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
    this->msg->setText( "Invalid Latency. Adjust the position of the light sensor to be on the blinking area." );
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
    QString url = tr("http://lagtest.org/app-help/%1").arg(QCoreApplication::applicationVersion());
    QDesktopServices::openUrl(QUrl(url, QUrl::TolerantMode));
}

void Window::quit(){
    emit QCoreApplication::quit();
}

SubWindow::SubWindow(QWidget* parent) : QWidget( parent , Qt::Window ) {}

// ########################################################################################################
// ####             Flashing Background Using QPaint                                                    ###
// ########################################################################################################


//flashingBGQPaint::flashingBGQPaint(int flipRate, TimeModel* clock, RingBuffer<screenFlip> *store) :
flashingBGQPaint::flashingBGQPaint(int flipRate, TimeModel *clock, RingBuffer<screenFlip> *store, QGLFormat &fmt, QWidget* parent) :
    //QWidget(0),
    QGLWidget(fmt, parent),
    r(parent->rect()),
    drawWhiteBG(false),
    clock(clock),
    store(store)
{
    this->timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(flipColor()));

    //setFixedSize( parent->size() );

    timer->setInterval(flipRate);
    //timer->setInterval(50);
}


void flashingBGQPaint::flipColor()
{
    emit update(); //Will produce a paint event, and make the screen update
    this->drawWhiteBG = !this->drawWhiteBG;
}

void flashingBGQPaint::paintEvent(QPaintEvent *event)
{
	static int cnt = 0;
    QPainter painter(this);

    /*
    //TIP: use this to test for vsync of or own. Simply trigger fast update calls. If vsync is off the vertical bar will be
    //split into displayced vertical bars.
    qDebug( "Paint Device %d , SwapInterval %d", (painter.paintEngine())->type() , this->format().swapInterval() );

    int w = (QApplication::desktop())->width();
    static QPolygon p;

    if( cnt == 0 ){
        p.clear();
        p << QPoint(0,0) << QPoint(20,0) << QPoint(20, 800) << QPoint(0,800);
    }

    painter.setBrush(QBrush("#c56c00"));
    cnt = ( (cnt+1) % 100 );
    painter.drawPolygon( p );
    p.translate( w / 100, 0);
    */

    //qDebug( "Paint Device %d , SwapInterval %d", (painter.paintEngine())->type() , this->format().swapInterval() );
	painter.fillRect(r, (this->drawWhiteBG) ? Qt::white : Qt::black );
    screenFlip sf;
    //sf.type = this->drawWhiteBG ? BLACK_TO_WHITE : WHITE_TO_BLACK;
    if( this->drawWhiteBG)
    {
        sf.type = BLACK_TO_WHITE;
        emit setLed( true );
    } else {
        sf.type = WHITE_TO_BLACK;
        emit setLed( false );
    }
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
