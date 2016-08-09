#include "inputwidget.h"
#include "ui_inputwidget.h"
#include <iostream>
#include <clocale>
#include "unistd.h"

#define EOUT(__x__) std::cerr << "#ERROR[ " << __PRETTY_FUNCTION__ << "]: " << __x__ << "\n";
#define IOUT(__x__) std::cerr << "#INFO [ " << __PRETTY_FUNCTION__ << "]: " << __x__ << "\n";

InputWidget::InputWidget(QTcpSocket* _socket, QWidget *parent) :
    QFrame(parent),
    socket(_socket),
    m_ui(new Ui::InputWidget)
{
    m_ui->setupUi(this);

    connect ( this->socket , SIGNAL(disconnected()) , this , SLOT(deinit()) );

    if (!init()) {
        EOUT("failed to init.");
        this->deinit();
    }
}

InputWidget::~InputWidget()
{
    delete m_ui;
    // if (socket!=0) delete socket;
}

bool InputWidget::init()
{
    setlocale (LC_ALL,"C");

    char buf[1024];
    usleep(100000);

    sprintf( buf , "ReferenceGUI Server 1.00\n");
    socket->write( buf , strlen(buf)  );

    if (!socket->waitForReadyRead (1000)) {
        EOUT("Timeout on waiting for handshake ... ");
        return false;
    }

    socket->readLine( buf , sizeof(buf) );
    // IOUT("Read: [" << buf << "]");

    QString cmd(buf);

    QStringList params = cmd.split(" ", QString::SkipEmptyParts);

    //for (int i=0; i<params.size(); i++) {
    //    IOUT("Received param " << i << ": " << params[i].toAscii().constData());
    //}

    if ( params[0] != "SLIDER" ) {
        EOUT("Unknown type: " << params[0].toAscii().constData());
        return false;
    }

    if (params.size() != 6) {
        sprintf( buf , "ERROR only %d params received, expected SLIDER label min max def steps, received: %s\n", (int) params.size() , cmd.toAscii().constData());
        socket->write( buf , strlen(buf)  );
        return false;
    }

    this->m_ui->description->setText(params[1]);

    this->min = params[2].toDouble();
    this->max = params[3].toDouble();
    this->defaultvalue = params[4].toDouble();
    steps = params[5].toInt();

    scale = (max - min) / steps;

    m_ui->slider->setMinimum( 0 );
    m_ui->SpinBox->setMinimum( min );

    m_ui->slider->setMaximum( steps );
    m_ui->SpinBox->setMaximum( max );

    m_ui->slider->setValue( (defaultvalue-min) / scale );
    m_ui->slider->setSingleStep(1);

    m_ui->SpinBox->setValue( defaultvalue );
    m_ui->SpinBox->setSingleStep( scale );
    value = defaultvalue;

    connect ( m_ui->SpinBox , SIGNAL(valueChanged(double)) , this , SLOT(set_double_value(double)) );
    connect ( m_ui->slider , SIGNAL(valueChanged(int)) , this , SLOT(set_int_value(int)) );

    connect ( this->socket , SIGNAL(readyRead()) , this , SLOT(com_handler()) );

    sprintf( buf , "OK\n");
    socket->write( buf , strlen(buf)  );

    return true;
}

void InputWidget::com_handler()
{
    char buf[1024];

    socket->readLine( buf , sizeof(buf) );
    //IOUT("Read: [" << buf << "]");

    sprintf( buf , "%lf\n" , value);

    socket->write( buf , strlen(buf)  );
}

void InputWidget::deinit()
{
    socket->close();
    //delete socket;
    //socket = 0;
    IOUT("Disconnect received ... ");
    emit connection_closed( this );
}

 void InputWidget::set_double_value( double val )
 {
      m_ui->slider->setValue( (int)((val-min) / scale));
   // m_ui->SpinBox->setValue( value );
 }

 void InputWidget::set_int_value( int val )
 {
    value = val * scale + min;
    m_ui->slider->setValue( (value-min) / scale );
    m_ui->SpinBox->setValue( value );
 }

void InputWidget::changeEvent(QEvent *e)
{
    QFrame::changeEvent(e);
    switch (e->type()) {
    case QEvent::LanguageChange:
        m_ui->retranslateUi(this);
        break;
    default:
        break;
    }
}
