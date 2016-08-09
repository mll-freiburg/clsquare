#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <iostream>
#include "inputwidget.h"

#define EOUT(__x__) std::cerr << "#ERROR[ " << __PRETTY_FUNCTION__ << "]: " << __x__ << "\n";
#define IOUT(__x__) std::cerr << "#INFO [ " << __PRETTY_FUNCTION__ << "]: " << __x__ << "\n";

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    server = new QTcpServer();

    bool res = server->listen(QHostAddress::Any , 7555);

    if (!res) {
        EOUT("Can not install server ... ");
        qApp->exit(0);
    }

    IOUT("Listening ... ");

    layout = new QVBoxLayout;
    ui->scrollAreaWidgetContents->setLayout( layout );

    connect( server , SIGNAL(newConnection()) , this, SLOT(handle_new_connection()) );
}

MainWindow::~MainWindow()
{
    delete ui;
    delete server;
}

void MainWindow::handle_new_connection()
{
    IOUT("Received new connection ... ");

    while( server->hasPendingConnections() ) {
        QTcpSocket* s = server->nextPendingConnection();
        InputWidget* w = new InputWidget( s );
        connect ( w , SIGNAL(connection_closed(InputWidget*)) , this , SLOT(handle_closed_connection(InputWidget*)) );
        layout->addWidget( w );
        w->show();
    }
}

void MainWindow::handle_closed_connection( InputWidget* sender )
{
    sender->hide();
    sender->disconnect();
    layout->removeWidget( sender );
    delete sender;
}
