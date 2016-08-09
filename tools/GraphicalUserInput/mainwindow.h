#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QtGui/QMainWindow>
#include <QTcpServer>
#include <QVBoxLayout>
#include "inputwidget.h"

namespace Ui
{
    class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = 0);
    ~MainWindow();

protected slots:
    void handle_new_connection();
    void handle_closed_connection( InputWidget* sender );

protected:
    QTcpServer  *server;
    QVBoxLayout *layout;


private:
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
