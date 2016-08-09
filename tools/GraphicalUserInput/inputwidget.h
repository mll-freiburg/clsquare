#ifndef INPUTWIDGET_H
#define INPUTWIDGET_H

#include <QtGui/QFrame>
#include <QTcpSocket>

namespace Ui {
    class InputWidget;
}

class InputWidget : public QFrame {
    Q_OBJECT
public:
    InputWidget(QTcpSocket* _socket, QWidget *parent = 0);
    ~InputWidget();

protected:
    void changeEvent(QEvent *e);

    bool init();

    QTcpSocket* socket;

    double min;
    double max;
    double defaultvalue;
    int    steps;
    double scale;

    double value;

signals:
    void connection_closed( InputWidget * sender );

protected slots:
    void set_double_value( double val );
    void set_int_value( int val );

    void com_handler();
    void deinit();

private:
    Ui::InputWidget *m_ui;
};

#endif // INPUTWIDGET_H
