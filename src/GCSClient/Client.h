#include <QTcpSocket>
#include <QWidget>
#include <QMainWindow>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QTableWidget>

#include "../dbnet/RTMapperNetInterface.h"

class InfomationViewer : public QTableWidget
{
    Q_OBJECT
public:
    InfomationViewer(QWidget* parent);
    QTableWidgetItem* setValue(int row,int col,QString val);
    QTableWidgetItem* setValue(int row,int col,double  val);
    void              update(const RTMapperNetHeader& frame);
    std::map<QString,QString>   vars;
};

class Client : public QWidget
{
    Q_OBJECT
public:
    Client(QWidget* parent=NULL);

public slots:
    void slotConnect(QString ip,qint16 port);
    void slotConnect();
    void slotConnected();
    void slotDisconnected();

    void slotReadyRead();

    bool isSocketGood(){
        return _socket != nullptr
               && _socket->state() == QTcpSocket::SocketState::ConnectedState;
    }
private:
    bool decode(RTMapperNetHeader& header,QByteArray& imageBuffer);

    RTMapperNetHeader header;
    QByteArray        imageBuffer;
    uint              received;

    QTcpSocket*     _socket;

    QLineEdit       *ip,*port;
    QPushButton     *buttonConnect,*buttonDisConnect;
    QLabel          *_image;
    InfomationViewer *headerVis;
};
