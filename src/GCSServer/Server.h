#include <QTcpServer>
#include <QTcpSocket>
#include <QNetworkInterface>
#include <QApplication>

class Server : public QObject
{
    Q_OBJECT
public:
    explicit Server(QObject *parent = 0);

    void init( int port=3000);
    void sendData(QString ip, QString data);
    void sendData(QString ip, uchar *rawData);
    void sendData(QByteArray rawData);
signals:
    void signalTest();
private slots:

    void newConnectSlot();
    void readMessage();
    void removeUserFormList();
public:
    QTcpServer *m_tcpServer;
    QMap<QString, QTcpSocket *> m_mapClient;
};
