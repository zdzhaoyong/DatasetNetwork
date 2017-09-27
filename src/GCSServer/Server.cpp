#include "Server.h"
#include <QThread>

Server::Server(QObject *parent) : QObject(parent)
{
    m_tcpServer = new QTcpServer();
    //设置最大允许连接数，不设置的话默认为30
    m_tcpServer->setMaxPendingConnections(2000);
//    QDBG << m_tcpServer->maxPendingConnections();
    connect(m_tcpServer,SIGNAL(newConnection()),this,SLOT(newConnectSlot()));

}

void Server::init( int port)
{
    if(m_tcpServer->listen(QHostAddress::Any, port)){
        qDebug() << "listen OK!";
    }else{
        qDebug() << "listen error!";
    }
    while(m_mapClient.empty()) {QApplication::processEvents();}

}

void Server::sendData(QString ip, QString data)
{
    m_mapClient.value(ip)->write(data.toLatin1());
}

void Server::sendData(QString ip, uchar *rawData)
{
    m_mapClient.value(ip)->write(QByteArray((char*)rawData));
}

void Server::sendData(QByteArray rawData)
{
    for(auto it=m_mapClient.begin();it!=m_mapClient.end();it++)
    {
        it.value()->write(rawData);
        it.value()->waitForBytesWritten();
    }
}


void Server::newConnectSlot()
{
    QTcpSocket *tcp = m_tcpServer->nextPendingConnection();
    connect(tcp,SIGNAL(readyRead()),this,SLOT(readMessage()));
    m_mapClient.insert(tcp->peerAddress().toString(), tcp);

    connect(tcp,SIGNAL(disconnected()),this,SLOT(removeUserFormList()));

}

void Server::readMessage()
{
    QTcpSocket *socket = static_cast<QTcpSocket*>(sender());
    qDebug()<<socket->peerAddress().toString()
           <<QString(socket->readAll());
}

void Server::removeUserFormList()
{
    QTcpSocket* socket = static_cast<QTcpSocket*>(sender());

    QMap<QString, QTcpSocket *>::iterator it;

    for(it=m_mapClient.begin();it!=m_mapClient.end();it++)
    {
        if(socket->peerAddress().toString() == it.key())
        {
            qDebug()<<"Removed "<<it.key();
            m_mapClient.erase(it);
            break;
        }
    }
}
