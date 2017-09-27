#include "Client.h"

#include <QGridLayout>
#include <QCoreApplication>
#include <QHeaderView>

InfomationViewer::InfomationViewer(QWidget* parent)
    :QTableWidget(parent){
    setColumnCount(2);
    setHorizontalHeaderLabels({"name","value"});

    QHeaderView *HorzHdr = horizontalHeader();
    HorzHdr->setResizeMode(QHeaderView::Stretch);
    verticalHeader()->setResizeMode(QHeaderView::ResizeToContents);
}

QTableWidgetItem* InfomationViewer::setValue(int row,int col,QString val)
{
    if(item(row,col)!=NULL)
    {
        item(row,col)->setText(val);
        return item(row,col);
    }
    else
    {
        QTableWidgetItem* item=new QTableWidgetItem();
        item->setText(val);
        setItem(row,col,item);
        return item;
    }
}

QTableWidgetItem* InfomationViewer::setValue(int row,int col,double  val)
{
    if(item(row,col)!=NULL)
    {
        item(row,col)->setText(QString("%1").arg(val,0,'g',13));
        return item(row,col);
    }
    else
    {
        QTableWidgetItem* item=new QTableWidgetItem();
        item->setText(QString("%1").arg(val,0,'g',13));
        setItem(row,col,item);
        return item;
    }
}

void InfomationViewer::update(const RTMapperNetHeader& frame)
{
    vars["timestamp"]=QString("%1").arg(frame.timestamp);
    vars["alt"]=QString("%1").arg(frame.alt);
    vars["lng"]=QString("%1").arg(frame.lng);
    vars["lat"]=QString("%1").arg(frame.lat);
    vars["battery"]=QString("%1").arg(frame.battery);
    vars["cam_pitch"]=QString("%1").arg(frame.cam_pitch);
    vars["cam_roll"]=QString("%1").arg(frame.cam_roll);
    vars["cam_yaw"]=QString("%1").arg(frame.cam_yaw);
    vars["control_quality"]=QString("%1").arg(frame.control_quality);
    vars["display_height"]=QString("%1").arg(frame.display_height);
    vars["display_width"]=QString("%1").arg(frame.display_width);
    vars["frame_type"]=QString("%1").arg(frame.frame_type);
    vars["H"]=QString("%1").arg(frame.H);
    vars["nSat"]=QString("%1").arg(frame.nSat);

    setRowCount(vars.size());
    int i=0;
    for(auto it:vars)
    {
        setValue(i,0,it.first)->setFlags(Qt::ItemIsEditable);
        setValue(i,1,it.second)->setFlags(Qt::ItemIsEditable);
        i++;
    }
}

Client::Client(QWidget* parent)
    :QWidget(parent)
{
    QGridLayout* layout=new QGridLayout(this);
    ip    = new QLineEdit("127.0.0.1",this);
    port  = new QLineEdit("3000",this);
    buttonConnect=new QPushButton("connect",this);
    buttonDisConnect=new QPushButton("disconnect",this);
    _image= new QLabel("image",this);
    headerVis=new InfomationViewer(this);
    layout->addWidget(ip,0,0);
    layout->addWidget(port,0,1);
    layout->addWidget(buttonConnect,0,2);
    layout->addWidget(buttonDisConnect,0,3);
    layout->addWidget(_image,1,0,1,2);
    layout->addWidget(headerVis,1,2,1,2);

    connect(buttonConnect,SIGNAL(clicked(bool)),this,SLOT(slotConnect()));
    connect(buttonDisConnect,SIGNAL(clicked(bool)),this,SLOT(slotDisconnected()));
}

void Client::slotConnect()
{
    QString ipStr=ip->text();
    bool ok=false;
    ushort  portInt=port->text().toUShort(&ok);
    if(!ok) return;
    slotConnect(ipStr,portInt);
}

void Client::slotConnect(QString ip,qint16 port)
{
    _socket=new QTcpSocket(this);
    connect(_socket,SIGNAL(connected()),this,SLOT(slotConnected()));
    connect(_socket,SIGNAL(disconnected()),this,SLOT(slotDisconnected()));

    _socket->connectToHost(ip,port,QTcpSocket::ReadWrite);

}


void Client::slotConnected()
{
    _socket->write("VIDEO_JPG_TYPE\n");
    _socket->flush();
    connect(_socket,SIGNAL(readyRead()),this,SLOT(slotReadyRead()));
    buttonConnect->setEnabled(false);
}

void Client::slotDisconnected()
{
    if(_socket) _socket->disconnectFromHost();
    qDebug()<<"Disconnected.";
    buttonConnect->setEnabled(true);
}


void Client::slotReadyRead()
{
    while (_socket->bytesAvailable() < sizeof(header)){
        QCoreApplication::processEvents( QEventLoop::AllEvents, 1 );
    }

    while(imageBuffer.isEmpty())
    {
        int lengthReceived=_socket->read((char*)&header,sizeof(header));
        if(lengthReceived<sizeof(header)) return;
        if (memcmp(&header,"PaVE",4)!=0) continue;
        else
        {
            if(header.payload_size<=0) return;
            received=0;
            imageBuffer.resize(header.payload_size);
            headerVis->update(header);
            qDebug()<<"New header "<<(int)header.header_size<<","<<header.timestamp;
            qDebug()<<"lla:"<<header.lng<<","<<header.lat<<","<<header.alt;

            break;
        }
    }

    if(received<imageBuffer.size())
    {
        uint remaining = imageBuffer.size() - received;
        uint readed = _socket->read(imageBuffer.data() + received,
                                    std::min(remaining,(uint)_socket->bytesAvailable()));
        received += readed;
    }

    if(received>=imageBuffer.size())
    {
        decode(header,imageBuffer);

        imageBuffer.clear();
        received=0;
    }
    QCoreApplication::processEvents();

    if(_socket->bytesAvailable()>sizeof(header)) slotReadyRead();
}

bool Client::decode(RTMapperNetHeader& header,QByteArray& imageBuffer)
{
    QImage image;
    if(header.video_codec==VIDEOCODEC_JPEG)
    {
        image.loadFromData(imageBuffer,QString("JPG").toLocal8Bit().data());
    }
    else
    {
        image=QImage((uchar*)imageBuffer.data(),header.display_width,header.display_height,
                     QImage::Format_RGB888).copy();
    }

    if(image.isNull()) return false;
    _image->setPixmap(QPixmap::fromImage(image));
    qDebug()<<"Decoded new frame.\n";
    return true;
}
