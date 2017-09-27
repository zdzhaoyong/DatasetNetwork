#include "RTMapperDataset.h"
#include "RTMapperNetInterface.h"
#include <QTcpSocket>
#include <QSocketNotifier>
#include <GSLAM/core/VecParament.h>
#include <QCoreApplication>
#include <QImage>

using GSLAM::FramePtr;

class DatasetNetwork: public RTMapperDataset
{
public:
    DatasetNetwork():_frameId(0),_lastGrabId(0){}

    virtual std::string type() const{return "DatasetNetwork";}

    virtual bool        isOpened(){
        return _socket.state() == QTcpSocket::SocketState::ConnectedState&&_camera.isValid();
    }

    bool                open(QString ip,quint16 port,GSLAM::Camera camera=GSLAM::Camera())
    {
        _socket.connectToHost(ip,port,QTcpSocket::ReadWrite);

        if(_socket.waitForConnected(3000))
        {
            _socket.write("VIDEO_JPG_TYPE\n");
            _socket.flush();
            _readthread=std::thread(&DatasetNetwork::run,this);
            return true;
        }
        return false;
    }

    virtual bool        open(const std::string& dataset)
    {
        GSLAM::Svar var;
        if(!var.ParseFile(dataset)) return false;

        if(!var.exist("Camera.Paraments")) return false;

        VecParament<double> camParas;
        camParas=var.get_var("Camera.Paraments",camParas);

        _camera=GSLAM::Camera(camParas.data);
        if(!_camera.isValid()) return false;

        QString ip=var.GetString("Dataset.IP","127.0.0.1").c_str();
        quint16 port=var.GetInt("Dataset.Port",1992);
        return open(ip,port,_camera);
    }

    void run()
    {
        int               lengthReceived=0;
        _socket.setReadBufferSize(1e7);
        while(_socket.state() == QTcpSocket::SocketState::ConnectedState)
        {
            if (_socket.waitForReadyRead(3000)){
                while (_socket.bytesAvailable() < sizeof(header)){
                    QCoreApplication::processEvents( QEventLoop::AllEvents, 1 );
                }
                lengthReceived=_socket.read((char*)&header,sizeof(header));
                if (memcmp(&header,"PaVE",4)!=0) continue;

                qDebug()<<"New header "<<(int)header.header_size<<","<<header.timestamp;
                qDebug()<<"lla:"<<header.lng<<","<<header.lat<<","<<header.alt;

                if(header.payload_size<=0) continue;

                // decode image
                {
                    QByteArray imageBuffer;
                    imageBuffer.resize(header.payload_size);
                    uint received = 0;
                    while (_socket.state() == QTcpSocket::SocketState::ConnectedState
                           && received < imageBuffer.size()){
                        uint remaining = imageBuffer.size() - received;
                        if (_socket.waitForReadyRead())
                        {
                            while (_socket.bytesAvailable()==0){
                                QCoreApplication::processEvents( QEventLoop::AllEvents, 1 );
                            }
                            uint readed = _socket.read(imageBuffer.data() + received,
                                                       std::min(remaining,(uint)_socket.bytesAvailable()));
                            received += readed;
                        }
                        else{
                            qDebug() << "Not ready socket for read, timed out.";
                        }
                        QCoreApplication::processEvents( QEventLoop::AllEvents, 1 );
                    }

                    decode(header,imageBuffer);
                }

            }
            else{
                qDebug() << "Not ready socket for read, timed out.";
            }

            QCoreApplication::processEvents( QEventLoop::AllEvents, 1 );
        }
    }

    bool decode(RTMapperNetHeader& header,QByteArray& imageBuffer)
    {
        GSLAM::GImage gimage;
        if(header.video_codec==VIDEOCODEC_JPEG)
        {
            QImage image;
            image.loadFromData(imageBuffer,QString("JPG").toLocal8Bit().data());
            if(image.isNull()) return false;
            gimage=GSLAM::GImage(image.width(),image.height(),GSLAM::GImageType<uchar,3>::Type,image.bits()).clone();
        }
        else
        {
            gimage=GSLAM::GImage(header.display_width,header.display_height,CV_8UC3,(uchar*)imageBuffer.data()).clone();
        }

        RTMapperFrame* frame=new RTMapperFrame(_frameId++,header.timestamp);
        _frame=SPtr<RTMapperFrame>(frame);
        frame->_image=gimage;
        frame->_timestamp=header.timestamp;
        frame->_camera=_camera;
        qDebug()<<"Decoded new frame.\n";
        return false;
    }

    virtual FramePtr grabFrame(){
        while(!_frame) usleep(10000);
        if(_frameId>_lastGrabId&&_frame) {
            _lastGrabId=_frameId;
            return _frame;
        }
        else return _frame;
    }

    GSLAM::Camera   _camera;
    QTcpSocket      _socket;
    std::thread     _readthread;

    RTMapperNetHeader header;

    int             _frameId,_lastGrabId;
    GSLAM::MutexRW  _mutexFrame;
    SPtr<RTMapperFrame>   _frame;
};
using GSLAM::DatasetFactory;

REGISTER_DATASET(DatasetNetwork,net);

