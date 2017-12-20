
#define QT_NO_DEBUG_STREAM
#include "Client.h"

#include <QGridLayout>
#include <QCoreApplication>
#include <QHeaderView>
#include <QNoDebug>
#include <fstream>
#include <GSLAM/core/Timer.h>

extern "C"{
#include "libavcodec/avcodec.h"
#include "libavfilter/avfilter.h"
#include "libavformat/avformat.h"
#include "libswscale/swscale.h"
}

class Decoder{
public:
    Decoder()
        :isFFmpegInitialized(0)
    {
        initialize();
    }

    ~Decoder(){
        if (m_pCodecCtx)
        {
            avcodec_close(m_pCodecCtx);
            m_pCodecCtx = NULL;
        }

        av_free(m_pYUVFrame);
        av_free(m_pCodecCtx);
        av_parser_close(m_pCodecPaser);
        av_free(avPicture);
        if(m_pImgCtx) sws_freeContext(m_pImgCtx);
        if(m_PicBuf) delete[] m_PicBuf;
    }

    int initialize()
    {
        if (isFFmpegInitialized == 0)
        {
            avcodec_register_all();
            av_register_all();
            isFFmpegInitialized = 1;
        }
        m_pAVCodec = avcodec_find_decoder(AV_CODEC_ID_H264);
        m_pCodecCtx = avcodec_alloc_context3(m_pAVCodec);
        m_pCodecPaser = av_parser_init(AV_CODEC_ID_H264);
        if (m_pAVCodec == NULL || m_pCodecCtx == NULL)
        {
            qDebug()<<("m_pAVCodec == NULL||m_pCodecCtx == NULL");
            return 0;
        }

        if (m_pAVCodec->capabilities & CODEC_CAP_TRUNCATED)
            m_pCodecCtx->flags |= CODEC_FLAG_TRUNCATED;

        m_pCodecCtx->thread_count = 4;
        m_pCodecCtx->thread_type = FF_THREAD_FRAME;

        if (avcodec_open2(m_pCodecCtx, m_pAVCodec,NULL) < 0)
        {
            m_pAVCodec = NULL;
            return 0;
        }

        m_pYUVFrame = av_frame_alloc();
        avPicture=av_frame_alloc();

        if (m_pYUVFrame == NULL)
        {
            qDebug()<<(" CDecoder avcodec_alloc_frame() == NULL ");
            return 0;
        }
        qDebug()<<("CDecoder::prepare()2");
        return 1;
    }

    bool decode(RTMapperNetHeader& header,QByteArray& imageBuffer,QImage& image)
    {
//        if(!ofs.is_open()) ofs.open("dji.h264");

//        if(ofs.is_open()) ofs.write(imageBuffer.data(),imageBuffer.size());
        localBuffer.append(imageBuffer);
        int size=localBuffer.size();
        uint8_t *buff = (uint8_t*) localBuffer.data();
        int parseSize=0;
        if(size >= fillersize2 && memcmp(fillerbuffer2, buff+size-fillersize2, fillersize2) == 0)
        {
            parseSize=size-fillersize2;
        }
        else if (size >= audaudsize2 && memcmp(audaudbuffer2, buff+size-audaudsize2, audaudsize2) == 0)
        {
            parseSize=size-audaudsize2;
        }
        else if (size >= audsize2 && memcmp(audbuffer2, buff+size-audsize2, audsize2) == 0)
        {
            parseSize=size-audsize2;
        }
        else
        {
            parseSize=size;
        }

        int paserLength_In = parseSize;
            int paserLen;
            int decode_data_length;
            int got_picture = 0;
            while (paserLength_In > 0)
            {
                timer.enter("av_parser_parse2");
                AVPacket packet;
                av_init_packet(&packet);
                if (m_pCodecPaser == NULL) {
                    qDebug()<<("m_pCodecPaser == NULL");
                    initialize();
                }
                if (m_pCodecCtx == NULL) {
                    qDebug()<<("m_pCodecCtx == NULL");
                    initialize();
                }
                paserLen = av_parser_parse2(m_pCodecPaser, m_pCodecCtx, &packet.data, &packet.size, buff,
                        parseSize, AV_NOPTS_VALUE, AV_NOPTS_VALUE, AV_NOPTS_VALUE);

                localBuffer.remove(0,paserLen);
                timer.leave("av_parser_parse2");

                if (packet.size > 0)
                {
                    timer.enter("avcodec_decode_video2");
                    int got;
                    paserLen= avcodec_decode_video2(m_pCodecCtx,m_pYUVFrame,&got,&packet);
                    timer.leave("avcodec_decode_video2");
                    if(paserLen<0)
                    {
                        qDebug()<<"Error while decoding frame.";
                        av_free_packet(&packet);
                        return false;
                    }
                    if(got)
                    {
                        timer.enter("sws_scale");
                        int bytes=avpicture_get_size(PIX_FMT_RGB32,m_pCodecCtx->width,m_pCodecCtx->height);
                        if(m_PicBytes!=bytes)
                        {
                            m_PicBytes=bytes;
                            if(m_PicBuf) delete[]m_PicBuf;
                            m_PicBuf =new uint8_t[m_PicBytes];
                            avpicture_fill((AVPicture*)avPicture,m_PicBuf,PIX_FMT_RGB32,
                                           m_pCodecCtx->width,m_pCodecCtx->height);
                            if(m_pImgCtx) sws_freeContext(m_pImgCtx);
                            m_pImgCtx=NULL;
                        }

                        if(!m_pImgCtx)
                            m_pImgCtx=sws_getContext(m_pCodecCtx->width,m_pCodecCtx->height,m_pCodecCtx->pix_fmt,
                                           m_pCodecCtx->width,m_pCodecCtx->height,PIX_FMT_RGB32,SWS_BICUBIC,NULL,NULL,NULL);

//                        m_pYUVFrame->data[0]+= m_pYUVFrame->linesize[0]*(m_pCodecCtx->height-1);
//                        m_pYUVFrame->linesize[0]*=-1;
//                        m_pYUVFrame->data[1]+= m_pYUVFrame->linesize[1]*(m_pCodecCtx->height/2-1);
//                        m_pYUVFrame->linesize[1]*=-1;
//                        m_pYUVFrame->data[2]+= m_pYUVFrame->linesize[2]*(m_pCodecCtx->height/2-1);
//                        m_pYUVFrame->linesize[2]*=-1;

                        sws_scale(m_pImgCtx,(const uint8_t* const*)m_pYUVFrame->data,
                                  m_pYUVFrame->linesize,0,m_pCodecCtx->height,avPicture->data,avPicture->linesize);
                        QImage img=QImage(avPicture->data[0],m_pCodecCtx->width,m_pCodecCtx->height,QImage::Format_RGB32).copy();
                        image=img;
                        qDebug()<<"packageSize:"<<packet.size
                               <<", keyframe:"<< m_pCodecPaser->key_frame
                              <<", width:"<<m_pCodecCtx->width
                             <<", height:"<<m_pCodecCtx->height;
                        av_free_packet(&packet);
                        timer.leave("sws_scale");
                        return true;
                    }
                    av_free_packet(&packet);
                    return false;

                }
                av_free_packet(&packet);
                return false;
            }

            return false;
    }

    uint8_t audbuffer2[6] = {0x00,0x00,0x00,0x01,0x09,0x10};
    uint8_t audsize2 = 6;
    uint8_t fillerbuffer2[11] = {0x00,0x00,0x00,0x01,0x0C,0x00,0x00,0x00,0x01,0x09,0x10};
    uint8_t fillersize2 = 11;
    uint8_t audaudbuffer2[12] = {0x00,0x00,0x00,0x01,0x09,0x10, 0x00,0x00,0x00,0x01,0x09,0x10};
    uint8_t audaudsize2 = 12;
    int m_PicBytes=0;
    uint8_t *m_PicBuf=NULL;
    int isFFmpegInitialized;

    QByteArray localBuffer;

    AVFrame* m_pYUVFrame,*avPicture;
    AVCodecContext* m_pCodecCtx;
    AVCodec* m_pAVCodec;
    AVCodecParserContext* m_pCodecPaser;
    SwsContext* m_pImgCtx=NULL;

    std::ofstream ofs;
};

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
    QTableWidgetItem* it=item(row,col);
    if(it!=NULL)
    {
        it->setText(val);
        return it;
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
    QTableWidgetItem* it=item(row,col);
    if(it!=NULL)
    {
        it->setText(QString("%1").arg(val,0,'g',13));
        return it;
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
    static double lastUpdateTime=0;
    double curTime=GSLAM::TicToc::timestamp();
    if(curTime-lastUpdateTime<0.1) return;
    lastUpdateTime=curTime;
    vars["timestamp"]=QString("%1").arg(frame.timestamp,0,'g',15);
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
    vars["model"]=frame.uavname;

    if(rowCount()!=vars.size())
        setRowCount(vars.size());
    int i=0;
    for(auto it:vars)
    {
        setValue(i,0,it.first);
        setValue(i,1,it.second);
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
    timer.enter("slotReadyRead");
    while (_socket->bytesAvailable() < sizeof(header)){
        QCoreApplication::processEvents( QEventLoop::AllEvents, 1 );
        return;
    }

    while (_socket->bytesAvailable()>sizeof(header))
    {
        timer.enter("slotReadyReadCircle");
        int losted=0;
        while(imageBuffer.isEmpty())
        {
            timer.enter("slotReadyReadCircle1");
            int lengthReceived=_socket->read((char*)&header,sizeof(header));
            if(lengthReceived<sizeof(header))
            {
                timer.leave("slotReadyReadCircle1");
                return;
            }
            header.convert();
            if (!header.isValid())
            {
                losted+=sizeof(header);
                static_assert(sizeof(RTMapperNetHeader)==128,"");
                timer.leave("slotReadyReadCircle1");
                continue;
            }
            else
            {
                if(losted)
                {
                    std::cerr<<"Losted "<<losted<<" bytes."<<std::endl;
                    losted=0;
                }
                header.convert();
                if(header.payload_size<=0) return;
                received=0;
                imageBuffer.resize(header.payload_size);

                qDebug()<<"New header "<<(int)header.header_size<<","<<header.timestamp;
                qDebug()<<"lla:"<<header.lng<<","<<header.lat<<","<<header.alt;

                timer.leave("slotReadyReadCircle1");
                break;
            }
        }

        timer.enter("readImageBuffer");
        if(received<imageBuffer.size())
        {
            uint remaining = imageBuffer.size() - received;
            uint readed = _socket->read(imageBuffer.data() + received,
                                        std::min(remaining,(uint)_socket->bytesAvailable()));
            received += readed;
        }
        timer.leave("readImageBuffer");

        if(received>=imageBuffer.size())
        {
            timer.enter("decode");
            decode(header,imageBuffer);
            timer.leave("decode");

            imageBuffer.clear();
            received=0;
        }
        timer.leave("slotReadyReadCircle");
    }
    QCoreApplication::processEvents();
    timer.leave("slotReadyRead");

}

bool Client::decode(RTMapperNetHeader& header,QByteArray& imageBuffer)
{
    timer.enter("updateHeader");
    headerVis->update(header);
    timer.leave("updateHeader");

    QImage image;
    if(header.video_codec==VIDEOCODEC_JPEG)
    {
        image.loadFromData(imageBuffer,QString("JPG").toLocal8Bit().data());
    }
    else if(header.video_codec==VIDEOCODEC_H264)
    {
        static Decoder decoder;
        if(!decoder.decode(header,imageBuffer,image)) return false;
    }
    else if(header.video_codec==VIDEOCODEC_NONE)
    {
        image=QImage((uchar*)imageBuffer.data(),header.display_width,header.display_height,
                     QImage::Format_RGB888).copy();
    }

    if(image.isNull()) return false;
    timer.enter("setPixmap");
    _image->setPixmap(QPixmap::fromImage(image));
    timer.leave("setPixmap");
    qDebug()<<"Decoded new frame.\n";
    return true;
}
