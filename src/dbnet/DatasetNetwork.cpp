#include "RTMapperDataset.h"
#include "RTMapperNetInterface.h"
#include <QTcpSocket>
#include <QSocketNotifier>
#include <GSLAM/core/VecParament.h>
#include <QCoreApplication>
#include <QImage>
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
//                        qDebug()<<"packageSize:"<<packet.size
//                               <<", keyframe:"<< m_pCodecPaser->key_frame
//                              <<", width:"<<m_pCodecCtx->width
//                             <<", height:"<<m_pCodecCtx->height;
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
#include <GSLAM/core/XML.h>

namespace tinyxml2 {

using std::string;
bool saveSvarXML(GSLAM::Svar& var,const char* filename)
{
    var.update();
    GSLAM::Svar::SvarMap data=var.get_data();

    XMLDocument doc;
    XMLElement* element = doc.NewElement("project");
    doc.InsertFirstChild(element);
    XMLDeclaration* declaration=doc.NewDeclaration();//添加xml文件头申明
    doc.InsertFirstChild(declaration);

    for(std::pair<string,string> it:data)
    {
        string      name=it.first;
        XMLElement* parentEle=element;
        while(true)
        {
            int idx=name.find_first_of('.');
            if(idx==string::npos)
            {
                XMLElement* ele=doc.NewElement(name.c_str());
                ele->SetAttribute("value",it.second.c_str());
                parentEle->InsertEndChild(ele);
                break;
            }
            string parentName=name.substr(0,idx);
            name=name.substr(idx+1);
            XMLElement* ele=parentEle->FirstChildElement(parentName.c_str());
            if(!ele)
            {
                ele=doc.NewElement(parentName.c_str());
                parentEle->InsertEndChild(ele);
            }
            parentEle=ele;
        }
    }

    if(XML_SUCCESS!=doc.SaveFile(filename)) {doc.PrintError();return false;}
    return true;
}

bool exportEle(GSLAM::Svar& var,XMLElement* ele,string parentName="")
{
    if(!ele) return false;
    if(ele->Attribute("value"))
        var.insert((parentName.empty()?string():parentName+".")+ele->Name(),ele->Attribute("value"));
    XMLElement* child=ele->FirstChildElement();
    while(child)
    {
        exportEle(var,child,(parentName.empty()?string():parentName+".")+ele->Name());
        child=child->NextSiblingElement();
    }
    return true;
}

bool loadSvarXML(GSLAM::Svar& var,string filepath)
{
    XMLDocument doc;
    if(XML_SUCCESS!=doc.LoadFile(filepath.c_str())) return false;
    XMLElement* element=doc.FirstChildElement("project");
    element->SetName("");
    return exportEle(var,element);
}

}

using GSLAM::FramePtr;

class DroneMapSaver
{
public:
    DroneMapSaver(std::string datasetPath="dronemap"):firstFrame(true),datasetFolder(datasetPath){
        svar.ParseLine("system mkdir -p "+datasetPath+"/rgb");
        traj.open(datasetPath+"/trajectory.txt");
    }
    GSLAM::SO3 PYR2Rotation(double pitch,double yaw,double roll)
    {
        if(pitch>-30) pitch=-90;
        if(fabs(180-fabs(roll))<30) roll+=180;
        GSLAM::SO3 camera2IMU(-0.5,0.5,-0.5,0.5);
        GSLAM::SO3 imu2world;
        imu2world.FromEulerAngle(-pitch,90.-yaw,roll);
        return imu2world*camera2IMU;
    }
    void append(GSLAM::FramePtr frame){
        if(firstFrame)
        {
            firstFrame=false;
            GSLAM::Svar var;
            GSLAM::Camera cam=frame->getCamera();
            GSLAM::Point3d pt;
            frame->getGPSLLA(pt);
            plane2ecef.get_translation()=GSLAM::GPS<>::GPS2XYZ(pt.y,pt.x,0);
            double D2R=3.1415925/180.;
            double lon=pt.x*D2R;
            double lat=pt.y*D2R;
            GSLAM::Point3d up(cos(lon)*cos(lat), sin(lon)*cos(lat), sin(lat));
            GSLAM::Point3d east(-sin(lon), cos(lon), 0);
            GSLAM::Point3d north=up.cross(east);
            double R[9]={east.x, north.x, up.x,
                         east.y, north.y, up.y,
                         east.z, north.z, up.z};
            plane2ecef.get_rotation().fromMatrix(R);

            var.insert("Plane","0 0 0 0 0 0 1");
            var.insert("GPS.Origin",std::to_string(pt.x)+" "+std::to_string(pt.y)+" 0");

            var.insert("Camera.CameraType","PinHole");
            var.insert("Camera.Paraments",VecParament<double>(cam.getParameters()).toString());
            var.save2file(datasetFolder+"/config.cfg");
        }

        GSLAM::Point3d t;
        if(!frame->getGPSECEF(t)) return ;

        GSLAM::SE3 se3;
        se3.get_translation()=plane2ecef.inverse()*t;
        if(!frame->getPitchYawRoll(t)) return;
        se3.get_rotation()=PYR2Rotation(t.x,t.y,t.z);

        std::string time=std::to_string(frame->timestamp());

        traj<<time<<" "<<se3<<std::endl;

        GSLAM::GImage img=frame->getImage();
        QImage qimage(img.data,img.cols,img.rows,QImage::Format_ARGB32);
        qimage.save(QString::fromStdString(datasetFolder)+"/rgb/"+QString(time.c_str())+".jpg");
    }

    bool          firstFrame;
    GSLAM::SE3    plane2ecef;
    std::string   datasetFolder;
    std::ofstream traj;
};
class DatasetNetwork: public RTMapperDataset
{
public:
    DatasetNetwork():_frameId(0),_lastGrabId(0){}

    ~DatasetNetwork(){
        _shouldStop=true;
        while(!_readthread.joinable()) GSLAM::Rate::sleep(0.01);
        _readthread.join();
    }

    virtual std::string type() const{return "DatasetNetwork";}

    virtual bool        isOpened(){
        return true;
    }

    virtual bool        open(const std::string& dataset)
    {
        if(!tinyxml2::loadSvarXML(var,dataset))
        if(!var.ParseFile(dataset)) return false;

        ip  =var.GetString("Dataset.IP","127.0.0.1").c_str();
        port=var.GetInt("Dataset.Port",3000);
        _readthread=std::thread(&DatasetNetwork::run,this);
        return true;
    }

    void run()
    {
        _shouldStop=false;

        int        received=0;
        QByteArray imageBuffer;
        SPtr<QTcpSocket>  _socket=SPtr<QTcpSocket>(new QTcpSocket());
        while(!_shouldStop)
        {
            GSLAM::ScopedTimer decodeLoop("DecodeLoop");
            if(_socket->state()!=QTcpSocket::SocketState::ConnectedState)
            {
                GSLAM::ScopedTimer connect("Connect");
                _socket->connectToHost(ip,port,QTcpSocket::ReadWrite);

                if(_socket->waitForConnected(3000))
                {
                    _socket->write("DatasetNetwork\n");
                    _socket->flush();
                }
                GSLAM::Rate::sleep(0.1);
                continue;
            }
            {

                if(_socket->bytesAvailable()<sizeof(header))
                {
                    GSLAM::ScopedTimer connect("waitForReadyRead");
                    if(!_socket->waitForReadyRead(30)) continue;
                }
            }

            if(imageBuffer.isEmpty())
            {
                GSLAM::ScopedTimer connect("ReadHeader");
                if(_socket->bytesAvailable()<sizeof(header)) continue;

                int  lengthReceived = _socket->read((char*)&header,sizeof(header));
                header.convert();
                if(!header.isValid()) continue;

                if(header.payload_size<=0) continue;
                imageBuffer.resize(header.payload_size);
                received=0;
            }

            if(received<imageBuffer.size())
            {
                GSLAM::ScopedTimer connect("ReceiveImage");
                uint remaining = imageBuffer.size() - received;
                uint readed = _socket->read(imageBuffer.data() + received,
                                            std::min(remaining,(uint)_socket->bytesAvailable()));
                received += readed;
            }

            if(received>=imageBuffer.size())
            {
                GSLAM::ScopedTimer connect("Decode");
                decode(header,imageBuffer);
                imageBuffer.clear();
                received=0;
            }

        }
    }

    bool decode(RTMapperNetHeader& header,QByteArray& imageBuffer)
    {
        static bool shot=true;
        static RTMapperNetHeader shotHeader;
        if(shot) {shotHeader=header;shot=false;}
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

        QString cameraName=QString("%1_%2_%3").arg((char*)header.uavname).arg(image.width()).arg(image.height());
        cameraName.replace(' ','_');

        if(_cameraName!=cameraName)
        {
            _cameraName=cameraName;
            if(var.exist(_cameraName.toStdString()+".Paraments"))
            {
                VecParament<double> camP=var.get_var(_cameraName.toStdString()+".Paraments",VecParament<double>());
                _camera=GSLAM::Camera(camP.data);
            }
            else{
                std::string cam=_cameraName.toStdString();
                std::string camFolder = svar.GetString("MainWindow.CalibFolder",svar.GetString("MainWindow.DataFolder","")+"/calib");
                GSLAM::Svar camVar;
                camVar.ParseFile(camFolder+"/"+cam+".cam");
                VecParament<double> vecPara;
                vecPara=camVar.get_var(cam+".Paraments",vecPara);
                _camera=GSLAM::Camera(vecPara.data);
            }
            qDebug()<<"New camera:"<<cameraName;
        }

        shot=true;
        header=shotHeader;
        SPtr<RTMapperFrame> fr(new RTMapperFrame(_frameId++,header.timestamp));
        fr->_camera=_camera;
        fr->_cameraName=_cameraName.toStdString();
        fr->_image=qtg(image);
        fr->_gpshpyr={header.lng,header.lat,header.alt,
                      fabs(header.lng)<=180?5:1e6,fabs(header.lng)<=180?5:1e6,fabs(header.lng)<=180?10:1e6,
                      header.H,10,
                      header.cam_pitch,header.cam_yaw,header.cam_roll,10,10,10};
        _frame=fr;
        _handle->handle(fr);

        if(_frame->id()%10==0)
        {
            static DroneMapSaver saver("dronemap");
            saver.append(_frame);
        }

        return true;
    }

    GSLAM::GImage qtg(QImage qimage)
    {
        if(qimage.format()==QImage::Format_RGB32)
        {
            return GSLAM::GImage(qimage.height(),qimage.width(),
                       GSLAM::GImageType<uchar,4>::Type,qimage.bits(),true);
        }
        else if(qimage.format()==QImage::Format_RGB888){
            return GSLAM::GImage(qimage.height(),qimage.width(),
                       GSLAM::GImageType<uchar,3>::Type,qimage.bits(),true);
        }
        else if(qimage.format()==QImage::Format_Indexed8)
        {
            return GSLAM::GImage(qimage.height(),qimage.width(),
                       GSLAM::GImageType<uchar,1>::Type,qimage.bits(),true);
        }
        return GSLAM::GImage();
    }

    virtual FramePtr grabFrame(){
        while(!_frame) usleep(10000);
        if(_frameId>_lastGrabId&&_frame) {
            _lastGrabId=_frameId;
            return _frame;
        }
        else return _frame;
    }
    virtual bool  isOnline()const{return true;}

    virtual void call(const std::string &command, void *arg)
    {
        if("SetUpdatedCallback"==command)
        {
        }
    }

    GSLAM::GObjectHandle* _handle=NULL;
    GSLAM::Camera   _camera;
    QString         _cameraName;
    std::thread     _readthread;
    QString ip;
    quint16 port;

    GSLAM::Svar var;
    RTMapperNetHeader header;

    int             _frameId,_lastGrabId,_shouldStop;
    GSLAM::MutexRW  _mutexFrame;
    SPtr<RTMapperFrame>   _frame;
};
using GSLAM::DatasetFactory;

REGISTER_DATASET(DatasetNetwork,net);

