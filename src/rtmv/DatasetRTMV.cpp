#include "RTMapperDataset.h"
#include "RTMapperNetInterface.h"
#include <GSLAM/core/VecParament.h>
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
            LOG(ERROR)<<("m_pAVCodec == NULL||m_pCodecCtx == NULL");
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
            LOG(ERROR)<<(" CDecoder avcodec_alloc_frame() == NULL ");
            return 0;
        }
        LOG(ERROR)<<("CDecoder::prepare()2");
        return 1;
    }

    bool decode(RTMapperNetHeader& header,QByteArray& imageBuffer,GSLAM::GImage& image)
    {
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
                AVPacket packet;
                av_init_packet(&packet);
                if (m_pCodecPaser == NULL) {
                    LOG(ERROR)<<("m_pCodecPaser == NULL");
                    initialize();
                }
                if (m_pCodecCtx == NULL) {
                    LOG(ERROR)<<("m_pCodecCtx == NULL");
                    initialize();
                }
                paserLen = av_parser_parse2(m_pCodecPaser, m_pCodecCtx, &packet.data, &packet.size, buff,
                        parseSize, AV_NOPTS_VALUE, AV_NOPTS_VALUE, AV_NOPTS_VALUE);

                localBuffer.remove(0,paserLen);

                if (packet.size > 0)
                {
                    int got;
                    paserLen= avcodec_decode_video2(m_pCodecCtx,m_pYUVFrame,&got,&packet);
                    if(paserLen<0)
                    {
                        LOG(ERROR)<<"Error while decoding frame.";
                        av_free_packet(&packet);
                        return false;
                    }
                    if(got)
                    {
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

                        sws_scale(m_pImgCtx,(const uint8_t* const*)m_pYUVFrame->data,
                                  m_pYUVFrame->linesize,0,m_pCodecCtx->height,avPicture->data,avPicture->linesize);
                        image=GSLAM::GImage(m_pCodecCtx->height,m_pCodecCtx->width,GSLAM::GImageType<uchar,4>::Type,
                                            avPicture->data[0]).clone();

                        av_free_packet(&packet);
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
        _ifs.open(dataset);
        _readthread=std::thread(&DatasetNetwork::run,this);
        return _ifs.is_open();
    }

    void run()
    {
        _shouldStop=false;
        if(!_ifs.is_open()) return ;

        QByteArray imageBuffer;
        int received=0;
        while(!_shouldStop)
        {
            if(imageBuffer.isEmpty())
            {
                _ifs.read((char*)&header,sizeof(header));
                header.convert();
                if(!header.isValid()) continue;

                if(header.payload_size<=0) continue;
                imageBuffer.resize(header.payload_size);
                received=0;
                if(startTimestamp<0)
                {
                    startTimestamp=GSLAM::TicToc::timestamp()-header.timestamp;
                }
                else
                {
                    GSLAM::Rate::sleep(startTimestamp+header.timestamp-GSLAM::TicToc::timestamp());
                }
            }

            if(received<imageBuffer.size())
            {
                uint remaining = imageBuffer.size() - received;
                _ifs.read(imageBuffer.data() + received,remaining);
                received=remaining;
            }

            if(received>=imageBuffer.size())
            {
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
        GSLAM::GImage image;
        if(header.video_codec==VIDEOCODEC_JPEG)
        {
//            image.loadFromData(imageBuffer,QString("JPG").toLocal8Bit().data());
        }
        else if(header.video_codec==VIDEOCODEC_H264)
        {
            static Decoder decoder;
            if(!decoder.decode(header,imageBuffer,image)) return false;
        }
        else if(header.video_codec==VIDEOCODEC_NONE)
        {
//            image=QImage((uchar*)imageBuffer.data(),header.display_width,header.display_height,
//                         QImage::Format_RGB888).copy();
        }

        if(image.empty()) return false;

        QString cameraName=QString("%1_%2_%3").arg((char*)header.uavname).arg(image.cols).arg(image.rows);
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
            LOG(ERROR)<<"New camera:"<<cameraName.toStdString();
        }

        shot=true;
        header=shotHeader;
        SPtr<RTMapperFrame> fr(new RTMapperFrame(_frameId++,header.timestamp));
        fr->_camera=_camera;
        fr->_cameraName=_cameraName.toStdString();
        fr->_image=image;
        fr->_gpshpyr={header.lng,header.lat,header.alt,
                      fabs(header.lng)<=180?5:1e6,fabs(header.lng)<=180?5:1e6,fabs(header.lng)<=180?10:1e6,
                      header.H,10,
                      header.cam_pitch,header.cam_yaw,header.cam_roll,10,10,10};
        _frame=fr;
        if(_handle)
        _handle->handle(fr);

        return true;
    }

    virtual GSLAM::FramePtr grabFrame(){
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
    std::ifstream   _ifs;

    GSLAM::Svar       var;
    RTMapperNetHeader header;

    int             _frameId,_lastGrabId,_shouldStop;
    GSLAM::MutexRW  _mutexFrame;
    SPtr<RTMapperFrame>   _frame;

    double startTimestamp=-1;
};
using GSLAM::DatasetFactory;

REGISTER_DATASET(DatasetNetwork,rtmv);

