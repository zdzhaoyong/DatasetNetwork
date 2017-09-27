#pragma once
#include <GSLAM/core/Dataset.h>
#include <GSLAM/core/GPS.h>
#include <QImage>

enum RTMapperFrameStatus
{
    SLAMFRAME_READY,
    SLAMFRAME_PROCESSING,
    SLAMFRAME_PROCESSED_SUCCESS,
    SLAMFRAME_PROCESSED_FAILED
};

class RTMapperFrame : public GSLAM::MapFrame
{
public:
    RTMapperFrame(const GSLAM::FrameID& id=0,const double& timestamp=0)
        :GSLAM::MapFrame(id,timestamp),_status(SLAMFRAME_READY){}

    virtual std::string type()const{return "RTMapperFrame";}

    virtual int     cameraNum()const{return 1;}                                      // Camera number
    virtual GSLAM::SE3     getCameraPose(int idx=0) const{return GSLAM::SE3();}                    // The transform from camera to local
    virtual int     imageChannels(int idx=0) const{return GSLAM::IMAGE_RGBA;}               // Default is a colorful camera
    virtual GSLAM::Camera  getCamera(int idx=0){return _camera;}                            // The camera model
    virtual GSLAM::GImage  getImage(int idx,int channalMask){
        if(idx==0)
        {
            if(_image.empty())
            {
                using namespace GSLAM;
                QImage qimage(_imagePath.c_str());
                if(qimage.format()==QImage::Format_RGB32)
                {
                    return GImage(qimage.width(),qimage.height(),
                               GImageType<uchar,4>::Type,qimage.bits(),true);
                }
                else if(qimage.format()==QImage::Format_RGB888){
                    return GImage(qimage.width(),qimage.height(),
                               GImageType<uchar,3>::Type,qimage.bits(),true);
                }
                else if(qimage.format()==QImage::Format_Indexed8)
                {
                    return GImage(qimage.width(),qimage.height(),
                               GImageType<uchar,1>::Type,qimage.bits(),true);
                }
            }
            else  return _image;
        }
        else return _thumbnail;
    }   // Just return the image if only one channel is available

    // When the frame contains IMUs or GPSs
    virtual int     getIMUNum()const{return (_gpshpyr.size()==11||_gpshpyr.size()==12||_gpshpyr.size()==14)?1:0;}
    virtual bool    getAcceleration(GSLAM::Point3d& acc,int idx=0)const{return false;}        // m/s^2
    virtual bool    getAngularVelocity(GSLAM::Point3d& angularV,int idx=0)const{return false;}// rad/s
    virtual bool    getMagnetic(GSLAM::Point3d& mag,int idx=0)const{return false;}            // gauss
    virtual bool    getAccelerationNoise(GSLAM::Point3d& accN,int idx=0)const{return false;}
    virtual bool    getAngularVNoise(GSLAM::Point3d& angularVN,int idx=0)const{return false;}
    virtual bool    getPitchYawRoll(GSLAM::Point3d& pyr,int idx=0)const{
        if(_gpshpyr.size()==11&&_gpshpyr[8]<20) {pyr=GSLAM::Point3d(_gpshpyr[5],_gpshpyr[6],_gpshpyr[7]);return true;}
        else if(_gpshpyr.size()==14&&_gpshpyr[11]) {pyr=GSLAM::Point3d(_gpshpyr[8],_gpshpyr[9],_gpshpyr[10]);return true;}
        else if(_gpshpyr.size()==12&&_gpshpyr[9]<20) {pyr=GSLAM::Point3d(_gpshpyr[6],_gpshpyr[7],_gpshpyr[8]);return true;}
        return false;
    }     // in rad
    virtual bool    getPYRSigma(GSLAM::Point3d& pyrSigma,int idx=0)const{
        if(_gpshpyr.size()==11&&_gpshpyr[8]<20) {pyrSigma=GSLAM::Point3d(_gpshpyr[8],_gpshpyr[9],_gpshpyr[10]);return true;}
        else if(_gpshpyr.size()==14&&_gpshpyr[11]) {pyrSigma=GSLAM::Point3d(_gpshpyr[11],_gpshpyr[12],_gpshpyr[13]);return true;}
        else if(_gpshpyr.size()==12&&_gpshpyr[9]<20) {pyrSigma=GSLAM::Point3d(_gpshpyr[9],_gpshpyr[10],_gpshpyr[11]);return true;}
        return false;
    }    // in rad

    virtual int     getGPSNum()const{return (_gpshpyr.size()>=6&&_gpshpyr[3]<10)?1:0;}
    virtual bool    getGPSLLA(GSLAM::Point3d& LonLatAlt,int idx=0)const{
        if(getGPSNum()==0) return false;
        LonLatAlt=GSLAM::Point3d(_gpshpyr[0],_gpshpyr[1],_gpshpyr[2]);
        return _gpshpyr[3]<10;
    }        // WGS84 [longtitude latitude altitude]
    virtual bool    getGPSLLASigma(GSLAM::Point3d& llaSigma,int idx=0)const{
        if(_gpshpyr.size()>=6||_gpshpyr.size()==8||_gpshpyr.size()==12||_gpshpyr.size()==14)
        {llaSigma=GSLAM::Point3d(_gpshpyr[3],_gpshpyr[4],_gpshpyr[5]);return true;}
        else if(_gpshpyr.size()==7) {llaSigma=GSLAM::Point3d(_gpshpyr[3],_gpshpyr[3],_gpshpyr[4]);return true;}
        return false;
    }    // meter
    virtual bool    getGPSECEF(GSLAM::Point3d& xyz,int idx=0)const{
        GSLAM::Point3d lla;
        if(!getGPSLLA(lla)) return false;
        xyz=GSLAM::GPS<>::GPS2XYZ(lla.y,lla.x,lla.z);
        return true;
    }             // meter
    virtual bool    getHeight2Ground(GSLAM::Point2d& height,int idx=0)const{
        if(_gpshpyr.size()==14||_gpshpyr.size()==8){height=GSLAM::Point2d(_gpshpyr[6],_gpshpyr[7]);return _gpshpyr[7]<100;}
        return false;
    }    // height against ground

    virtual void call(const std::string &command, void *arg)
    {
        if("GetImagePath"==command)
        {
            if(arg) *(std::string*)arg=_imagePath;
        }
        else if("ReleaseImage"==command)
        {
            _image=GSLAM::GImage();
        }
        else if("GetGPS"==command)
        {
            if((!arg)||_gpshpyr.empty()) return;
            std::vector<double>* dest=(std::vector<double>*)arg;
            *dest=_gpshpyr;
        }
    }

    std::string imagePath(){return _imagePath;}
    GSLAM::GImage& thumbnail(){return _thumbnail;}

    std::string         _imagePath;       // where image come from or where to cache
    std::string         _cameraName;
    GSLAM::GImage       _image,_thumbnail;// should be RGBA
    GSLAM::Camera       _camera;
    GSLAM::FramePtr     _slamFrame;
    RTMapperFrameStatus _status;
    std::vector<double> _gpshpyr;
    // 6 : long,lat,alt,sigmaX,sigmaY,sigmaZ
    // 8 : long,lat,alt,sigmaX,sigmaY,sigmaZ,Height,sigmaH
    // 11: long,lat,alt,sigmaH,sigmaV,pitch,yaw,roll,sigmaP,sigmaR,sigmaP
    // 12: long,lat,alt,sigmaX,sigmaY,sigmaZ,pitch,yaw,roll,sigmaP,sigmaR,sigmaP
    // 14: long,lat,alt,sigmaX,sigmaY,sigmaZ,Height,sigmaH,pitch,yaw,roll,sigmaP,sigmaR,sigmaP
};

class RTMapperDataset : public GSLAM::Dataset
{
public:
    virtual bool  obtainPreparedFrames(std::vector<SPtr<RTMapperFrame> >& frames){return false;} // things for visualize
    virtual float processedPercentage()const{return 0;}
    virtual bool  getCameras(std::vector<std::pair<std::string,GSLAM::Camera> >& cameras){return false;}
};
