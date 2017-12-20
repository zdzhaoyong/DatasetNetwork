
#include <iostream>
#include <fstream>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <GSLAM/core/Timer.h>

#include "Server.h"
#include "RTMapperNetInterface.h"

Server server;

using std::string;
using std::cout;
using std::endl;
using namespace std;


int sendRtmv(string filepath)
{
    ifstream ifs(filepath.c_str());
    if(!ifs.is_open()) return -1;

    RTMapperNetHeader header;

    double startTimestamp=-1;

    while(ifs.read((char*)&header,sizeof(header)))
    {
        header.convert();
        if (!header.isValid())
        {
            cerr<<"Header not PaVE."<<endl;
            continue;
        }
        RTMapperNetHeader headerCopy=header;
        header.convert();
        if(header.payload_size<=0)
        {
            continue;
            cerr<<"Payload size:"<<header.payload_size<<endl;
        }
        QByteArray message;
        message.append(QByteArray((char*)&headerCopy,sizeof(header)));
        char* buf=new char[header.payload_size];
        ifs.read(buf,header.payload_size);
        message.append(QByteArray(buf,header.payload_size));
        delete[] buf;
        if(startTimestamp<0)
        {
            startTimestamp=GSLAM::TicToc::timestamp()-header.timestamp;
        }
        else
        {
            GSLAM::Rate::sleep(startTimestamp+header.timestamp-GSLAM::TicToc::timestamp());
        }
        server.sendData(message);
        QApplication::instance()->processEvents();
    }
}

int sendVideo(string filepath)
{
    cv::VideoCapture video(filepath);
    if(!video.isOpened()) return -1;
    RTMapperNetHeader header;
    header.video_codec=VIDEOCODEC_JPEG;
    while(true)
    {
        if(server.m_mapClient.empty())
        {
            cv::waitKey(10);
            QApplication::instance()->processEvents();
            continue;
        }
        cv::Mat mat;
        video>>mat;
        if(mat.empty()) break;
        cv::resize(mat,mat,cv::Size(640,480));
        cv::imshow("ServerImage",mat);
        std::vector<uchar> buf;
        if(header.video_codec==VIDEOCODEC_JPEG)
        {
            cv::imencode(".jpeg",mat,buf);
        }
        else if(header.video_codec==VIDEOCODEC_NONE)
        {
            cv::cvtColor(mat,mat,CV_BGR2RGB);
            header.display_height=mat.rows;
            header.display_width=mat.cols;
            buf.resize(mat.cols*mat.rows*3);
            memcpy(buf.data(),mat.data,buf.size());
        }

        header.payload_size=buf.size();
        header.timestamp+=0.1;
        QByteArray message;
        message.append(QByteArray((char*)&header,sizeof(header)));
        message.append(QByteArray((char*)buf.data(),buf.size()));
        server.sendData(message);
        cout<<"New frame sended by mode "<<header.video_codec<<endl;
        cv::waitKey(10);
        QApplication::instance()->processEvents();
    }
}

int main(int argc,char** argv)
{
    if(argc<2)
    {
        cout<<"Usage:\n  GCSServer VideoFile [port]\n";
        return 0;
    }
    QApplication app(argc,argv);

    if(argc>2)
        server.init(std::stoi(argv[2]));
    else server.init();

    string filePath=argv[1];
    if(filePath.find(".rtmv")!=string::npos) return sendRtmv(filePath);
    else return sendVideo(filePath);
    return 0;
}
