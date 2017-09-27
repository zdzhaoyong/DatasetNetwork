
#include <iostream>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "Server.h"
#include "../dbnet/RTMapperNetInterface.h"



using std::cout;
using std::endl;

int main(int argc,char** argv)
{
    if(argc<2)
    {
        cout<<"Usage:\n  GCSServer VideoFile [port]\n";
        return 0;
    }
    QApplication app(argc,argv);
    cv::VideoCapture video(argv[1]);
    if(!video.isOpened()) return -1;

    Server server;
    if(argc>2)
        server.init(std::stoi(argv[2]));
    else server.init();

    RTMapperNetHeader header;
    memcpy(header.signature,RTMapperNetHeader::get_signature(),4);
    header.video_codec=VIDEOCODEC_JPEG;
    while(true)
    {
        if(server.m_mapClient.empty())
        {
            cv::waitKey(10);
            app.processEvents();
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
        app.processEvents();
    }
    return 0;
}
