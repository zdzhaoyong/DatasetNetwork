#ifndef RTMAPPERNETINTERFACE_H
#define RTMAPPERNETINTERFACE_H

#include "stdint.h"
#include "string.h"

enum VideoCodecType
{
    VIDEOCODEC_NONE,
    VIDEOCODEC_JPEG,
    VIDEOCODEC_H264,
    VIDEOCODEC_H265
};

enum VideoFrameType
{
    FRAMETYPE_IFRAME,
    FRAMETYPE_PFRAME
};

struct RTMapperNetHeader // 128 bytes
{
    RTMapperNetHeader()
        : signature(get_signature()),version(1),control_quality(255),video_quality(255),
          header_size(sizeof(RTMapperNetHeader)),timestamp(0),battery(1),
          vx(0),vy(0),vz(0),lat(0),lng(0),alt(0),H(0),HDOP_H(0),HDOP_V(0),
          nSat(0),uav_roll(0),uav_pitch(0),uav_yaw(0),cam_roll(0),cam_pitch(0),cam_yaw(0),
          payload_size(0),video_codec(VIDEOCODEC_JPEG),frame_type(FRAMETYPE_IFRAME),
          stream_width(0),stream_height(0),display_width(0),display_height(0){
    }

    template <typename T>
    void convert(T& t)
    {
        T copy=t;
        char* it=sizeof(t)-1+(char*)&t;
        char* c=(char*)&copy;
        for(int i=0;i<sizeof(t);i++)
        {
            *(it--)=*(c++);
        }
    }
    void convert()
    {
        if(signature==get_signature()) return;
        convert(signature);
        convert(timestamp);
        convert(battery);
        convert(vx);
        convert(vy);
        convert(vz);
        convert(lat);convert(lng);
        convert(alt);convert(H);convert(HDOP_H);convert(HDOP_V);
        convert(nSat);convert(uav_roll);convert(uav_pitch);convert(uav_yaw);
        convert(cam_roll);convert(cam_pitch);convert(cam_yaw);
        convert(payload_size);convert(stream_width);convert(stream_height);
        convert(display_width);convert(display_height);
    }

    static int32_t get_signature(){
        return 1348556357;
    }

    bool isValid(){return signature==get_signature();}

    // 16 bytes
    int32_t     signature;      // "PaVE" - used to identify the start of frame
    uint8_t     version;        // Version code, current version 1 */
    uint8_t     control_quality;// Current connect quality for control commands
    uint8_t     video_quality;  // Current connect quality for video transfer
    uint8_t     header_size;    // Size of the RTMapperNetHeader */
    double      timestamp;      // In milliseconds */

    // The UAV status
    float       battery;        //< battery percent, 32
    float       vx;             //< forward velocity
    float       vy;             //< leftward velocity
    float       vz;             //< upward velocity

    double      lat;            //< latitude, 48
    double      lng;            //< longitude

    float       alt;            //< alititude
    float       H;              //< height above ground
    float       HDOP_H;         //< HDOP (horizontal, unit is meter)
    float       HDOP_V;         //< HDOP (vertical, unit is meter)

    int32_t     nSat;           //< Satellite number, 96
    float       uav_roll;       //< attitude - roll, 104
    float       uav_pitch;      //< attitude - pitch
    float       uav_yaw;        //< attitude - yaw

    float       cam_roll;       //< attitude - camera roll
    float       cam_pitch;      //< attitude - camera pitch
    float       cam_yaw;        //< attitude - camera yaw

    // The Image/Video Payload
    uint32_t    payload_size;   /* Amount of data following this PaVE */

    uint8_t     video_codec;    /* Codec of the following frame : VideoCodecType, 140 */
    uint8_t     frame_type;     /* I-frame, P-frame*/
    uint16_t    stream_width;   /* ex: 640 */
    uint16_t    stream_height;  /* ex: 368 */
    uint16_t    display_width;  /* ex: 640 */
    uint16_t    display_height; /* ex: 360 */
    char        uavname[20];  //< UAV Name
};


#endif // RTMAPPERNETINTERFACE_H
