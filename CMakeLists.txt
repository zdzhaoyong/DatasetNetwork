cmake_minimum_required(VERSION 2.8)

include(cmake/PICMake.cmake)

INCLUDE_DIRECTORIES(src)

pi_add_target(gslamDB_net SHARED src/dbnet REQUIRED GSLAM Qt DEPENDENCY QtNetwork avcodec avfilter avutil avformat swscale)
pi_add_target(gslamDB_rtmv SHARED src/rtmv REQUIRED GSLAM Qt DEPENDENCY QtNetwork avcodec avfilter avutil avformat swscale)
pi_add_target(GCSServer BIN src/GCSServer REQUIRED OpenCV Qt DEPENDENCY QtNetwork)
pi_add_target(GCSClient BIN src/GCSClient REQUIRED Qt DEPENDENCY QtNetwork avcodec avfilter avutil avformat swscale )
pi_report_target()

pi_install(TARGETS gslamDB_net LIB_DESTINATION lib)
