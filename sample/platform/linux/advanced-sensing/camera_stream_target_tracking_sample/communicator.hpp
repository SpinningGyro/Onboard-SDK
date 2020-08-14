/*! @file communicator.hpp
 *  @version 0.1
 *  @date 2020.07.17
 *  @author Breeze, rencong
 *  @brief 
 * MSDK 与 OSDK 通讯类
*/ 
#ifndef COMMUNICATOR_HPP
#define COMMUNICATOR_HPP

#include "dji_mop_server.hpp"
#include "dji_linux_helpers.hpp"

void *runMop(void *args);

class Communicator {

    public:
        int isUpload;
        int isStart;
        int isPause;
        int isResume;
        int isStop;
        int isFocus;

        Communicator(Vehicle *vehicle);
        ~Communicator();
        void* MopClientTask();
        void* Sendmessege();
        void OPDownloadFileTask();
        int threadRun();
        bool sendMessage(std::string messages);

    private:
        Vehicle *vehicle;
        MopPipeline *MO_Pipeline;

};

#endif
