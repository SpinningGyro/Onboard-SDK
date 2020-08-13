#include "communicator.hpp"
#include "dji_linux_helpers.hpp"
#include <openssl/md5.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <dirent.h>
#include <stdio.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include "mop_protocol.hpp"
#include <map>

#include "mop.h"
#include "dji_mop_define.hpp"

using namespace DJI::OSDK;

#define TEST_OP_PIPELINE_ID 49153
#define TEST_MO_PIPELINE_ID 20
#define READ_ONCE_BUFFER_SIZE 200*1024
#define SEND_ONCE_BUFFER_SIZE 200*1024
#define TEST_RECV_FILE_NAME "/home/nvidia/mop.txt"
#define TEST_SEND_INFO "TEST_SEND_INFO sucess!"

PipelineType pip_type = RELIABLE;
typedef enum OPDownloadSampleState {
//   DOWNLOAD_TASK_IDLE,
//   REQUEST_FILE_DOWNLOAD,
//   RECV_FILE_DOWNLOAD_ACK,
//   SEND_FILE_NAME,
  RECV_FILE_INFO_DATA,
  RECV_FILE_RAW_DATA,
  MD5_RESULT_CHECK,
  RECV_MESSAGE_ACK,
//   DOWNLOAD_TASK_FINISH,
} OPUploadSampleState;


Communicator::Communicator(Vehicle *vehicle){
    isUpload = 0;
    isStart = 0;
    isPause = 0;
    isResume = 0;
    isStop = 0;
    vehicle = vehicle;
    MO_Pipeline = NULL;
}

Communicator::~Communicator(){
}

void Communicator::OPDownloadFileTask(){
    
    fileInfo downloadFileInfo = {0};
    MopErrCode mopRet;
    OPUploadSampleState uploadState = RECV_FILE_INFO_DATA;
    

    /*! MD5 prepare*/
    MD5_CTX md5Ctx;
    unsigned char md5_out[16];

    fstream f("txt_out_comm.txt", ios::out);

    while(true) {
        /*连接中断后，判断连接状态进行重新连接，需要APP具有自动连接功能*/
        //MO_Pipeline = NULL;
        //if (!vehicle->initMopServer())
        //    {
        //        DERROR("Failed to initialize MopServer!\n");
        //    }
        //if(pipelineMap.find((PipelineID)TEST_MO_PIPELINE_ID == pipelineMap.end())){ 
        //PipelineID id = 20;
        //if(pipelineMap.find(id) != pipelineMap.end()){
        //    delete pipelineMap[id];
        //    pipelineMap.erase(id);
        //}
        //mop_channel_handle_t handler = pipelineMap[id];
	//int ret = mop_destroy_channel(handler);
        //std::cout << "######### accept ##########\n";
        //if (vehicle->mopServer->accept((PipelineID)TEST_MO_PIPELINE_ID, RELIABLE, MO_Pipeline) != MOP_PASSED) {
        //    DERROR("MOP Pipeline accept failed");
        //}
//}

        // Step 4: 接收信息指令： 0：起飞，1：暂停，2：继续，3：结束
        uint8_t recvBuf[READ_ONCE_BUFFER_SIZE] = {0};
        MopPipeline::DataPackType readPack = {(uint8_t *) recvBuf, READ_ONCE_BUFFER_SIZE};
        mopRet = MO_Pipeline->recvData(readPack, &readPack.length);
        //if(mopRet != MOP_PASSED){
        if(mopRet == MOP_CONNECTIONCLOSE){
            PipelineID id = 20;
            int32_t ret;
            mop_channel_handle_t bind_handle;
            mop_close_channel(pipelineMap[id]);
            mop_destroy_channel(pipelineMap[id]);
            vehicle->mopServer->close(id);
            delete pipelineMap[id];
            pipelineMap.erase(id); 
            /*! 1.Create handler for binding */
            DSTATUS("/*! 1.Create handler for binding */");
            ret = mop_create_channel(&bind_handle, (mop_trans_t)pip_type);
            if (MOP_SUCCESS != ret) {
                DERROR("MOP create channel failed");
            }
            
            /*! 2.Do binding */
            DSTATUS("/*! 2.Do binding */");
            ret = mop_bind_channel(bind_handle, id);
            if (ret != MOP_SUCCESS) {
                DERROR("MOP Pipeline bind failed");
             }
            /*! 3.Do accepting */
            MopPipeline *p = new MopPipeline(id, pip_type);
            if (!p) {
                DERROR("Pipeline create failed");                
            }
            DSTATUS("/*! 3.Do accepting */");
            DSTATUS("Do accepting blocking for channel [%d] ...", id);
            ret = mop_accept_channel(bind_handle, &p->channelHandle);
            if (MOP_SUCCESS != ret) {
                DERROR("MOP accept failed");
            }
            /*! 4.Accept finished */
            DSTATUS("/*! 4.Accept finished */");
            MO_Pipeline = p;
            pipelineMap[id] = p;
            DSTATUS("MOP channel [%d] accepted success", id);
            continue;
        }        

        sampleProtocolStruct *ackData = (sampleProtocolStruct *)readPack.data;
        // std::cout << "ackData->cmd: " << ackData->cmd << std::endl;
        // std::cout << "ackData->subcmd: " << ackData->subcmd << std::endl;
        // std::cout << "ackData->cmd == CMD_FLY: " <<  CMD_FLY << std::endl;

        if (ackData->cmd == CMD_FLY) {
            std::cout << "设置拍照\n" << std::endl; 
            f << "设置拍照\n";        
            isStart = 1;
            // vehicle->mopServer->close((PipelineID)TEST_MO_PIPELINE_ID);
        }
        if (ackData->cmd == CMD_FOCUS) {
            isFocus = 1;
        }
        // else {
        //     DERROR("起飞命令接收失败！");       
        //     // TODO
        //     // 往 MSDK 发送状态     
        // }
        if (ackData->cmd == CMD_PAUSE) {            
            isPause = 1;
        } 
        // else {
        //     DERROR("暂停命令接收失败！");       
        //     // TODO
        //     // 往 MSDK 发送状态    
        // }
        
        if (ackData->cmd == CMD_RESUME) {            
            isResume = 1;
        } 
        // else {
        //     DERROR("继续命令接收失败！");       
        //     // TODO
        //     // 往 MSDK 发送状态    
        // }

        if (ackData->cmd == CMD_STOP) {            
            isStop = 1;                    
            uploadState = RECV_FILE_INFO_DATA;
        }
        // else {
        //     DERROR("继续命令接收失败！");       
        //     // TODO
        //     // 往 MSDK 发送状态    
        // }

    }  
}

void* Communicator::MopClientTask(){
    fstream f("txt_out_mop.txt", ios::out);
    
    if (!vehicle->initMopServer())
    {
        DERROR("Failed to initialize MopServer!\n");
        f << "Failed to initialize MopServer!\n";
    }
    if (vehicle->mopServer->accept((PipelineID)TEST_MO_PIPELINE_ID, RELIABLE, MO_Pipeline) != MOP_PASSED) {
        DERROR("MOP Pipeline accept failed");
        f << "Failed to initialize MopServer!\n";
        return 0;
    } else {
        DSTATUS("Accept to mop pipeline id(%d) successfully", TEST_MO_PIPELINE_ID);
        f << "Accept to mop pipeline id(%d) successfully\n";
    }

    if (!MO_Pipeline) std::runtime_error("Error param");
    /*! OSDK upload file to PSDK */
    OPDownloadFileTask();
    f << "通道建立成功！\n";
}

void *runMop(void *args){
    auto *pCom = static_cast<Communicator *>(args);
    pCom->MopClientTask();
}

int Communicator::threadRun(){
    int result;
    pthread_t clientTask;
    result = pthread_create(&clientTask, NULL, runMop, this);
    return result;
}

bool Communicator::sendMessage(std::string messages){
    if (!MO_Pipeline)std::runtime_error("Error param");
  fileInfo dSownloadFileInfo = {0};
  MopErrCode mopRet;

  uint8_t recvBuf[READ_ONCE_BUFFER_SIZE] = {0};

  /*! send file download request */
  DSTATUS("Send messege to MSDK ...");
  sampleProtocolStruct req = {
          .cmd = CMD_DL_FILENAME,
          .subcmd = 0xFF,
          .seq = 0,
          .dataLen = sizeof(req.data.targetFile)
      };
    //    const char* str = "send data sucess.txt";
    // std::string mesg = "已经起飞";
  sprintf(req.data.targetFile.fileName, "%s",messages.c_str());
//   req.data.targetFile.fileName = mesg.c_str();
  
  MopPipeline::DataPackType fileInfoPack = {.data = (uint8_t *) &req, .length = 6 + req.dataLen};
  mopRet = MO_Pipeline->sendData(fileInfoPack, &fileInfoPack.length);
  DSTATUS("Send messege to MSDK finished...");
  return true;
}
