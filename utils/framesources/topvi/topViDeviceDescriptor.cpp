#include <iostream>

#include "topViDeviceDescriptor.h"
#include "topViCapture.h"

#include "tpv.h"

#include <sys/types.h>

#ifdef Q_OS_UNIX
  #include <sys/socket.h>
  #include <netdb.h>

  #define READ(s, buf, len)     read(s, buf, len)
  #define WRITE(s, buf, len)	write(s, buf, len)
  #define CLOSE(s)              close(s)
  #define ERROR_TO_STR          gai_strerror
#else
  #include <winsock2.h>
  #include <ws2tcpip.h>

  #define READ(s, buf, len)     recv(s, (char*)(buf), len, 0)
  #define WRITE(s, buf, len)    send(s, (char*)(buf), len, 0)
  #define CLOSE(s)              { shutdown(s, SD_BOTH); closesocket(s); }
  typedef int socklen_t;
  #define ERROR_TO_STR          gai_strerrorA                
#endif

void TopViDeviceDescriptor::CmdSpinThread::run()
{
    int             fd;
    struct addrinfo	hints;
    struct addrinfo	*result, *rp;
    int             retval;

#ifndef Q_OS_UNIX
    WORD wVersionRequested;
    WSADATA wsaData;
    int err;

    wVersionRequested = MAKEWORD(2, 2);
    err = WSAStartup(wVersionRequested, &wsaData);
    if (err != 0){
            SYNC_PRINT(("WSAStartup failed with error: %d\n", err));
            return;
    }

    if (LOBYTE(wsaData.wVersion) != 2 || HIBYTE(wsaData.wVersion) != 2){
            SYNC_PRINT(("Could not find a usable version of Winsock.dll\n"));
            WSACleanup();
            return;
    }
#endif

    memset(&hints, 0, sizeof(struct addrinfo));
    hints.ai_family = AF_UNSPEC;     /* Allow IPv4 or IPv6 */
    hints.ai_socktype = SOCK_STREAM; /* TCP socket */
    hints.ai_flags = 0;
    hints.ai_protocol = 0;           /* Any protocol */

xxx:
    //string addr = "localhost";
    //string addr = "193.232.110.156";
    string addr = "217.71.226.214";
    int port = 8002;

    retval = getaddrinfo(addr.c_str(), to_string(port).c_str(), &hints, &result);
    if (retval != 0)
    {
       fprintf(stderr, "Incorrect address %s\n", ERROR_TO_STR(retval));
       sleep(5);
       goto xxx;
    }

    for (rp = result; rp != NULL; rp = rp->ai_next) {
       fd = socket(rp->ai_family, rp->ai_socktype, rp->ai_protocol);
       if (fd == -1)
           continue;
       if (::connect(fd, rp->ai_addr, rp->ai_addrlen) != -1)
           break;
       CLOSE(fd);
    }

    if (rp == NULL) {
        fprintf(stderr, "Could not connect to <%s>. Waiting 5s for power ON\n", (addr + ":" + to_string(port)).c_str());
        freeaddrinfo(result);
        sleep(5);
        goto xxx;
    }

    freeaddrinfo(result);

    SYNC_PRINT(("TopViDeviceDescriptor::CmdSpinThread(): new command thread running\n"));

    while (device->cmdSpinRunning.tryLock())
    {
        long long length = 0, retval = 0;
        char s[0xFFF +1];

        device->protectGrillRequest.lock();

        QMutableListIterator<TopViGrillCommand*> it(device->grillInterface.commandList);
        while (it.hasNext())
        {
            TopViGrillCommand *cmd = it.next();
            int result = false;

            //new command found
            if (cmd->state == TPV_UNKNOWN) {

                SYNC_PRINT(("TopViGrillInterface::CmdSpinThread(): SEND cmdId = %d, cmdType = %s, cmdName = %s\n", cmd->cmdId, tpvCmdType(cmd->cmdType), tpvCmdName(cmd->cmdName)));

                length = strlen(cmd->request->requestStr);
                try {
                    retval = WRITE(fd, cmd->request->requestStr, length);
                }
                catch(...) {
                    SYNC_PRINT(("Broken pipe\n"));
                    CLOSE(fd);
                    goto xxx;
                }

                if (retval != length)
                {
                    if (retval < 0) SYNC_PRINT(("TopViGrillInterface::CmdSpinThread(): write error: %s\n", strerror(errno)));
                    break;
                }

                device->protectGrillReply.lock();
                cmd->state = TPV_QUERY;
                device->protectGrillReply.unlock();

                cmd->state = tpvCmdNeedReplay(cmd->cmdType);

                if (cmd->state == TPV_WAIT_REPLY) {
#if 0
                    string serverReply(tpvGenerateTestReply(cmd->cmdType));
#else
                    retval = READ(fd, s, sizeof(s) - 1);
                    if (retval <= 0){
                        if (retval < 0)
                            SYNC_PRINT(("TopViGrillInterface::CmdSpinThread(): read error %s\n", strerror(errno)));
                        break;
                    }

                    while (retval > 0 && (s[retval - 1] == '\r' || s[retval - 1] == '\n')) {
                        retval--;
                    }
                    s[retval] = '\0';
                    string serverReply(s);
#endif
                    SYNC_PRINT(("TopViGrillInterface::CmdSpinThread(): server reply %s\n", serverReply.c_str()));

                    device->grillInterface.addReplyToCommand(serverReply);
                    device->protectGrillReply.lock();
                    cmd->state = TPV_REPLY;
                    device->protectGrillReply.unlock();
                    SYNC_PRINT(("TopViGrillInterface::CmdSpinThread(): before callback cmdId = %d, cmdName = %s, cmdStatus = %s\n", cmd->cmdId, tpvCmdName(cmd->cmdName), tpvCmdStatus(cmd->state)));

                }
             }

            //other commands in list
            if (cmd->parent && cmd->state != TPV_NO_REPLY && cmd->state != TPV_OK) {
               result = cmd->parent->replyCallback(cmd);
            }
            else {
               result = device->replyCallback(cmd);
            }
            if (result) {
               device->protectGrillReply.lock();
               cmd->state = TPV_OK;
               device->protectGrillReply.unlock();
             }
             else {
                device->protectGrillReply.lock();
                cmd->state = TPV_NO_REPLY;
                device->protectGrillReply.unlock();
             }
             SYNC_PRINT(("TopViGrillInterface::CmdSpinThread(): after callback cmdId = %d, cmdStatus = %s\n", cmd->cmdId, tpvCmdStatus(cmd->state)));
        }
        device->protectGrillRequest.unlock();

        usleep(200);

        device->protectGrillRequest.lock();
        device->grillInterface.removeOldCommands();
        device->protectGrillRequest.unlock();

        device->cmdSpinRunning.unlock();
        if (device->shouldStopCmdSpinThread)
        {
            SYNC_PRINT(("Break command to cmd thread received\n"));
            break;
        }
    }
    CLOSE(fd);
    SYNC_PRINT(("TopViDeviceDescriptor::CmdSpinThread(): command thread finished\n"));
}

int TopViDeviceDescriptor::init(int _deviceId)
{
    SYNC_PRINT(("TopViDeviceDescriptor::init(): init called\n"));
    this->deviceId = _deviceId;
    if (inited.testAndSetAcquire(0,1))
    {
        this->deviceId = deviceId;
        this->cmdSpinRunning.lock();
        this->shouldStopCmdSpinThread = false;
        this->cmdSpinRunning.unlock();
        cmdSpin.device = this;
        cmdSpin.start();
        this->createAllCameras();
        this->executeCommand(TPV_SET, TPV_INIT, 0, "", "", NULL);
    }
    return 0;
}

int TopViDeviceDescriptor::createAllCameras()
{
    SYNC_PRINT(("TopViDeviceDescriptor: %d cameras was NOT inited\n", mCamerasNumber));
    for (int i = 0; i < mCamerasNumber; i++ ) {
        TopViCameraDescriptor *camera = new TopViCameraDescriptor(i+1);
        if (camera->inited.testAndSetAcquire(0,1)) {
            mCameras.push_back(camera);
        }
    }
    SYNC_PRINT(("TopViDeviceDescriptor: cameras are inited now\n"));
    this->executeCommand(TPV_GET, TPV_STATUS, 0, "", "", NULL);
    return mCamerasNumber;
}

int TopViDeviceDescriptor::getCamerasSysId(vector<string> &camDesc)
{
    if (inited) {
        SYNC_PRINT(("TopViDeviceDescriptor: getCamerasSysId for inited\n"));
        for (int i = 0; i < mCamerasNumber; i++) {
            TopViCameraDescriptor *camera = mCameras[i];
            camDesc.push_back(camera->getSysId());
        }
    }
    else {
        SYNC_PRINT(("TopViDeviceDescriptor: getCamerasSysId for NOT inited\n"));
        mCamerasNumber = 9;
        for (int i = 0; i < mCamerasNumber; i++ ) {
            std::stringstream ss;
            ss << "eTopVi_" << (i + 1);
            string dev = ss.str();
            camDesc.push_back(dev);
        }
    }
    return mCamerasNumber;
}

int TopViDeviceDescriptor::replyCallback(TopViGrillCommand *cmd){
  SYNC_PRINT(("TopViDeviceDescriptor: replyCallback  return answer for DEVICE: [%d], %s\n", cmd->reply->replyResult, cmd->reply->replyStr));
  int result = false;

  string reply(cmd->reply->replyStr);

  if (cmd->reply->replyResult == GRILL_ERROR) {
      SYNC_PRINT(("TopViDeviceDescriptor: ER answer, do nothing\n"));
      return result;
  }

  if (cmd->reply->cmdName == TPV_GRAB) {
      SYNC_PRINT(("TopViDeviceDescriptor replyCallback: unknown interface for TPV_GRAB\n"));
      SYNC_PRINT(("TopViDeviceDescriptor replyCallback: may be good place for TPV_GRAB 0\n"));
      return false;
  }

  size_t pos = 0;
  string answer = "";
  int num = 0;
  do {
    pos = reply.find_first_of(",");
    answer = reply.substr(0, pos);
    if (answer != "") {
        //SYNC_PRINT(("[%d] next answer: %s\n", num, answer.c_str()));
        string id = answer.substr(0, 1);
        TopViCameraDescriptor *camera = mCameras[QString(id.c_str()).toInt() - 1];
        if (camera) camera->replyCallback(cmd->reply->cmdName, answer);
        num++;
    }
    reply = reply.substr(pos + 1);
    //SYNC_PRINT(("[%d] rest of reply: %s\n", num, reply.c_str()));
  } while (pos != std::string::npos);

  return result;
}

void TopViDeviceDescriptor::executeCommand(TopViCmdType cmdType, TopViCmdName cmdName, int camId, string value, string add_value, TopViCaptureInterface *parent)
{
    SYNC_PRINT(("TopViDeviceDescriptor::executeCommand() %s:%s called for camera %d\n", tpvCmdType(cmdType), tpvCmdName(cmdName), camId));
    //make grill command
    protectGrillRequest.lock();
    grillInterface.addCommand(parent, cmdType, cmdName, camId, value, add_value);
    protectGrillRequest.unlock();
}

void TopViDeviceDescriptor::grab(TopViCaptureInterface *parent)
{
    int camId = parent->getCamSysId();
    string sAuto = (parent->fAuto) ? "auto" : "";
    SYNC_PRINT(("TopViDeviceDescriptor::grab() called for camera %d\n", camId));
    executeCommand(TPV_GET, TPV_GRAB, camId, "", sAuto, parent);
}

void TopViDeviceDescriptor::grabAll(TopViCaptureInterface *parent)
{
    SYNC_PRINT(("TopViDeviceDescriptor::grabAll() called\n"));
    string sAuto = (parent->fAuto) ? "auto" : "";
#if 0
    for (int i = 0; i < mCamerasNumber; i++) {
        executeCommand(TPV_GET, TPV_GRAB, i, "", sAuto, parent);
    }
#else
    executeCommand(TPV_GET, TPV_GRAB, 0, "", sAuto, parent);
#endif
}

void TopViDeviceDescriptor::getStatus(TopViCaptureInterface *parent) {
    int camId =  parent->getCamSysId();
    SYNC_PRINT(("TopViDeviceDescriptor::getStatus() called for camera %d\n", camId));
    executeCommand(TPV_GET, TPV_STATUS, camId, "", "", NULL);
}

void TopViDeviceDescriptor::getExposure(TopViCaptureInterface *parent) {
    int camId = parent->getCamSysId();
    SYNC_PRINT(("TopViDeviceDescriptor::setExposure() called for camera %d\n", camId));
    executeCommand(TPV_SET, TPV_EXPOSURE, camId, "", "", NULL);
}

void TopViDeviceDescriptor::setExposure(TopViCaptureInterface *parent, double value) {
    int camId = parent->getCamSysId();
    SYNC_PRINT(("TopViDeviceDescriptor::setExposure() called for camera %d\n", camId));
    executeCommand(TPV_SET, TPV_EXPOSURE, camId, "", to_string(value), NULL);
}

void TopViDeviceDescriptor::setGain(TopViCaptureInterface *parent, enum TopViGain gainType, double value) {
    int camId = parent->getCamSysId();
    SYNC_PRINT(("TopViDeviceDescriptor::setGain() called for camera %d with parameter %s\n", camId, tpvGetColorGain(gainType)));
    executeCommand(TPV_SET, TPV_GAIN, camId, tpvGetColorGain(gainType), to_string(value), NULL);
//    executeCommand(TPV_SET, TPV_GAIN, camId, tpvGetColorGain(gainType), to_string(tpvGetGainCode(value)), NULL);
}

void TopViDeviceDescriptor::setCommonExposure(TopViCaptureInterface *parent, double value) {
    CORE_UNUSED(parent);
    SYNC_PRINT(("TopViDeviceDescriptor::setExposure() called for all cameras\n"));
    executeCommand(TPV_SET, TPV_EXPOSURE, 0, "", to_string(value), NULL);
}

void TopViDeviceDescriptor::setCommonGain(TopViCaptureInterface *parent, double value) {
    CORE_UNUSED(parent);
    SYNC_PRINT(("TopViDeviceDescriptor::setGain() called for all cameras with parameter %.2lf\n", value));
    executeCommand(TPV_SET, TPV_GAIN, 0, tpvGetColorGain(TPV_GLOBAL), to_string(value), NULL);
//    executeCommand(TPV_SET, TPV_GAIN, 0, tpvGetColorGain(TPV_GLOBAL), to_string(tpvGetGainCode(value)), NULL);
}

