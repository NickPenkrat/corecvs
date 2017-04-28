#include <iostream>

#include "topViDeviceDescriptor.h"
#include "topViCapture.h"

#include <sys/types.h>

#ifdef Q_OS_UNIX
  #include <sys/socket.h>
  #include <netdb.h>

  #define READ(s, buf, len)     read(s, buf, len)
  #define WRITE(s, buf, len)	write(s, buf, len)
#else
  #include <winsock2.h>
  #include <ws2tcpip.h>

  #define READ(s, buf, len)	recv(s, (char*)(buf), len, 0)
  #define WRITE(s, buf, len)	send(s, (char*)(buf), len, 0)
  typedef int socklen_t;
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
            return 1;
    }

    if (LOBYTE(wsaData.wVersion) != 2 || HIBYTE(wsaData.wVersion) != 2){
            SYNC_PRINT(("Could not find a usable version of Winsock.dll\n"));
            WSACleanup();
            return 1;
    }
#endif

    memset(&hints, 0, sizeof(struct addrinfo));
    hints.ai_family = AF_UNSPEC;     /* Allow IPv4 or IPv6 */
    hints.ai_socktype = SOCK_STREAM; /* TCP socket */
    hints.ai_flags = 0;
    hints.ai_protocol = 0;           /* Any protocol */

xxx:
    string addr = "localhost";
    int port = 8002;

    retval = getaddrinfo(addr.c_str(), to_string(port).c_str(), &hints, &result);
    if (retval != 0) {
       fprintf(stderr, "Incorrect address %s\n", gai_strerror(retval));
       sleep(5);
       goto xxx;
    }

    for (rp = result; rp != NULL; rp = rp->ai_next) {
       fd = socket(rp->ai_family, rp->ai_socktype, rp->ai_protocol);
       if (fd == -1) continue;
       if (::connect(fd, rp->ai_addr, rp->ai_addrlen) != -1) break;
       close(fd);
    }

    if (rp == NULL) {
        fprintf(stderr, "Could not connect to %s. Waiting 5s  for power on\n", (addr + ":" + to_string(port)).c_str());
        freeaddrinfo(result);
        sleep(5);
        goto xxx;
    }

    freeaddrinfo(result);

    SYNC_PRINT(("TopViDeviceDescriptor::CmdSpinThread(): new command thread running\n"));

    while (device->cmdSpinRunning.tryLock()) {

        ssize_t	length = 0, retval = 0;
        char s[0xFFF +1];

        device->protectGrillRequest.lock();

        QMutableListIterator<TopViGrillCommand*> it(device->grillInterface.commandList);
        while (it.hasNext()) {
            TopViGrillCommand *cmd = it.next();

            if (cmd->state != TPV_UNKNOWN) continue;

            SYNC_PRINT(("TopViGrillInterface::CmdSpinThread(): SEND cmdId = %d, cmdType = %s\n", cmd->cmdId, cmdTypeName(cmd->cmdType).c_str()));

            length = cmd->request->requestStr.length();
            try{
                retval = WRITE(fd, cmd->request->requestStr.c_str(), length);
            }
            catch(...){
                SYNC_PRINT(("Broken pipe\n"));
                close(fd);
                goto xxx;
            }

            if (retval != length){
                if (retval < 0) SYNC_PRINT(("TopViGrillInterface::CmdSpinThread(): write error: %s\n", strerror(errno)));
                break;
            }

            device->protectGrillReply.lock();
            cmd->state = TPV_QUERY;
            device->protectGrillReply.unlock();

            cmd->state = cmdNeedReplay(cmd->cmdType);

            if (cmd->state == TPV_WAIT_REPLY) {
                int result = false;
#if 1
                string serverReply = generateTestReply(cmd->cmdType);
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

                device->grillInterface.replyCallback(serverReply);
                device->protectGrillReply.lock();
                cmd->state = TPV_REPLY;
                device->protectGrillReply.unlock();
                SYNC_PRINT(("TopViGrillInterface::CmdSpinThread(): before callback cmdId = %d, cmdStatus = %s\n", cmd->cmdId, cmdStatusName(cmd->state).c_str()));
                if (cmd->parent) {
                    result = cmd->parent->replyCallback(cmd->reply->replyStr);
                }
                else {
                    device->replyCallback(cmd);
                    result = true;
                }
                SYNC_PRINT(("TopViGrillInterface::CmdSpinThread(): after callback cmdId = %d, cmdStatus = %s\n", cmd->cmdId, cmdStatusName(cmd->state).c_str()));
                if (result) {
                    device->protectGrillReply.lock();
                    cmd->state = TPV_OK;
                    device->protectGrillReply.unlock();
                    SYNC_PRINT(("TopViGrillInterface::CmdSpinThread(): should be OK cmdId = %d, cmdStatus = %s\n", cmd->cmdId, cmdStatusName(cmd->state).c_str()));
                }
                else {
                    device->protectGrillReply.lock();
                    cmd->state = TPV_OK;
                    device->protectGrillReply.unlock();
                    SYNC_PRINT(("TopViGrillInterface::CmdSpinThread(): command doesn't resolve cmdId = %d, cmdStatus = %s\n", cmd->cmdId, cmdStatusName(cmd->state).c_str()));
                }
             }
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
    close(fd);
    SYNC_PRINT(("TopViDeviceDescriptor::CmdSpinThread(): command thread finished\n"));
}


int TopViDeviceDescriptor::init(int _deviceId, TopViCaptureInterface *parent){
    SYNC_PRINT(("TopViDeviceDescriptor::init(): init called\n"));
    this->deviceId = _deviceId;
    if (inited.testAndSetAcquire(0,1)) {
        this->deviceId = deviceId;
        this->cmdSpinRunning.lock();
        this->shouldStopCmdSpinThread = false;
        this->cmdSpinRunning.unlock();
        cmdSpin.device = this;
        cmdSpin.start();
        this->executeCommand(TPV_INIT, 0);
    }
    //we assume that all objects with the type TopViCapture creation in the single thread
    this->executeCommand(TPV_STATUS, 0);

}

int TopViDeviceDescriptor::getCamerasSysId(vector<string> &camDesc)
{
    if (this->inited) {
        for (int i = 0; i < mCamerasNumber; i++) {
            TopViCameraDescriptor *camera = mCameras[i];
            camDesc.push_back(camera->getSysId());
        }
    }
    else {
        mCamerasNumber = 9;
        for (int i = 0; i < mCamerasNumber; i++ ) {
            std::stringstream ss;
            ss << "eTopVi_" << (i + 1);
            string dev = ss.str();
            TopViCameraDescriptor *camera = new TopViCameraDescriptor();
            mCameras.push_back(camera);
            camDesc.push_back(dev);
        }
    }
    return mCamerasNumber;
}

void TopViDeviceDescriptor::replyCallback(TopViGrillCommand *cmd){
  SYNC_PRINT(("TopViDeviceDescriptor: callback reply for command %s(%d): %s\n", cmdTypeName(cmd->cmdType).c_str(), cmd->cmdId, cmd->reply->replyStr.c_str()));
}

void TopViDeviceDescriptor::executeCommand(TopViCmd cmdType, int camId, int value, int add_value, TopViCaptureInterface *parent)
{
    SYNC_PRINT(("TopViDeviceDescriptor::executeCommand() %s called for camera %d\n", cmdTypeName(cmdType).c_str(), camId));
    //make grill command
    protectGrillRequest.lock();
    grillInterface.addCommand(parent, cmdType, camId, value, add_value);
    protectGrillRequest.unlock();
}

void TopViDeviceDescriptor::grab(TopViCaptureInterface *parent)
{
    int camId =  QString(parent->getDeviceSerial().c_str()).toInt();
    SYNC_PRINT(("TopViDeviceDescriptor::grab() called for camera %d\n", camId));
    executeCommand(TPV_GRAB, camId, 0, 0, parent);
}

void TopViDeviceDescriptor::grabAll(TopViCaptureInterface *parent)
{
    SYNC_PRINT(("TopViDeviceDescriptor::grabAll() called\n"));
    for (int i = 0; i < mCamerasNumber; i++) {
        executeCommand(TPV_GRAB, i, 0, 0, parent);
    }
}

