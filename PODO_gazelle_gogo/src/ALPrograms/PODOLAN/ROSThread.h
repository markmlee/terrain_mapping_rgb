#ifndef ROSTHREAD_H
#define ROSTHREAD_H

#include <sys/mman.h>
#include <fcntl.h>

#include <QByteArray>
#include <QTimer>

#include "PODOServer.h"

class ROSServer;
class RSTServer;


class ROSWorker : public QObject
{
    Q_OBJECT
public:
    ROSWorker();
    ~ROSWorker();
    ROSServer    *serverROS;
    RSTServer    *serverRST;

    P2R_status status;
    R2P_command command;
    P2R_result result;

    QTimer *sendTimer;
    void    SendtoROS();
    void    ReadfromROS();

signals:
    void ROS_Connected();
    void ROS_Disconnected();
    void RST_Connected();
    void RST_Disconnected();
    void ROS_CMD_READ(R2P_command _cmd);

private slots:
    void ROSConnected();
    void ROSDisconnected();
    void RSTConnected();
    void RSTDisconnected();
    void readCMD(char *_data);
    void sendSTATUS();
    void sendRESULT();

private:
    int ROSflag;
};

class ROSServer : public RBTCPServer
{
    Q_OBJECT
public:
    ROSServer();
    ~ROSServer();

    QByteArrays     dataReceived;
    char      *data;
    int     TXSize;
    int     RXSize;

signals:
    void ROS_UPDATE(char *_data);

protected slots:
    virtual void ReadData();
};

class RSTServer : public RBTCPServer
{
    Q_OBJECT
public:
    RSTServer();
    ~RSTServer();

    int  TXSize;
signals:
    void ROS_UPDATE(char *_data);

protected slots:
    virtual void ReadData();
};


#endif // ROSTHREAD_H
