#include "RBIMUSensor.h"

RBIMUSensor::RBIMUSensor()
{
    delX = delY = delZ = ROLL = PITCH = YAW = 0.0;

    FOGQ0 = 1.0;
    FOGQ1 = 0.0;
    FOGQ2 = 0.0;
    FOGQ3 = 0.0;

//    if(rt_task_create(&ReceiveThreadHandler, "IMU_READ_TASK", 0, 94, 0) == 0){
//        cpu_set_t aCPU;
//        CPU_ZERO(&aCPU);
//        CPU_SET(1, &aCPU);
//        if(rt_task_set_affinity(&ReceiveThreadHandler, &aCPU) != 0){
//            FILE_LOG(logWARNING) << "RBIMU: Read thread set affinity CPU failed..";
//        }
//        if(rt_task_start(&ReceiveThreadHandler, &IMU_ReadThread, this) == 0){

//        }else{
//            FILE_LOG(logERROR) << "RBIMU: Read thread Creation Error";
//            return;
//        }
//    }else{
//        FILE_LOG(logERROR) << "RBIMU: Read thread Creation Error";
//        return;
//    }

}


void RBIMUSensor::RBIMU_AddCANMailBox(){
    canHandler->RBCAN_AddMailBox(ID_RCV_DATA1);
    canHandler->RBCAN_AddMailBox(ID_RCV_DATA2);
    canHandler->RBCAN_AddMailBox(ID_RCV_INFO);
    canHandler->RBCAN_AddMailBox(ID_RCV_PARA);
    //canHandler->RBCAN_AddMailBox(ID_RCV_STAT);
}

void RBIMUSensor::RBBoard_GetDBData(DB_IMU db){
    BOARD_ID        = db.BOARD_ID;
    BOARD_NAME      = db.BOARD_NAME;
    SENSOR_ID       = db.SENSOR_ID;
    CAN_CHANNEL     = db.CAN_CHANNEL;
    SENSOR_TYPE     = db.SENSOR_TYPE;
    ID_RCV_DATA1    = db.ID_RCV_DATA1;
    ID_RCV_DATA2    = db.ID_RCV_DATA2;
    ID_RCV_STAT     = db.ID_RCV_STAT;
    ID_RCV_INFO     = db.ID_RCV_INFO;
    ID_RCV_PARA     = db.ID_RCV_PARA;
}

int RBIMUSensor::RBBoard_CANCheck(int _canr){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = BOARD_ID;		// board no.
    mb.data[1] = 0x01;		// command
    mb.data[2] = _canr;	// CAN communication rate(msec)
    mb.dlc = 3;
    mb.id = COMMAND_CANID;

    if( canHandler->RBCAN_WriteData(mb) == true ){
        usleep(15*1000);
        mb.channel = CAN_CHANNEL;
        mb.id = ID_RCV_INFO;
        canHandler->RBCAN_ReadData(&mb);
        if(mb.status != RBCAN_NODATA){
            std::cout << ">>> RMIMU: Board(" << BOARD_ID << ": " << BOARD_NAME.toStdString().data() << ") is \033[32minitialized.\033[0m[ch " << CAN_CHANNEL << "]\n";
            ConnectionStatus = true;
            mb.status = RBCAN_NODATA;
            return true;
        }else{
            std::cout << ">>> RMIMU: Board(" << BOARD_ID << ": " << BOARD_NAME.toStdString().data() << ") is \033[31mfailed \033[0mto initialize.[ch " << CAN_CHANNEL << "]\n";
            ConnectionStatus = false;
            return false;
        }
    }
    else return false;
}

int RBIMUSensor::RBBoard_RequestStatus(void){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = BOARD_ID;		// board no.
    mb.data[1] = 0x02;		// command
    mb.dlc = 2;
    mb.id = COMMAND_CANID;

    return canHandler->RBCAN_WriteData(mb);
}

int RBIMUSensor::RBBoard_LoadDefaultValue(void){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = BOARD_ID;			// board no.
    mb.data[1] = 0xFA;			// command
    mb.dlc = 2;
    mb.id = COMMAND_CANID;

    return canHandler->RBCAN_WriteData(mb);
}

int RBIMUSensor::RBIMU_RequestNulling(void){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = BOARD_ID;			// board no.
    mb.data[1] = 0x81;			// command
    mb.dlc = 2;
    mb.id = COMMAND_CANID;

    return canHandler->RBCAN_WriteData(mb);
}

int RBIMUSensor::RBIMU_RequestCalibration(void){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = BOARD_ID;			// board no.
    mb.data[1] = 0x82;			// command
    mb.dlc = 2;
    mb.id = COMMAND_CANID;

    return canHandler->RBCAN_WriteData(mb);
}

int RBIMUSensor::RBIMU_RequestParameter(unsigned char _prf){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = BOARD_ID;			// board no.
    mb.data[1] = 0x24;			// command
    mb.data[2] = _prf;		    // parameter request
    // _prf = 0x01 : ACC_X_GAIN  ACC_Y_GAIN  ACC_Z_GAIN
    // _prf = 0x02 : ACC_X_BIAS  ACC_Y_BIAS  ACC_Z_BIAS
    mb.dlc = 3;
    mb.id = COMMAND_CANID;

    return canHandler->RBCAN_WriteData(mb);
}

int RBIMUSensor::RBIMU_SetBoardNumber(int _newbno){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = BOARD_ID;						// board no.
    mb.data[1] = 0xA8;						// command
    mb.data[2] = _newbno;					// new board number
    mb.dlc = 3;
    mb.id = COMMAND_CANID;

    return canHandler->RBCAN_WriteData(mb);
}

//int RBIMUSensor::RBIMU_RequestData(int _type){
//    RBCAN_MB mb;
//    mb.channel = CAN_CHANNEL;
//    mb.data[0] = SENSOR_ID;				// Sensor board no. if _sbno = 0xFF : all sensor boards ??
//    mb.data[1] = _type;              // Request Angle and Rate data
//    mb.data[2] = 0x01;              // Extra value to indicate an IMU sensor, otherwise this CAN message would be
//                                    // the same as one of the FT request data messages.
//    mb.dlc = 3;
//    mb.id = 0x01;//SENSOR_REQUEST_CANID;

//    return canHandler->RBCAN_WriteDataDirectly(mb);
//}


int RBIMUSensor::RBIMU_RequestData(int _type){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 0x32;//SENSOR_ID;				// Sensor board no. if _sbno = 0xFF : all sensor boards ??
    mb.data[1] = _type;              // Request Angle and Rate data
    mb.data[2] = 0x01;              // Extra value to indicate an IMU sensor, otherwise this CAN message would be
                                    // the same as one of the FT request data messages.
    mb.dlc = 3;
    mb.id = 0x01;//SENSOR_REQUEST_CANID;

    return canHandler->RBCAN_WriteDataDirectly(mb);
}

int RBIMUSensor::RBIMU_ReadData(void){
    RBCAN_MB mb;
    RBCAN_MB mb2;

    double Complementary_ROLL, Complementary_PITCH;

    // Read IMU Data1
    mb.channel = CAN_CHANNEL;
    mb.id = ID_RCV_DATA1;
    canHandler->RBCAN_ReadData(&mb);
    if(mb.status != RBCAN_NODATA)
    {
//        Complementary_ROLL = (double)((short)((mb.data[1]<<8)|mb.data[0]))/100.0f;// + ROLL_OFFSET;
//        Complementary_PITCH = (double)((short)((mb.data[3]<<8)|mb.data[2]))/100.0f;// + -1.0;//PITCH_OFFSET;
        ROLL_VEL = (double)((short)((mb.data[5]<<8)|mb.data[4]))/100.0f;
        PITCH_VEL = (double)((short)((mb.data[7]<<8)|mb.data[6]))/100.0f;


        mb.status = RBCAN_NODATA;
    }

     //Read IMU Data2
    mb2.channel = CAN_CHANNEL;
    mb2.id = ID_RCV_DATA2;
    canHandler->RBCAN_ReadData(&mb2);
    if(mb2.status != RBCAN_NODATA){

        ACC_X = (double)((short)((mb2.data[1]<<8)|mb2.data[0]))/100.0f;//-0.4;//ROLL_OFFSET;//FOG Edit
        ACC_Y = (double)((short)((mb2.data[3]<<8)|mb2.data[2]))/100.0f;//-0.2;//PITCH_OFFSET;//FOG Edit
        ACC_Z = (double)((short)((mb2.data[5]<<8)|mb2.data[4]))/100.0f;
        YAW_VEL = (double)((short)((mb2.data[7]<<8)|mb2.data[6]))/100.0f;
        mb2.status = RBCAN_NODATA;
    }

    float R2D = 57.2957802f;
    float D2R = 0.0174533f;

    double velX = ROLL_VEL*D2R;
    double velY = PITCH_VEL*D2R;
    double velZ = YAW_VEL*D2R;

    delX = velX + velY*sin(ROLL*D2R)*tan(PITCH*D2R) + velZ*cos(ROLL*D2R)*tan(PITCH*D2R);
    delY = velY*cos(ROLL*D2R) - velZ*sin(ROLL*D2R);
    delZ = velY*sin(ROLL*D2R)/(cos(PITCH*D2R)) + velZ*cos(ROLL*D2R)/(cos(PITCH*D2R)); // it does not return

//    tempX += delX*R2D*0.005;
//    tempY += delY*R2D*0.005;
//    tempZ += delZ*R2D*0.005;

//    if(tempY >= 30.0)   tempY = 30.0;
//    if(tempY <= -30.0)  tempY = -30.0;
//    if(tempX >= 30.0)   tempX = 30.0;
//    if(tempX <= -30.0)  tempX = -30.0;

//    float delX = velX + velY*sin((fog->FOGRoll))*tan(fog->FOGPitch) + velZ*cos(fog->FOGRoll)*tan(fog->FOGPitch);
//    float delY = velY*cos(fog->FOGRoll) - velZ*sin(fog->FOGRoll);
//    float delZ = velY*sin(fog->FOGRoll)/(cos(fog->FOGPitch)) + velZ*cos(fog->FOGRoll)/(cos(fog->FOGPitch));

    double l = sqrt(velX*velX + velY*velY + velZ*velZ);
    double p[4] = {cos(0.00125*l/2.0),velX*sin(0.00125*l/2.0)/l,velY*sin(0.00125*l/2.0)/l,velZ*sin(0.00125*l/2.0)/l};

    double Q[4];
//    for(int i=0;i<4;i++){
//        if(isnan(p[i])){
//            p[0] = 1; p[1] = 0; p[2] = 0; p[3] = 0;
//           // std::cout<<"p["<<i<<"] - isnan"<<std::endl;
//            break;
//        }
//    }
//    if(isnan(FOGQ0)||isnan(FOGQ1)||isnan(FOGQ2)||isnan(FOGQ3)){
//        FOGQ0 = 1; FOGQ1 = 0; FOGQ2 = 0; FOGQ3 = 0;
//        std::cout<<"Q - isnan"<<std::endl;
//    }

    Q[0] = FOGQ0*p[0] - FOGQ1*p[1] - FOGQ2*p[2] - FOGQ3*p[3];
    Q[1] = FOGQ0*p[1] + FOGQ1*p[0] + FOGQ2*p[3] - FOGQ3*p[2];
    Q[2] = FOGQ0*p[2] - FOGQ1*p[3] + FOGQ2*p[0] + FOGQ3*p[1];
    Q[3] = FOGQ0*p[3] + FOGQ1*p[2] - FOGQ2*p[1] + FOGQ3*p[0];
//    Q[0] = FOGQ0*p[0] - FOGQ1*p[1] - FOGQ2*p[2] - FOGQ3*p[3];
//    Q[1] = FOGQ0*p[1] + FOGQ1*p[0] - FOGQ2*p[3] + FOGQ3*p[2];
//    Q[2] = FOGQ0*p[2] + FOGQ1*p[3] + FOGQ2*p[0] - FOGQ3*p[1];
//    Q[3] = FOGQ0*p[3] - FOGQ1*p[2] + FOGQ2*p[1] + FOGQ3*p[0];

//    Q[0] = Quat_IMU[0]*p[0] - Quat_IMU[1]*p[1] - Quat_IMU[2]*p[2] - Quat_IMU[3]*p[3];
//        Q[1] = Quat_IMU[0]*p[1] + Quat_IMU[1]*p[0] - Quat_IMU[2]*p[3] + Quat_IMU[3]*p[2];
//        Q[2] = Quat_IMU[0]*p[2] + Quat_IMU[1]*p[3] + Quat_IMU[2]*p[0] - Quat_IMU[3]*p[1];
//        Q[3] = Quat_IMU[0]*p[3] - Quat_IMU[1]*p[2] + Quat_IMU[2]*p[1] + Quat_IMU[3]*p[0];

    double ll = sqrt(Q[0]*Q[0] + Q[1]*Q[1] + Q[2]*Q[2] + Q[3]*Q[3]);

    FOGQ0 = Q[0]/ll;
    FOGQ1 = Q[1]/ll;
    FOGQ2 = Q[2]/ll;
    FOGQ3 = Q[3]/ll;

    // Euler 321 sequence -> yaw pitch roll
    YAW = R2D*atan2(2.0*(Q[1]*Q[2] + Q[0]*Q[3]),Q[0]*Q[0] + Q[1]*Q[1] - Q[2]*Q[2] - Q[3]*Q[3]);//Q[1];//delX*0.002;//0.001;
    PITCH = R2D*asin(-2.0*(Q[1]*Q[3] - Q[0]*Q[2]));//Q[2];//delY*0.002;//0.001;
    ROLL = R2D*atan2(2.0*(Q[2]*Q[3] + Q[0]*Q[1]) , Q[0]*Q[0] - Q[1]*Q[1] - Q[2]*Q[2] + Q[3]*Q[3]);//Q[3];//delZ*0.002;//0.001;

//    double R11 = Q[0]*Q[0] + Q[1]*Q[1] + Q[2]*Q[2] + Q[3]*Q[3];
//    double R12 = 2*(Q[1]*Q[2] - Q[0]*Q[3]);
//    double R13 = 2*(Q[0]*Q[2] + Q[1]*Q[3]);

//    static int cnt = 0;

//    if(cnt == 400){
//        std::cout<<"R11: "<<R11<<" R12: "<<R12<<" R13: "<<R13<<std::endl;
//        cnt = 0;
//    }
//    cnt++;

//    YAW += R2D*delZ*0.00125;//0.001;
//    PITCH += R2D*delY*0.00125;//0.001;
//    ROLL += R2D*delX*0.00125;//0.001;

//    YAW = 0;//Complementary_ROLL;
//    PITCH = Complementary_PITCH;//0.001;
//    ROLL = Complementary_ROLL;//0.001;



    return true;
}

//void RBIMUSensor::IMU_ReadThread(void *_arg)
//{
//    isWorking = 1;

//    rt_task_set_periodic(NULL, TM_NOW, 2*1000000);

//    while(isWorking)
//    {
//        rt_task_wait_period(NULL);




//    }
//}
