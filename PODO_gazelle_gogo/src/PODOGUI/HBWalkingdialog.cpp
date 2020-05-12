#include "HBWalkingdialog.h"
#include "ui_HBWalkingdialog.h"

#include "BasicFiles/PODOALDialog.h"

enum HBWalking_COMMAND
{
    HBWalking_NO_ACT = 100,
    HBWalking_DATA_SAVE,
    HBWalking_TORQUE_TEST,
    HBWalking_ANKLE_TEST,
    HBWalking_STOP,
    HBWalking_WALK_TEST,
    HBWalking_SYSID_START,
    HBWalking_SYSID_STEP_INPUT_START,
    HBWalking_SYSID_STOP_SAVE,
    HBWalking_CONTROL_ON,
    HBWalking_OL_TORQUE_TUNING,
    HBWalking_ZERO_GAIN,
    HBWalking_INV_DYN_CONTROL,
    HBWalking_DYN_STANDING,
    HBWalking_POS_STANDING,
    HBWalking_DYN_WALKING,
    HBWalking_DYN_WALKING2,
    HBWalking_JUMP,
    HBWalking_PrevWalk,
    HBWalking_GetComHeight,
    HBWalking_Test,
    HBWalking_JoyStick_Walk_Stop,
    HBWalking_Ready_To_Walk,
    HBWalking_SingleLog,
    HBWalking_ROSWalk
};

enum WALKREADYCOMMAND
{
    WALKREADY_NO_ACT = 100,
    WALKREADY_GO_WALKREADYPOS,
    WALKREADY_GO_HOMEPOS,
    WALKREADY_WHEELCMD,
    WALKREADY_INFINITY,
    WALKREADY_DUMMY1,
    WALKREADY_DUMMY2,
    WALKREADY_DUMMY3,
    WALKREADY_DUMMY4,
    WALKREADY_DUMMY5,
    WALKREADY_SAVE
};

HBWalkingDialog::HBWalkingDialog(QWidget *parent, LANDialog *_lanData) :
    QDialog(parent),lanData(_lanData),
    ui(new Ui::HBWalkingDialog)
{
    ui->setupUi(this);
    
    displayTimer = new QTimer(this);
    connect(displayTimer, SIGNAL(timeout()), this, SLOT(DisplayUpdate()));
    displayTimer->start(50);//50

    AlnumHBWalking = PODOALDialog::GetALNumFromFileName("HBWalking");
    AlnumWalkReady = PODOALDialog::GetALNumFromFileName("WalkReady");
}

HBWalkingDialog::~HBWalkingDialog()
{
    delete ui;
}

void HBWalkingDialog::DisplayUpdate()
{
    if(PODO_DATA.UserM2G.ROSWalk_state == 1)
    {
        ui->LE_ROS_STATUS->setText(QString().sprintf("Start"));
    }
    else if(PODO_DATA.UserM2G.ROSflag == true)
    {
        ui->LE_ROS_STATUS->setText(QString().sprintf("Connected"));
    }else
    {
        ui->LE_ROS_STATUS->setText(QString().sprintf("Disconnected"));
    }
}

void HBWalkingDialog::on_BT_WALK_TEST_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = HBWalking_PrevWalk;//HBWalking_WALK_TEST;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = ui->LE_NO_OF_STEP->text().toInt();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->LE_STEP_TIME->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = ui->LE_STEP_STRIDE->text().toDouble();
    cmd.COMMAND_TARGET = AlnumHBWalking;
    pLAN->SendCommand(cmd);

}

void HBWalkingDialog::on_BT_WALK_READY_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = WALKREADY_GO_WALKREADYPOS;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 0;
    cmd.COMMAND_TARGET = AlnumWalkReady;
    pLAN->SendCommand(cmd);
}

void HBWalkingDialog::on_BT_DATA_SAVE_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = HBWalking_DATA_SAVE;
    cmd.COMMAND_TARGET = AlnumHBWalking;
    pLAN->SendCommand(cmd);
}

void HBWalkingDialog::on_BT_TEST_STOP_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = HBWalking_STOP;
    cmd.COMMAND_TARGET = AlnumHBWalking;
    pLAN->SendCommand(cmd);
}

void HBWalkingDialog::on_BT_SYSID_INPUT_START_clicked()
{
//    USER_COMMAND cmd;
//    cmd.COMMAND_DATA.USER_COMMAND = HBWalking_SYSID_START;
//    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->LE_SYSID_INPUT_FREQ->text().toDouble();
//    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = ui->LE_SYSID_INPUT_AMP->text().toDouble();
//    cmd.COMMAND_TARGET = AlnumHBWalking;
//    pLAN->SendCommand(cmd);
}

void HBWalkingDialog::on_BT_SYSID_STOP_SAVE_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = HBWalking_SYSID_STOP_SAVE;
    cmd.COMMAND_TARGET = AlnumHBWalking;
    pLAN->SendCommand(cmd);
}

void HBWalkingDialog::on_BT_SYSID_INPUT_START_2_clicked()
{
//    USER_COMMAND cmd;
//    cmd.COMMAND_DATA.USER_COMMAND = HBWalking_SYSID_STEP_INPUT_START;
//    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->LE_SYSID_INPUT_FREQ->text().toDouble();
//    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = ui->LE_SYSID_INPUT_AMP->text().toDouble();
//    cmd.COMMAND_TARGET = AlnumHBWalking;
//    pLAN->SendCommand(cmd);
}

void HBWalkingDialog::on_BT_WALK_TEST_2_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = 222;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->LE_IMPACT_TIME->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = ui->LE_IMPACT_FORCE->text().toDouble();
    cmd.COMMAND_TARGET = MAX_AL-1;
    pLAN->SendCommand(cmd);
}

void HBWalkingDialog::on_BT_CONTROL_ON_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = HBWalking_CONTROL_ON;
    cmd.COMMAND_TARGET = AlnumHBWalking;
    pLAN->SendCommand(cmd);
}

void HBWalkingDialog::on_BT_OLTORQUE_GO_clicked()
{
//    USER_COMMAND cmd;
//    cmd.COMMAND_DATA.USER_COMMAND = HBWalking_OL_TORQUE_TUNING;
//    cmd.COMMAND_DATA.USER_PARA_INT[0] = ui->LE_OLTORQUE_MA->text().toInt();
//    cmd.COMMAND_TARGET = AlnumHBWalking;
//    pLAN->SendCommand(cmd);
}

void HBWalkingDialog::on_BT_ONE_LEG_READY_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = WALKREADY_GO_WALKREADYPOS;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 1;
    cmd.COMMAND_TARGET = AlnumWalkReady;
    pLAN->SendCommand(cmd);
}

void HBWalkingDialog::on_BT_ONE_LEG_READY_2_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = WALKREADY_GO_WALKREADYPOS;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 3;
    cmd.COMMAND_TARGET = AlnumWalkReady;
    pLAN->SendCommand(cmd);
}

void HBWalkingDialog::on_BT_TORQUE_TEST_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = HBWalking_TORQUE_TEST;
    cmd.COMMAND_TARGET = AlnumHBWalking;
    pLAN->SendCommand(cmd);
}

void HBWalkingDialog::on_BT_ZERO_GAIN_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = HBWalking_ZERO_GAIN;
    cmd.COMMAND_TARGET = AlnumHBWalking;
    pLAN->SendCommand(cmd);
}

void HBWalkingDialog::on_BT_INV_DYNAMICS_CON_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = HBWalking_INV_DYN_CONTROL;
    cmd.COMMAND_TARGET = AlnumHBWalking;
    pLAN->SendCommand(cmd);
}

void HBWalkingDialog::on_BT_DYN_STANDING_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = HBWalking_DYN_STANDING;
    cmd.COMMAND_TARGET = AlnumHBWalking;
    pLAN->SendCommand(cmd);
    std::cout<<"GUI : Dyn standing clicked"<<std::endl;
}

void HBWalkingDialog::on_BT_POS_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = HBWalking_POS_STANDING;
    cmd.COMMAND_TARGET = AlnumHBWalking;
    pLAN->SendCommand(cmd);
}

void HBWalkingDialog::on_BT_DYN_WALKING_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = HBWalking_DYN_WALKING2;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = ui->LE_NO_OF_STEP->text().toInt();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->LE_STEP_TIME->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = ui->LE_STEP_STRIDE->text().toDouble();
    cmd.COMMAND_TARGET = AlnumHBWalking;
    pLAN->SendCommand(cmd);
}

void HBWalkingDialog::on_BT_JUMP_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = HBWalking_JUMP;
    cmd.COMMAND_TARGET = AlnumHBWalking;
    pLAN->SendCommand(cmd);
}

void HBWalkingDialog::on_BT_JUMP_READY_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = WALKREADY_GO_WALKREADYPOS;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = 2;
    cmd.COMMAND_TARGET = AlnumWalkReady;
    pLAN->SendCommand(cmd);
}

void HBWalkingDialog::on_BT_COM_HEIGHT_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = HBWalking_GetComHeight;
    cmd.COMMAND_TARGET = AlnumHBWalking;
    pLAN->SendCommand(cmd);
}

void HBWalkingDialog::on_BT_LOGGING_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = HBWalking_Test;
    cmd.COMMAND_TARGET = AlnumHBWalking;
    pLAN->SendCommand(cmd);
}



void HBWalkingDialog::on_BT_READY_TO_WALK_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = HBWalking_Ready_To_Walk;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = ui->LE_NO_OF_STEP->text().toInt();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->LE_STEP_TIME->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = ui->LE_STEP_STRIDE->text().toDouble();
    cmd.COMMAND_TARGET = AlnumHBWalking;
    pLAN->SendCommand(cmd);
}

void HBWalkingDialog::on_BT_WALK_SINGLELOG_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = HBWalking_SingleLog;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = ui->LE_NO_OF_STEP_2->text().toInt();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->LE_STEP_TIME_2->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = ui->LE_STEP_STRIDE_2->text().toDouble();
    cmd.COMMAND_TARGET = AlnumHBWalking;
    pLAN->SendCommand(cmd);
}

void HBWalkingDialog::on_BT_WALK_ROS_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = AlnumHBWalking;
    if(ui->LE_ROS_STATUS->text() == "Connected")
    {
        cmd.COMMAND_DATA.USER_COMMAND = HBWalking_ROSWalk;
        cmd.COMMAND_DATA.USER_PARA_INT[0] = 10;
        cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = 0.8;
        cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = 0;
        cmd.COMMAND_DATA.USER_PARA_INT[10] = 1;
    }else if(ui->LE_ROS_STATUS->text() == "Start")
    {
        cmd.COMMAND_DATA.USER_COMMAND = HBWalking_ROSWalk;
        cmd.COMMAND_DATA.USER_PARA_INT[10] = 0;
    }
    printf("before send\n");
    pLAN->SendCommand(cmd);
    printf("after send\n");
}

void HBWalkingDialog::on_BT_DATA_SAVE_2_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_TARGET = AlnumWalkReady;
    cmd.COMMAND_DATA.USER_COMMAND = WALKREADY_SAVE;
    pLAN->SendCommand(cmd);
}
