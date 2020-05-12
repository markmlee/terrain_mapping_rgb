#include "SensorDialog.h"
#include "ui_SensorDialog.h"

SensorDialog::SensorDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::SensorDialog)
{
    ui->setupUi(this);

    connect(pLAN, SIGNAL(NewPODOData()), this, SLOT(UpdateSensors()));
}

SensorDialog::~SensorDialog()
{
    delete ui;
}


void SensorDialog::UpdateSensors(){
    QString str;

    // FT ======
    ui->LE_SENSOR_RAFT_MX->setText(str.sprintf("%.2f", PODO_DATA.CoreSEN.FT[0].Mx));
    ui->LE_SENSOR_RAFT_MY->setText(str.sprintf("%.2f", PODO_DATA.CoreSEN.FT[0].My));
    ui->LE_SENSOR_RAFT_MZ->setText(str.sprintf("%.2f", PODO_DATA.CoreSEN.FT[0].Mz));
    ui->LE_SENSOR_RAFT_FX->setText(str.sprintf("%.2f", PODO_DATA.CoreSEN.FT[0].Fx));
    ui->LE_SENSOR_RAFT_FY->setText(str.sprintf("%.2f", PODO_DATA.CoreSEN.FT[0].Fy));
    ui->LE_SENSOR_RAFT_FZ->setText(str.sprintf("%.2f", PODO_DATA.CoreSEN.FT[0].Fz));

    ui->LE_SENSOR_LAFT_MX->setText(str.sprintf("%.2f", PODO_DATA.CoreSEN.FT[1].Mx));
    ui->LE_SENSOR_LAFT_MY->setText(str.sprintf("%.2f", PODO_DATA.CoreSEN.FT[1].My));
    ui->LE_SENSOR_LAFT_MZ->setText(str.sprintf("%.2f", PODO_DATA.CoreSEN.FT[1].Mz));
    ui->LE_SENSOR_LAFT_FX->setText(str.sprintf("%.2f", PODO_DATA.CoreSEN.FT[1].Fx));
    ui->LE_SENSOR_LAFT_FY->setText(str.sprintf("%.2f", PODO_DATA.CoreSEN.FT[1].Fy));
    ui->LE_SENSOR_LAFT_FZ->setText(str.sprintf("%.2f", PODO_DATA.CoreSEN.FT[1].Fz));

    ui->LE_SENSOR_RWFT_MX->setText(str.sprintf("%.2f", PODO_DATA.CoreSEN.FT[2].Mx));
    ui->LE_SENSOR_RWFT_MY->setText(str.sprintf("%.2f", PODO_DATA.CoreSEN.FT[2].My));
    ui->LE_SENSOR_RWFT_MZ->setText(str.sprintf("%.2f", PODO_DATA.CoreSEN.FT[2].Mz));
    ui->LE_SENSOR_RWFT_FX->setText(str.sprintf("%.2f", PODO_DATA.CoreSEN.FT[2].Fx));
    ui->LE_SENSOR_RWFT_FY->setText(str.sprintf("%.2f", PODO_DATA.CoreSEN.FT[2].Fy));
    ui->LE_SENSOR_RWFT_FZ->setText(str.sprintf("%.2f", PODO_DATA.CoreSEN.FT[2].Fz));

    ui->LE_SENSOR_LWFT_MX->setText(str.sprintf("%.2f", PODO_DATA.CoreSEN.FT[3].Mx));
    ui->LE_SENSOR_LWFT_MY->setText(str.sprintf("%.2f", PODO_DATA.CoreSEN.FT[3].My));
    ui->LE_SENSOR_LWFT_MZ->setText(str.sprintf("%.2f", PODO_DATA.CoreSEN.FT[3].Mz));
    ui->LE_SENSOR_LWFT_FX->setText(str.sprintf("%.2f", PODO_DATA.CoreSEN.FT[3].Fx));
    ui->LE_SENSOR_LWFT_FY->setText(str.sprintf("%.2f", PODO_DATA.CoreSEN.FT[3].Fy));
    ui->LE_SENSOR_LWFT_FZ->setText(str.sprintf("%.2f", PODO_DATA.CoreSEN.FT[3].Fz));

    // IMU ======
    ui->LE_SENSOR_CIMU_ROLL->setText(str.sprintf("%.2f", PODO_DATA.CoreSEN.IMU[0].Roll));
    ui->LE_SENSOR_CIMU_PITCH->setText(str.sprintf("%.2f", PODO_DATA.CoreSEN.IMU[0].Pitch));
    ui->LE_SENSOR_CIMU_ROLL_VEL->setText(str.sprintf("%.2f", PODO_DATA.CoreSEN.IMU[0].RollVel));
    ui->LE_SENSOR_CIMU_PITCH_VEL->setText(str.sprintf("%.2f", PODO_DATA.CoreSEN.IMU[0].PitchVel));
    ui->LE_SENSOR_CIMU_ROLL_ACC->setText(str.sprintf("%.2f", PODO_DATA.CoreSEN.IMU[0].Roll_Acc));
    ui->LE_SENSOR_CIMU_PITCH_ACC->setText(str.sprintf("%.2f", PODO_DATA.CoreSEN.IMU[0].Pitch_Acc));

    // FOG ======
//    ui->LE_SENSOR_FOG_ROLL->setText(str.sprintf("%.2f", PODO_DATA.CoreSEN.FOG.Roll));
//    ui->LE_SENSOR_FOG_PITCH->setText(str.sprintf("%.2f", PODO_DATA.CoreSEN.FOG.Pitch));
//    ui->LE_SENSOR_FOG_YAW->setText(str.sprintf("%.2f", PODO_DATA.CoreSEN.FOG.Yaw));
    ui->LE_SENSOR_FOG_ROLL->setText(str.sprintf("%.2f", PODO_DATA.CoreSEN.IMU[0].Roll_Comp));
    ui->LE_SENSOR_FOG_PITCH->setText(str.sprintf("%.2f", PODO_DATA.CoreSEN.IMU[0].Pitch_Comp));
    ui->LE_SENSOR_FOG_YAW->setText(str.sprintf("%.2f", PODO_DATA.CoreSEN.FOG.Yaw));

//    double roll = asin(-PODO_DATA.CoreSEN.IMU[0].Roll_Comp/9.8)*180/3.14;
//    double pitch = asin(PODO_DATA.CoreSEN.IMU[0].Pitch_Comp/9.8)*180/3.14;
//    ui->LE_SENSOR_FOG_ROLL->setText(str.sprintf("%.2f", roll));
//    ui->LE_SENSOR_FOG_PITCH->setText(str.sprintf("%.2f",pitch));
//    ui->LE_SENSOR_FOG_YAW->setText(str.sprintf("%.2f", PODO_DATA.CoreSEN.FOG.Yaw));

    // OF Flow =====
    ui->LE_OPT_Xmea->setText(str.sprintf("%.2f", PODO_DATA.CoreSEN.OF.Xmea));
    ui->LE_OPT_Ymea->setText(str.sprintf("%.2f", PODO_DATA.CoreSEN.OF.Ymea));
    ui->LE_OPT_thmea->setText(str.sprintf("%.2f", PODO_DATA.CoreSEN.OF.thmea*R2D));
    ui->LE_OPT_RIGHTACCX->setText(str.sprintf("%d", PODO_DATA.CoreSEN.OF.AccumX[0]));
    ui->LE_OPT_RIGHTACCY->setText(str.sprintf("%d", PODO_DATA.CoreSEN.OF.AccumY[0]));
    ui->LE_OPT_LEFTACCX->setText(str.sprintf("%d", PODO_DATA.CoreSEN.OF.AccumX[1]));
    ui->LE_OPT_LEFTACCY->setText(str.sprintf("%d", PODO_DATA.CoreSEN.OF.AccumY[1]));


}

void SensorDialog::on_BTN_SENSOR_ENABLE_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 1;     // on
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON_SENSOR_SENSOR_ONOFF;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void SensorDialog::on_BTN_SENSOR_DISABLE_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 0;     // off
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON_SENSOR_SENSOR_ONOFF;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void SensorDialog::on_BTN_SENSOR_FT_NULL_clicked(){
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = -1;     // all
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON_SENSOR_FT_NULL;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void SensorDialog::on_BTN_SENSOR_IMU_NULL_clicked(){
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = -1;	// all
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON_SENSOR_IMU_NULL;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void SensorDialog::on_BTN_CIMU_GET_OFFSET_clicked()
{
    ui->LE_SENSOR_CIMU_ROLL_OFFSET->setText(QString().sprintf("%.2f",-PODO_DATA.CoreSEN.IMU[0].Roll));
    ui->LE_SENSOR_CIMU_PITCH_OFFSET->setText(QString().sprintf("%.2f",-PODO_DATA.CoreSEN.IMU[0].Pitch));
}

void SensorDialog::on_BTN_CIMU_SET_OFFSET_clicked(){
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 0;	//CIMU
    cmd.COMMAND_DATA.USER_PARA_FLOAT[0] = ui->LE_SENSOR_CIMU_ROLL_OFFSET->text().toFloat();
    cmd.COMMAND_DATA.USER_PARA_FLOAT[1] = ui->LE_SENSOR_CIMU_PITCH_OFFSET->text().toFloat();
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON_SENSOR_IMU_OFFSET_SET;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void SensorDialog::on_BTN_SENSOR_FOG_ZERO_clicked(){
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 1; // Zero
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON_SENSOR_FOG_NULL;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void SensorDialog::on_BTN_SENSOR_FOG_NULL_clicked(){
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 0; // Null
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON_SENSOR_FOG_NULL;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void SensorDialog::on_BTN_OPTZERO_clicked(){
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON_SENSOR_OF_NULL;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void SensorDialog::on_BTN_OPT_LAMP_ON_clicked(){
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = true; // lamp on
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON_SENSOR_OF_LAMP_ONOFF;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void SensorDialog::on_BTN_OPT_LAMP_OFF_clicked(){
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = false; // lamp off
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON_SENSOR_OF_LAMP_ONOFF;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void SensorDialog::on_BTN_SENSOR_FOG_USB_RESET_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON_SENSOR_FOG_USB_RESET;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void SensorDialog::on_BTN_SENSOR_IMU_ZERO_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = -1;	// all
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON_SENSOR_IMU_ZERO;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void SensorDialog::on_NIMU_ENABLE_clicked()
{
    // enable
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 1;
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON_SENSOR_IMU_NULL;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void SensorDialog::on_NIMU_NULL_clicked()
{
    // null
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 2;
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON_SENSOR_IMU_NULL;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void SensorDialog::on_NIMU_RESET_clicked()
{
    // reset to acc angle
    USER_COMMAND cmd;
    ui->LE_SENSOR_CIMU_ROLL_OFFSET->setText(QString().sprintf("0.0"));
    ui->LE_SENSOR_CIMU_PITCH_OFFSET->setText(QString().sprintf("0.0"));
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 0;	//CIMU
    cmd.COMMAND_DATA.USER_PARA_FLOAT[0] = ui->LE_SENSOR_CIMU_ROLL_OFFSET->text().toFloat();
    cmd.COMMAND_DATA.USER_PARA_FLOAT[1] = ui->LE_SENSOR_CIMU_PITCH_OFFSET->text().toFloat();
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON_SENSOR_IMU_OFFSET_SET;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);

    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 3;
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON_SENSOR_IMU_NULL;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void SensorDialog::on_NIMU_Zero_clicked()
{
    // zero
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 4;
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON_SENSOR_IMU_NULL;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void SensorDialog::on_FOG_EN_clicked()
{
    // Data enable
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = 321;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 1;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void SensorDialog::on_FOG_NULL_clicked()
{
    // old nulling
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = 321;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 2;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void SensorDialog::on_pushButton_3_clicked()
{
    // old reset to acc
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = 321;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 3;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void SensorDialog::on_pushButton_4_clicked()
{
    // old reset to zero
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = 321;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 4;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void SensorDialog::on_pushButton_5_clicked()
{
    // new reset to acc
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = 321;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 5;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void SensorDialog::on_pushButton_6_clicked()
{
    // new reset to zero
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = 321;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 6;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}
