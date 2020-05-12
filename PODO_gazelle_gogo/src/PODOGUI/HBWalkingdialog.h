#ifndef HBWALKINGDIALOG_H
#define HBWALKINGDIALOG_H

#include <QDialog>

#include "CommonHeader.h"

namespace Ui {
class HBWalkingDialog;
}

class HBWalkingDialog : public QDialog
{
    Q_OBJECT

public:
    explicit HBWalkingDialog(QWidget *parent = 0, LANDialog *_lanData = 0);
    ~HBWalkingDialog();

private slots:
    void DisplayUpdate();
    void on_BT_WALK_TEST_clicked();
    
    void on_BT_WALK_READY_clicked();

    void on_BT_DATA_SAVE_clicked();

    void on_BT_TEST_STOP_clicked();

    void on_BT_SYSID_INPUT_START_clicked();

    void on_BT_SYSID_STOP_SAVE_clicked();

    void on_BT_SYSID_INPUT_START_2_clicked();

    void on_BT_WALK_TEST_2_clicked();

    void on_BT_CONTROL_ON_clicked();

    void on_BT_OLTORQUE_GO_clicked();

    void on_BT_ONE_LEG_READY_clicked();

    void on_BT_TORQUE_TEST_clicked();

    void on_BT_ZERO_GAIN_clicked();

    void on_BT_INV_DYNAMICS_CON_clicked();

    void on_BT_DYN_STANDING_clicked();

    void on_BT_POS_clicked();

    void on_BT_DYN_WALKING_clicked();

    void on_BT_JUMP_clicked();

    void on_BT_JUMP_READY_clicked();

    void on_BT_COM_HEIGHT_clicked();

    void on_BT_LOGGING_clicked();

    void on_BT_ONE_LEG_READY_2_clicked();

    void on_BT_READY_TO_WALK_clicked();

    void on_BT_WALK_SINGLELOG_clicked();

    void on_BT_WALK_ROS_clicked();
    
    void on_BT_DATA_SAVE_2_clicked();

private:
    LANDialog			*lanData;
    Ui::HBWalkingDialog *ui;
    QTimer				*displayTimer;

    int                 AlnumHBWalking;
    int                 AlnumWalkReady;
};

#endif // HBWALKINGDIALOG_H
