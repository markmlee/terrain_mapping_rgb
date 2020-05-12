/********************************************************************************
** Form generated from reading UI file 'HBWalkingdialog.ui'
**
** Created by: Qt User Interface Compiler version 5.7.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_HBWALKINGDIALOG_H
#define UI_HBWALKINGDIALOG_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QDialog>
#include <QtWidgets/QFrame>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_HBWalkingDialog
{
public:
    QFrame *frame;
    QWidget *layoutWidget;
    QGridLayout *gridLayout;
    QLabel *label_2;
    QLabel *label_3;
    QLabel *label_4;
    QLineEdit *LE_NO_OF_STEP;
    QLineEdit *LE_STEP_TIME;
    QLineEdit *LE_STEP_STRIDE;
    QWidget *layoutWidget1;
    QHBoxLayout *horizontalLayout_2;
    QPushButton *BT_WALK_TEST;
    QPushButton *BT_READY_TO_WALK;
    QWidget *layoutWidget2;
    QHBoxLayout *horizontalLayout;
    QPushButton *BT_WALK_READY;
    QVBoxLayout *verticalLayout_2;
    QPushButton *BT_ONE_LEG_READY;
    QPushButton *BT_ONE_LEG_READY_2;
    QLabel *label_6;
    QFrame *frame_2;
    QWidget *layoutWidget3;
    QHBoxLayout *horizontalLayout_3;
    QPushButton *BT_CONTROL_ON;
    QPushButton *BT_INV_DYNAMICS_CON;
    QPushButton *BT_DYN_WALKING;
    QWidget *layoutWidget4;
    QGridLayout *gridLayout_2;
    QPushButton *BT_POS;
    QPushButton *BT_DYN_STANDING;
    QPushButton *BT_JUMP_READY;
    QPushButton *BT_JUMP;
    QWidget *layoutWidget5;
    QVBoxLayout *verticalLayout_3;
    QGridLayout *gridLayout_3;
    QLabel *label_12;
    QLabel *label_13;
    QHBoxLayout *horizontalLayout_4;
    QLineEdit *LE_IMPACT_TIME;
    QLabel *label_10;
    QHBoxLayout *horizontalLayout_5;
    QLineEdit *LE_IMPACT_FORCE;
    QLabel *label_11;
    QPushButton *BT_WALK_TEST_2;
    QWidget *layoutWidget6;
    QVBoxLayout *verticalLayout_5;
    QPushButton *BT_COM_HEIGHT;
    QPushButton *BT_LOGGING;
    QWidget *layoutWidget7;
    QVBoxLayout *verticalLayout_4;
    QLabel *label;
    QHBoxLayout *horizontalLayout_6;
    QLineEdit *LE_REF_TORQUE;
    QPushButton *BTN_APPLY;
    QFrame *frame_3;
    QLabel *label_5;
    QWidget *layoutWidget8;
    QGridLayout *gridLayout_4;
    QLabel *label_7;
    QLabel *label_8;
    QLabel *label_9;
    QLineEdit *LE_NO_OF_STEP_2;
    QLineEdit *LE_STEP_TIME_2;
    QLineEdit *LE_STEP_STRIDE_2;
    QPushButton *BT_WALK_SINGLELOG;
    QPushButton *BT_WALK_ROS;
    QLineEdit *LE_ROS_STATUS;
    QWidget *layoutWidget9;
    QVBoxLayout *verticalLayout;
    QPushButton *BT_TEST_START;
    QPushButton *BT_DATA_SAVE;
    QPushButton *BT_TORQUE_TEST;
    QPushButton *BT_TEST_STOP;
    QPushButton *BT_DATA_SAVE_2;

    void setupUi(QDialog *HBWalkingDialog)
    {
        if (HBWalkingDialog->objectName().isEmpty())
            HBWalkingDialog->setObjectName(QStringLiteral("HBWalkingDialog"));
        HBWalkingDialog->resize(622, 598);
        frame = new QFrame(HBWalkingDialog);
        frame->setObjectName(QStringLiteral("frame"));
        frame->setGeometry(QRect(20, 10, 491, 221));
        frame->setFrameShape(QFrame::StyledPanel);
        frame->setFrameShadow(QFrame::Raised);
        layoutWidget = new QWidget(frame);
        layoutWidget->setObjectName(QStringLiteral("layoutWidget"));
        layoutWidget->setGeometry(QRect(100, 100, 281, 51));
        gridLayout = new QGridLayout(layoutWidget);
        gridLayout->setSpacing(0);
        gridLayout->setObjectName(QStringLiteral("gridLayout"));
        gridLayout->setContentsMargins(0, 0, 0, 0);
        label_2 = new QLabel(layoutWidget);
        label_2->setObjectName(QStringLiteral("label_2"));
        label_2->setAlignment(Qt::AlignCenter);

        gridLayout->addWidget(label_2, 0, 0, 1, 1);

        label_3 = new QLabel(layoutWidget);
        label_3->setObjectName(QStringLiteral("label_3"));
        label_3->setAlignment(Qt::AlignCenter);

        gridLayout->addWidget(label_3, 0, 1, 1, 1);

        label_4 = new QLabel(layoutWidget);
        label_4->setObjectName(QStringLiteral("label_4"));
        label_4->setAlignment(Qt::AlignCenter);

        gridLayout->addWidget(label_4, 0, 2, 1, 1);

        LE_NO_OF_STEP = new QLineEdit(layoutWidget);
        LE_NO_OF_STEP->setObjectName(QStringLiteral("LE_NO_OF_STEP"));
        QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(LE_NO_OF_STEP->sizePolicy().hasHeightForWidth());
        LE_NO_OF_STEP->setSizePolicy(sizePolicy);

        gridLayout->addWidget(LE_NO_OF_STEP, 1, 0, 1, 1);

        LE_STEP_TIME = new QLineEdit(layoutWidget);
        LE_STEP_TIME->setObjectName(QStringLiteral("LE_STEP_TIME"));
        sizePolicy.setHeightForWidth(LE_STEP_TIME->sizePolicy().hasHeightForWidth());
        LE_STEP_TIME->setSizePolicy(sizePolicy);

        gridLayout->addWidget(LE_STEP_TIME, 1, 1, 1, 1);

        LE_STEP_STRIDE = new QLineEdit(layoutWidget);
        LE_STEP_STRIDE->setObjectName(QStringLiteral("LE_STEP_STRIDE"));
        sizePolicy.setHeightForWidth(LE_STEP_STRIDE->sizePolicy().hasHeightForWidth());
        LE_STEP_STRIDE->setSizePolicy(sizePolicy);

        gridLayout->addWidget(LE_STEP_STRIDE, 1, 2, 1, 1);

        layoutWidget1 = new QWidget(frame);
        layoutWidget1->setObjectName(QStringLiteral("layoutWidget1"));
        layoutWidget1->setGeometry(QRect(99, 160, 281, 51));
        horizontalLayout_2 = new QHBoxLayout(layoutWidget1);
        horizontalLayout_2->setObjectName(QStringLiteral("horizontalLayout_2"));
        horizontalLayout_2->setContentsMargins(0, 0, 0, 0);
        BT_WALK_TEST = new QPushButton(layoutWidget1);
        BT_WALK_TEST->setObjectName(QStringLiteral("BT_WALK_TEST"));
        QSizePolicy sizePolicy1(QSizePolicy::Minimum, QSizePolicy::Expanding);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(BT_WALK_TEST->sizePolicy().hasHeightForWidth());
        BT_WALK_TEST->setSizePolicy(sizePolicy1);

        horizontalLayout_2->addWidget(BT_WALK_TEST);

        BT_READY_TO_WALK = new QPushButton(layoutWidget1);
        BT_READY_TO_WALK->setObjectName(QStringLiteral("BT_READY_TO_WALK"));
        sizePolicy1.setHeightForWidth(BT_READY_TO_WALK->sizePolicy().hasHeightForWidth());
        BT_READY_TO_WALK->setSizePolicy(sizePolicy1);

        horizontalLayout_2->addWidget(BT_READY_TO_WALK);

        layoutWidget2 = new QWidget(frame);
        layoutWidget2->setObjectName(QStringLiteral("layoutWidget2"));
        layoutWidget2->setGeometry(QRect(100, 10, 281, 81));
        horizontalLayout = new QHBoxLayout(layoutWidget2);
        horizontalLayout->setSpacing(0);
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        horizontalLayout->setContentsMargins(0, 0, 0, 0);
        BT_WALK_READY = new QPushButton(layoutWidget2);
        BT_WALK_READY->setObjectName(QStringLiteral("BT_WALK_READY"));
        sizePolicy.setHeightForWidth(BT_WALK_READY->sizePolicy().hasHeightForWidth());
        BT_WALK_READY->setSizePolicy(sizePolicy);

        horizontalLayout->addWidget(BT_WALK_READY);

        verticalLayout_2 = new QVBoxLayout();
        verticalLayout_2->setObjectName(QStringLiteral("verticalLayout_2"));
        BT_ONE_LEG_READY = new QPushButton(layoutWidget2);
        BT_ONE_LEG_READY->setObjectName(QStringLiteral("BT_ONE_LEG_READY"));
        sizePolicy.setHeightForWidth(BT_ONE_LEG_READY->sizePolicy().hasHeightForWidth());
        BT_ONE_LEG_READY->setSizePolicy(sizePolicy);

        verticalLayout_2->addWidget(BT_ONE_LEG_READY);

        BT_ONE_LEG_READY_2 = new QPushButton(layoutWidget2);
        BT_ONE_LEG_READY_2->setObjectName(QStringLiteral("BT_ONE_LEG_READY_2"));
        sizePolicy.setHeightForWidth(BT_ONE_LEG_READY_2->sizePolicy().hasHeightForWidth());
        BT_ONE_LEG_READY_2->setSizePolicy(sizePolicy);

        verticalLayout_2->addWidget(BT_ONE_LEG_READY_2);


        horizontalLayout->addLayout(verticalLayout_2);

        label_6 = new QLabel(frame);
        label_6->setObjectName(QStringLiteral("label_6"));
        label_6->setGeometry(QRect(0, 0, 81, 16));
        frame_2 = new QFrame(HBWalkingDialog);
        frame_2->setObjectName(QStringLiteral("frame_2"));
        frame_2->setGeometry(QRect(20, 380, 581, 201));
        frame_2->setFrameShape(QFrame::StyledPanel);
        frame_2->setFrameShadow(QFrame::Raised);
        layoutWidget3 = new QWidget(frame_2);
        layoutWidget3->setObjectName(QStringLiteral("layoutWidget3"));
        layoutWidget3->setGeometry(QRect(10, 20, 381, 51));
        horizontalLayout_3 = new QHBoxLayout(layoutWidget3);
        horizontalLayout_3->setObjectName(QStringLiteral("horizontalLayout_3"));
        horizontalLayout_3->setContentsMargins(0, 0, 0, 0);
        BT_CONTROL_ON = new QPushButton(layoutWidget3);
        BT_CONTROL_ON->setObjectName(QStringLiteral("BT_CONTROL_ON"));
        sizePolicy1.setHeightForWidth(BT_CONTROL_ON->sizePolicy().hasHeightForWidth());
        BT_CONTROL_ON->setSizePolicy(sizePolicy1);

        horizontalLayout_3->addWidget(BT_CONTROL_ON);

        BT_INV_DYNAMICS_CON = new QPushButton(layoutWidget3);
        BT_INV_DYNAMICS_CON->setObjectName(QStringLiteral("BT_INV_DYNAMICS_CON"));
        sizePolicy1.setHeightForWidth(BT_INV_DYNAMICS_CON->sizePolicy().hasHeightForWidth());
        BT_INV_DYNAMICS_CON->setSizePolicy(sizePolicy1);

        horizontalLayout_3->addWidget(BT_INV_DYNAMICS_CON);

        BT_DYN_WALKING = new QPushButton(layoutWidget3);
        BT_DYN_WALKING->setObjectName(QStringLiteral("BT_DYN_WALKING"));
        sizePolicy1.setHeightForWidth(BT_DYN_WALKING->sizePolicy().hasHeightForWidth());
        BT_DYN_WALKING->setSizePolicy(sizePolicy1);

        horizontalLayout_3->addWidget(BT_DYN_WALKING);

        layoutWidget4 = new QWidget(frame_2);
        layoutWidget4->setObjectName(QStringLiteral("layoutWidget4"));
        layoutWidget4->setGeometry(QRect(160, 90, 231, 91));
        gridLayout_2 = new QGridLayout(layoutWidget4);
        gridLayout_2->setSpacing(0);
        gridLayout_2->setObjectName(QStringLiteral("gridLayout_2"));
        gridLayout_2->setContentsMargins(0, 0, 0, 0);
        BT_POS = new QPushButton(layoutWidget4);
        BT_POS->setObjectName(QStringLiteral("BT_POS"));
        sizePolicy1.setHeightForWidth(BT_POS->sizePolicy().hasHeightForWidth());
        BT_POS->setSizePolicy(sizePolicy1);

        gridLayout_2->addWidget(BT_POS, 0, 0, 1, 1);

        BT_DYN_STANDING = new QPushButton(layoutWidget4);
        BT_DYN_STANDING->setObjectName(QStringLiteral("BT_DYN_STANDING"));
        sizePolicy1.setHeightForWidth(BT_DYN_STANDING->sizePolicy().hasHeightForWidth());
        BT_DYN_STANDING->setSizePolicy(sizePolicy1);

        gridLayout_2->addWidget(BT_DYN_STANDING, 0, 1, 1, 1);

        BT_JUMP_READY = new QPushButton(layoutWidget4);
        BT_JUMP_READY->setObjectName(QStringLiteral("BT_JUMP_READY"));
        sizePolicy1.setHeightForWidth(BT_JUMP_READY->sizePolicy().hasHeightForWidth());
        BT_JUMP_READY->setSizePolicy(sizePolicy1);

        gridLayout_2->addWidget(BT_JUMP_READY, 1, 0, 1, 1);

        BT_JUMP = new QPushButton(layoutWidget4);
        BT_JUMP->setObjectName(QStringLiteral("BT_JUMP"));
        sizePolicy1.setHeightForWidth(BT_JUMP->sizePolicy().hasHeightForWidth());
        BT_JUMP->setSizePolicy(sizePolicy1);

        gridLayout_2->addWidget(BT_JUMP, 1, 1, 1, 1);

        layoutWidget5 = new QWidget(frame_2);
        layoutWidget5->setObjectName(QStringLiteral("layoutWidget5"));
        layoutWidget5->setGeometry(QRect(10, 90, 141, 91));
        verticalLayout_3 = new QVBoxLayout(layoutWidget5);
        verticalLayout_3->setSpacing(0);
        verticalLayout_3->setObjectName(QStringLiteral("verticalLayout_3"));
        verticalLayout_3->setContentsMargins(0, 0, 0, 0);
        gridLayout_3 = new QGridLayout();
        gridLayout_3->setObjectName(QStringLiteral("gridLayout_3"));
        label_12 = new QLabel(layoutWidget5);
        label_12->setObjectName(QStringLiteral("label_12"));

        gridLayout_3->addWidget(label_12, 0, 0, 1, 1);

        label_13 = new QLabel(layoutWidget5);
        label_13->setObjectName(QStringLiteral("label_13"));

        gridLayout_3->addWidget(label_13, 0, 1, 1, 1);

        horizontalLayout_4 = new QHBoxLayout();
        horizontalLayout_4->setObjectName(QStringLiteral("horizontalLayout_4"));
        LE_IMPACT_TIME = new QLineEdit(layoutWidget5);
        LE_IMPACT_TIME->setObjectName(QStringLiteral("LE_IMPACT_TIME"));

        horizontalLayout_4->addWidget(LE_IMPACT_TIME);

        label_10 = new QLabel(layoutWidget5);
        label_10->setObjectName(QStringLiteral("label_10"));

        horizontalLayout_4->addWidget(label_10);


        gridLayout_3->addLayout(horizontalLayout_4, 1, 0, 1, 1);

        horizontalLayout_5 = new QHBoxLayout();
        horizontalLayout_5->setObjectName(QStringLiteral("horizontalLayout_5"));
        LE_IMPACT_FORCE = new QLineEdit(layoutWidget5);
        LE_IMPACT_FORCE->setObjectName(QStringLiteral("LE_IMPACT_FORCE"));

        horizontalLayout_5->addWidget(LE_IMPACT_FORCE);

        label_11 = new QLabel(layoutWidget5);
        label_11->setObjectName(QStringLiteral("label_11"));

        horizontalLayout_5->addWidget(label_11);


        gridLayout_3->addLayout(horizontalLayout_5, 1, 1, 1, 1);


        verticalLayout_3->addLayout(gridLayout_3);

        BT_WALK_TEST_2 = new QPushButton(layoutWidget5);
        BT_WALK_TEST_2->setObjectName(QStringLiteral("BT_WALK_TEST_2"));
        sizePolicy1.setHeightForWidth(BT_WALK_TEST_2->sizePolicy().hasHeightForWidth());
        BT_WALK_TEST_2->setSizePolicy(sizePolicy1);
        QFont font;
        font.setPointSize(15);
        BT_WALK_TEST_2->setFont(font);

        verticalLayout_3->addWidget(BT_WALK_TEST_2);

        layoutWidget6 = new QWidget(frame_2);
        layoutWidget6->setObjectName(QStringLiteral("layoutWidget6"));
        layoutWidget6->setGeometry(QRect(456, 20, 111, 52));
        verticalLayout_5 = new QVBoxLayout(layoutWidget6);
        verticalLayout_5->setObjectName(QStringLiteral("verticalLayout_5"));
        verticalLayout_5->setContentsMargins(0, 0, 0, 0);
        BT_COM_HEIGHT = new QPushButton(layoutWidget6);
        BT_COM_HEIGHT->setObjectName(QStringLiteral("BT_COM_HEIGHT"));
        sizePolicy1.setHeightForWidth(BT_COM_HEIGHT->sizePolicy().hasHeightForWidth());
        BT_COM_HEIGHT->setSizePolicy(sizePolicy1);

        verticalLayout_5->addWidget(BT_COM_HEIGHT);

        BT_LOGGING = new QPushButton(layoutWidget6);
        BT_LOGGING->setObjectName(QStringLiteral("BT_LOGGING"));
        sizePolicy1.setHeightForWidth(BT_LOGGING->sizePolicy().hasHeightForWidth());
        BT_LOGGING->setSizePolicy(sizePolicy1);

        verticalLayout_5->addWidget(BT_LOGGING);

        layoutWidget7 = new QWidget(frame_2);
        layoutWidget7->setObjectName(QStringLiteral("layoutWidget7"));
        layoutWidget7->setGeometry(QRect(400, 130, 171, 46));
        verticalLayout_4 = new QVBoxLayout(layoutWidget7);
        verticalLayout_4->setObjectName(QStringLiteral("verticalLayout_4"));
        verticalLayout_4->setContentsMargins(0, 0, 0, 0);
        label = new QLabel(layoutWidget7);
        label->setObjectName(QStringLiteral("label"));

        verticalLayout_4->addWidget(label);

        horizontalLayout_6 = new QHBoxLayout();
        horizontalLayout_6->setObjectName(QStringLiteral("horizontalLayout_6"));
        LE_REF_TORQUE = new QLineEdit(layoutWidget7);
        LE_REF_TORQUE->setObjectName(QStringLiteral("LE_REF_TORQUE"));

        horizontalLayout_6->addWidget(LE_REF_TORQUE);

        BTN_APPLY = new QPushButton(layoutWidget7);
        BTN_APPLY->setObjectName(QStringLiteral("BTN_APPLY"));

        horizontalLayout_6->addWidget(BTN_APPLY);


        verticalLayout_4->addLayout(horizontalLayout_6);

        frame_3 = new QFrame(HBWalkingDialog);
        frame_3->setObjectName(QStringLiteral("frame_3"));
        frame_3->setGeometry(QRect(20, 240, 581, 131));
        frame_3->setFrameShape(QFrame::StyledPanel);
        frame_3->setFrameShadow(QFrame::Raised);
        label_5 = new QLabel(frame_3);
        label_5->setObjectName(QStringLiteral("label_5"));
        label_5->setGeometry(QRect(0, 0, 91, 16));
        layoutWidget8 = new QWidget(frame_3);
        layoutWidget8->setObjectName(QStringLiteral("layoutWidget8"));
        layoutWidget8->setGeometry(QRect(20, 20, 281, 51));
        gridLayout_4 = new QGridLayout(layoutWidget8);
        gridLayout_4->setSpacing(0);
        gridLayout_4->setObjectName(QStringLiteral("gridLayout_4"));
        gridLayout_4->setContentsMargins(0, 0, 0, 0);
        label_7 = new QLabel(layoutWidget8);
        label_7->setObjectName(QStringLiteral("label_7"));
        label_7->setAlignment(Qt::AlignCenter);

        gridLayout_4->addWidget(label_7, 0, 0, 1, 1);

        label_8 = new QLabel(layoutWidget8);
        label_8->setObjectName(QStringLiteral("label_8"));
        label_8->setAlignment(Qt::AlignCenter);

        gridLayout_4->addWidget(label_8, 0, 1, 1, 1);

        label_9 = new QLabel(layoutWidget8);
        label_9->setObjectName(QStringLiteral("label_9"));
        label_9->setAlignment(Qt::AlignCenter);

        gridLayout_4->addWidget(label_9, 0, 2, 1, 1);

        LE_NO_OF_STEP_2 = new QLineEdit(layoutWidget8);
        LE_NO_OF_STEP_2->setObjectName(QStringLiteral("LE_NO_OF_STEP_2"));
        sizePolicy.setHeightForWidth(LE_NO_OF_STEP_2->sizePolicy().hasHeightForWidth());
        LE_NO_OF_STEP_2->setSizePolicy(sizePolicy);

        gridLayout_4->addWidget(LE_NO_OF_STEP_2, 1, 0, 1, 1);

        LE_STEP_TIME_2 = new QLineEdit(layoutWidget8);
        LE_STEP_TIME_2->setObjectName(QStringLiteral("LE_STEP_TIME_2"));
        sizePolicy.setHeightForWidth(LE_STEP_TIME_2->sizePolicy().hasHeightForWidth());
        LE_STEP_TIME_2->setSizePolicy(sizePolicy);

        gridLayout_4->addWidget(LE_STEP_TIME_2, 1, 1, 1, 1);

        LE_STEP_STRIDE_2 = new QLineEdit(layoutWidget8);
        LE_STEP_STRIDE_2->setObjectName(QStringLiteral("LE_STEP_STRIDE_2"));
        sizePolicy.setHeightForWidth(LE_STEP_STRIDE_2->sizePolicy().hasHeightForWidth());
        LE_STEP_STRIDE_2->setSizePolicy(sizePolicy);

        gridLayout_4->addWidget(LE_STEP_STRIDE_2, 1, 2, 1, 1);

        BT_WALK_SINGLELOG = new QPushButton(frame_3);
        BT_WALK_SINGLELOG->setObjectName(QStringLiteral("BT_WALK_SINGLELOG"));
        BT_WALK_SINGLELOG->setGeometry(QRect(320, 20, 121, 49));
        sizePolicy1.setHeightForWidth(BT_WALK_SINGLELOG->sizePolicy().hasHeightForWidth());
        BT_WALK_SINGLELOG->setSizePolicy(sizePolicy1);
        BT_WALK_ROS = new QPushButton(frame_3);
        BT_WALK_ROS->setObjectName(QStringLiteral("BT_WALK_ROS"));
        BT_WALK_ROS->setGeometry(QRect(450, 70, 101, 49));
        sizePolicy1.setHeightForWidth(BT_WALK_ROS->sizePolicy().hasHeightForWidth());
        BT_WALK_ROS->setSizePolicy(sizePolicy1);
        LE_ROS_STATUS = new QLineEdit(frame_3);
        LE_ROS_STATUS->setObjectName(QStringLiteral("LE_ROS_STATUS"));
        LE_ROS_STATUS->setGeometry(QRect(450, 30, 101, 31));
        LE_ROS_STATUS->setAutoFillBackground(false);
        LE_ROS_STATUS->setAlignment(Qt::AlignCenter);
        LE_ROS_STATUS->setReadOnly(true);
        layoutWidget9 = new QWidget(HBWalkingDialog);
        layoutWidget9->setObjectName(QStringLiteral("layoutWidget9"));
        layoutWidget9->setGeometry(QRect(520, 10, 82, 121));
        verticalLayout = new QVBoxLayout(layoutWidget9);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        verticalLayout->setContentsMargins(0, 0, 0, 0);
        BT_TEST_START = new QPushButton(layoutWidget9);
        BT_TEST_START->setObjectName(QStringLiteral("BT_TEST_START"));
        sizePolicy1.setHeightForWidth(BT_TEST_START->sizePolicy().hasHeightForWidth());
        BT_TEST_START->setSizePolicy(sizePolicy1);

        verticalLayout->addWidget(BT_TEST_START);

        BT_DATA_SAVE = new QPushButton(layoutWidget9);
        BT_DATA_SAVE->setObjectName(QStringLiteral("BT_DATA_SAVE"));
        sizePolicy1.setHeightForWidth(BT_DATA_SAVE->sizePolicy().hasHeightForWidth());
        BT_DATA_SAVE->setSizePolicy(sizePolicy1);

        verticalLayout->addWidget(BT_DATA_SAVE);

        BT_TORQUE_TEST = new QPushButton(layoutWidget9);
        BT_TORQUE_TEST->setObjectName(QStringLiteral("BT_TORQUE_TEST"));
        sizePolicy1.setHeightForWidth(BT_TORQUE_TEST->sizePolicy().hasHeightForWidth());
        BT_TORQUE_TEST->setSizePolicy(sizePolicy1);

        verticalLayout->addWidget(BT_TORQUE_TEST);

        BT_TEST_STOP = new QPushButton(layoutWidget9);
        BT_TEST_STOP->setObjectName(QStringLiteral("BT_TEST_STOP"));
        sizePolicy1.setHeightForWidth(BT_TEST_STOP->sizePolicy().hasHeightForWidth());
        BT_TEST_STOP->setSizePolicy(sizePolicy1);

        verticalLayout->addWidget(BT_TEST_STOP);

        BT_DATA_SAVE_2 = new QPushButton(HBWalkingDialog);
        BT_DATA_SAVE_2->setObjectName(QStringLiteral("BT_DATA_SAVE_2"));
        BT_DATA_SAVE_2->setGeometry(QRect(530, 180, 61, 41));
        sizePolicy1.setHeightForWidth(BT_DATA_SAVE_2->sizePolicy().hasHeightForWidth());
        BT_DATA_SAVE_2->setSizePolicy(sizePolicy1);

        retranslateUi(HBWalkingDialog);

        QMetaObject::connectSlotsByName(HBWalkingDialog);
    } // setupUi

    void retranslateUi(QDialog *HBWalkingDialog)
    {
        HBWalkingDialog->setWindowTitle(QApplication::translate("HBWalkingDialog", "Dialog", 0));
        label_2->setText(QApplication::translate("HBWalkingDialog", "No Of Step", 0));
        label_3->setText(QApplication::translate("HBWalkingDialog", "Step Time(s)", 0));
        label_4->setText(QApplication::translate("HBWalkingDialog", "Step Stride(m)", 0));
        LE_NO_OF_STEP->setText(QApplication::translate("HBWalkingDialog", "5", 0));
        LE_STEP_TIME->setText(QApplication::translate("HBWalkingDialog", "0.8", 0));
        LE_STEP_STRIDE->setText(QApplication::translate("HBWalkingDialog", "0", 0));
        BT_WALK_TEST->setText(QApplication::translate("HBWalkingDialog", "Position Walking", 0));
        BT_READY_TO_WALK->setText(QApplication::translate("HBWalkingDialog", "Ready to Walk", 0));
        BT_WALK_READY->setText(QApplication::translate("HBWalkingDialog", "Walk Ready", 0));
        BT_ONE_LEG_READY->setText(QApplication::translate("HBWalkingDialog", "Right Stance", 0));
        BT_ONE_LEG_READY_2->setText(QApplication::translate("HBWalkingDialog", "Left Stance", 0));
        label_6->setText(QApplication::translate("HBWalkingDialog", "PreviewWalk", 0));
        BT_CONTROL_ON->setText(QApplication::translate("HBWalkingDialog", "Control On", 0));
        BT_INV_DYNAMICS_CON->setText(QApplication::translate("HBWalkingDialog", "Inv Dynamics Con", 0));
        BT_DYN_WALKING->setText(QApplication::translate("HBWalkingDialog", "Dynamics Walking", 0));
        BT_POS->setText(QApplication::translate("HBWalkingDialog", "Pos Standing", 0));
        BT_DYN_STANDING->setText(QApplication::translate("HBWalkingDialog", "Dyn Standing", 0));
        BT_JUMP_READY->setText(QApplication::translate("HBWalkingDialog", "Jump Ready", 0));
        BT_JUMP->setText(QApplication::translate("HBWalkingDialog", "Jump", 0));
        label_12->setText(QApplication::translate("HBWalkingDialog", "Time", 0));
        label_13->setText(QApplication::translate("HBWalkingDialog", "Force", 0));
        LE_IMPACT_TIME->setText(QApplication::translate("HBWalkingDialog", "20", 0));
        label_10->setText(QApplication::translate("HBWalkingDialog", "ms", 0));
        LE_IMPACT_FORCE->setText(QApplication::translate("HBWalkingDialog", "0.01", 0));
        label_11->setText(QApplication::translate("HBWalkingDialog", "N", 0));
        BT_WALK_TEST_2->setText(QApplication::translate("HBWalkingDialog", "Impact !", 0));
        BT_COM_HEIGHT->setText(QApplication::translate("HBWalkingDialog", "Get COM Height", 0));
        BT_LOGGING->setText(QApplication::translate("HBWalkingDialog", "Logging", 0));
        label->setText(QApplication::translate("HBWalkingDialog", "Ref torque", 0));
        BTN_APPLY->setText(QApplication::translate("HBWalkingDialog", "Apply", 0));
        label_5->setText(QApplication::translate("HBWalkingDialog", "SingleLogWalk", 0));
        label_7->setText(QApplication::translate("HBWalkingDialog", "No Of Step", 0));
        label_8->setText(QApplication::translate("HBWalkingDialog", "Step Time(s)", 0));
        label_9->setText(QApplication::translate("HBWalkingDialog", "Step Stride(m)", 0));
        LE_NO_OF_STEP_2->setText(QApplication::translate("HBWalkingDialog", "5", 0));
        LE_STEP_TIME_2->setText(QApplication::translate("HBWalkingDialog", "0.8", 0));
        LE_STEP_STRIDE_2->setText(QApplication::translate("HBWalkingDialog", "0", 0));
        BT_WALK_SINGLELOG->setText(QApplication::translate("HBWalkingDialog", "Position Walking", 0));
        BT_WALK_ROS->setText(QApplication::translate("HBWalkingDialog", "ROS Walking\n"
"Start", 0));
        LE_ROS_STATUS->setText(QApplication::translate("HBWalkingDialog", "Disconnected", 0));
        BT_TEST_START->setText(QApplication::translate("HBWalkingDialog", "Test Start", 0));
        BT_DATA_SAVE->setText(QApplication::translate("HBWalkingDialog", "data save", 0));
        BT_TORQUE_TEST->setText(QApplication::translate("HBWalkingDialog", "Torque Test", 0));
        BT_TEST_STOP->setText(QApplication::translate("HBWalkingDialog", "Stop", 0));
        BT_DATA_SAVE_2->setText(QApplication::translate("HBWalkingDialog", "walkready save", 0));
    } // retranslateUi

};

namespace Ui {
    class HBWalkingDialog: public Ui_HBWalkingDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_HBWALKINGDIALOG_H
