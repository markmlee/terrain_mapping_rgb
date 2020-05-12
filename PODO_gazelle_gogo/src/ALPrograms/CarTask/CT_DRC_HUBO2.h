//-----------------------------------------
// Computed Torque Method for DRC Hubo2
// CT_DRC_HUBO2.h
// Inhyeok Kim (inhyeok@raionbow.re.kr)
// Rainbow Inc.
//-----------------------------------------

#ifndef CT_DRC_HUBO2_H
#define CT_DRC_HUBO2_H

#include "../../../share/Headers/kine_drc_hubo2.h"

#ifndef PI
#define PI			3.141592653589793
#endif

#ifndef D2R
#define D2R			1.745329251994330e-2
#endif

#ifndef R2D
#define R2D			5.729577951308232e1
#endif

#ifndef EPS
#define EPS         1e-6
#endif


class CT_DRC_HUBO2 : public CKINE_DRC_HUBO2
{
public:
    CT_DRC_HUBO2();
    ~CT_DRC_HUBO2();

public:
    long get_ct_version() {return ct_version;}


public:    
    double g_3x1[3];

public:
    int get_gravity_RightArm(const double Q_7x1[], const double qBase_4x1[], double wst_ang,
                        double torque_7x1[]);
    int get_gravity_LeftArm(const double Q_7x1[], const double qBase_4x1[], double wst_ang,
                       double torque_7x1[]);



private:
    long ct_version;


};


#endif // CT_DRC_HUBO2_H
