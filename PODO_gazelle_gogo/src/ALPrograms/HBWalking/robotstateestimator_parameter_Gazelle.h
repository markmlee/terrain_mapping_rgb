#ifndef ROBOTSTATEESTIMATOR_PARAMETER_GAZELLE
#define ROBOTSTATEESTIMATOR_PARAMETER_GAZELLE

#define     CV_LIPM_H           0.630       // LIMP height
#define     CV_CONTACT_TH_FORCE 55          // Contact detection threshold
#define     CV_LINK_L_P2H1      0.06        // link length: pelvis to hip1
#define     CV_LINK_L_H12H2Y    0.0491      // link length: hip1 to hip2 y
#define     CV_LINK_L_H12H2Z    0.072      // link length: hip1 to hip2 z
#define     CV_LINK_L_ULEG      0.33        // link length: hip2 to knee
#define     CV_LINK_L_LLEG      0.32249       // link length: knee to ankle pitch
#define     CV_LINK_L_AP2AR     0.000       // link length: ankle pitch to ankle roll
#define     CV_LINK_L_A2F       0.085       // link length: ankle to ground

#define     CV_LINK_L_P2SC      0.3675      // link length: pelvis to shoulder center
#define     CV_LINK_L_SC2S      0.215       // link length: shoulder center to shoulder
#define     CV_LINK_L_UARM      0.182       // link length: shoulder to elbow
#define     CV_LINK_L_LARM      0.164       // link length: elbow to wrist
#define     CV_LINK_L_OFFELB    0.022       // link length: elbow offset

#define     CV_LINK_L_FOOT_LX_1 0.110       // foot x-dir width front
#define     CV_LINK_L_FOOT_LX_2 0.110       // foot x-dir width back
#define     CV_LINK_L_FOOT_LY_1 0.075       // foot y-dir width inside
#define     CV_LINK_L_FOOT_LY_2 0.075       // foot y-dir width outside

#define     CV_MASS_M_HY        1e-6        // Mass: RHY/LHY
#define     CV_MASS_M_HR        1e-6        // Mass: RHR/LHR
#define     CV_MASS_M_HP        6.3512      // Mass: RHP/LHP
#define     CV_MASS_M_KN        1.9592      // Mass: RKN/LKN
#define     CV_MASS_M_AP        1e-6        // Mass: RAP/LAP
#define     CV_MASS_M_AR        2.6626      // Mass: RAR/LAR

#define     CV_MASS_M_SP        1e-6        // Mass: RSP/LSP
#define     CV_MASS_M_SR        1e-6        // Mass: RSR/LSR
#define     CV_MASS_M_SY        2.3147      // Mass: RSY/LSY
#define     CV_MASS_M_EB        0.5542      // Mass: REB/LEB
#define     CV_MASS_M_WY        1e-6        // Mass: RWY/LWY
#define     CV_MASS_M_WP        0.2247      // Mass: RWP/LWP
#define     CV_MASS_M_F1        0.0         // Mass: RF1/LF1

#define     CV_MASS_M_TORSO     7.2025      // Mass: Torso
#define     CV_MASS_M_PEL       3.8736      // Mass: Pelvis

#define     CV_MCL_L_RSP        {0.,0.,0.}        // mass cetner offset w/t RSP
#define     CV_MCL_L_RSR        {0.,0.,0.}       // mass cetner offset w/t RSR
#define     CV_MCL_L_RSY        {0.0062,0.0178,-0.0451}   // mass cetner offset w/t RSY
#define     CV_MCL_L_REB        {0.0003,-0.0006,-0.047}   // mass cetner offset w/t REB
#define     CV_MCL_L_RWY        {0.,0.,0.}         // mass cetner offset w/t RWY
#define     CV_MCL_L_RWP        {0.0033,-0.0015,-0.0635}    // mass cetner offset w/t RWP
#define     CV_MCL_L_RF1        {0.,0.,0.}                  // mass cetner offset w/t RF1

#define     CV_MCL_L_TORSO      {-0.0115,0,0.1347}      // mass cetner offset w/t WST
#define     CV_MCL_L_PEL        {-0.0119,0,0.1323}      // mass cetner offset w/t WST

#define     CV_MCL_L_RHY        {0.,0.,0.}              // mass cetner offset w/t RHY
#define     CV_MCL_L_RHR        {0.,0.,0.}              // mass cetner offset w/t RHR
#define     CV_MCL_L_RHP        {0.0175,-0.0099,-0.0995}    // mass cetner offset w/t RHP
#define     CV_MCL_L_RKN        {0.0146,-0.0146,-0.1845}    // mass cetner offset w/t RKN
#define     CV_MCL_L_RAP        {0.,0.,0.}              // mass cetner offset w/t RAP
#define     CV_MCL_L_RAR        {0.0216,-0.0014,-0.016} // mass cetner offset w/t RAR

#endif
