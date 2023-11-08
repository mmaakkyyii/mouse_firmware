#ifndef _MACHINE_PARAMATER_H_
#define _MACHINE_PARAMATER_H_

const float SECTION_WIDTH=90.0;
const float TREAD_WIDTH=35;//TODO
const float TIYA_R=12.75;//TODO
const float MACHINE_BACK_LENGTH=20;//TODO
const int MAZESIZE_X=16;
const int MAZESIZE_Y=16;

const int MAZEGOAL_X=3;
const int MAZEGOAL_Y=3;

const float CONTROL_PERIOD_ms=1.0;

const float Kp_motorR=0.0005;
const float Ki_motorR=0.00003;
const float Kd_motorR=0.0;

const float Kp_motorL=0.0005;
const float Ki_motorL=0.00004;
const float Kd_motorL=0.0;

const float Kp_wall=0.004;//0.004 0.0025

#endif //_MACHINE_PARAMATER_H_
