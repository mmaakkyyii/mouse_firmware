#ifndef _MACHINE_PARAMATER_H_
#define _MACHINE_PARAMATER_H_

const float SECTION_WIDTH=90.0;
const float TREAD_WIDTH=35.54;//TODO
const float TIYA_R=13.05/2.0;//TODO 12.75
const float MACHINE_BACK_LENGTH=90/2.0-0.3-25;
const float MACHINE_BACK_VOLTAGE_R=0.8;//TODO
const float MACHINE_BACK_VOLTAGE_L=0.8;//TODO
const float MACHINE_BACK_TIME=350;//TODO



const float CONTROL_PERIOD_ms=1.0;

const float Kp_motorR=0.00001;//0.001
const float Ki_motorR=0.002	;//0.001
const float Kd_motorR=0.00;

const float Kp_motorL=0.00001;
const float Ki_motorL=0.002;
const float Kd_motorL=0.00;

const float Kp_wall=0.008;//0.004
const float Kd_wall=0.000;//0.0003
const float Kp_omega=0.0005;//0.001
const float crash_acc=45.0;


#endif //_MACHINE_PARAMATER_H_
