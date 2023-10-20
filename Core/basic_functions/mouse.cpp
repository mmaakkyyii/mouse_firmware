#include "mouse.hpp"
#include "debug.hpp"
#include "init_mpu.hpp"
#include "Jacobian.hpp"
#include "delay.h"

Mouse::Mouse(PID_Controler* motorR, PID_Controler* motorL, Motors* _motors, Localization* _localization, Encorders* _encorders,IMU* _imu, WallSensor* _wall_sensor, BatteryCheck* _battery_check, Buzzer* _buzzer,UI* _ui,MazeSolver* _maze_solver)
:motorR_PID(motorR),
 motorL_PID(motorL),
 motors(_motors),
 localization(_localization),
 encorders(_encorders),
 imu(_imu),
 wall_sensor(_wall_sensor),
 battery_check(_battery_check),
 buzzer(_buzzer),
 ui(_ui),
 maze_solver(_maze_solver)
 {
//	mode=new ModeSelect();
	period_ms=1.0;
	
	v_max=350;
	turn_v_max=200;

}

void Mouse::Init(){
	init_mpu();
	
	ui->Init();
	wall_sensor->Init();
	battery_check->Init();
	buzzer->Init();
	motors->Init();
	encorders->InitEncorders();
	imu->Init();
	
	int map_data[MAZESIZE_X][MAZESIZE_Y]={0};
	maze_solver->Init();
	maze_solver->adachi.InitMaze(UNKNOWN,map_data);
	mouse_pos_x=0;
	mouse_pos_y=0;
	mouse_dir=North;

	maze_solver->adachi.SetMap(mouse_pos_x,mouse_pos_y,map_data[mouse_pos_x][mouse_pos_y],mouse_dir);


	buzzer->SetFrequency(4000);	//�l�������������̎��g����ݒ�				
	buzzer->On();		//�u�U�[�𔭐U������
	delay_ms(100);
	buzzer->Off();		//�u�U�[�̔��U���~������	
	printf("Hello ita_PiCo\r\n");

//	delete trajectory;
//	trajectory=new Line(0.0, 180.0/2, 0.0, 0, v_max, v_max, 10000.0, 0.0);
	mouse_pos_y++;

	timer_start();

}
void Mouse::Interrupt_10ms(){

}

void SetGoal(int x, int y){
	
}
int Mouse::GetWallInfo(){
	int FR=wall_sensor->GetWallFR();
	int FL=wall_sensor->GetWallFL();
	int R=wall_sensor->GetWallR();
	int L=wall_sensor->GetWallL();

	int wall_info=0;
	switch(mouse_dir){
		//NWSE
		//3210
		case North:
			wall_info=(L<<2) | (FR|FL)<<3 | R << 0;
			break;
		case West:
			wall_info=(L<<1) | (FR|FL)<<2 | R << 3;
			break;
		case South:
			wall_info=(L<<0) | (FR|FL)<<1 | R << 2;
			break;
		case East:
			wall_info=(L<<3) | (FR|FL)<<0 | R << 1;
			break;
	}
	
	return wall_info;

}

/*

void Mouse::SerchRunMode(){
	if(trajectory->Update()){
		
		ui.SetLED( wall_sensor->GetWallL() <<3 |  
			   wall_sensor->GetWallFL()<<2 |
			   wall_sensor->GetWallFR()<<1 |  
			   wall_sensor->GetWallR()
			 );
		
		maze_solver.adachi.MakeStepMap(goal_pos_x,goal_pos_y,wall_mask);
 		
		if(mouse_pos_x==goal_pos_x && mouse_pos_y==goal_pos_y ){
			goal_time++;
			if(goal_time%2==1){
				goal_pos_x = 0;
				goal_pos_y = 0;
			}else if(goal_time%2==0){
				goal_pos_x = GOAL_X;
				goal_pos_y = GOAL_Y;
				wall_mask=UNUSE_UNKOWN_WALL_MASK;

			}
			switch(mouse_dir){
			case North:
				mouse_dir=South;
				mouse_pos_y--;
				break;
			case West:
				mouse_dir=East;
				mouse_pos_x++;
				break;
			case South:
				mouse_dir=North;
				mouse_pos_y++;
				break;
			case East:
				mouse_dir=West;
				mouse_pos_x--;
				break;
			}
			
			delete trajectory;
			trajectory=new DoubleTrajectory(
				new DoubleTrajectory(
					new Line(0.0, 180.0/2, 0.0 , v_max, v_max, 0.0, 10000.0, 0.0),
					new Rotate(180, 0, turn_v_max, 0, 1)
				),
				new DoubleTrajectory(
					new Stay(2000),
					new Line(0.0, 180/2.0, 0.0, 0, v_max, v_max, 10000.0, 0.0)
				)
				);
		}else{

			if(wall_mask==USE_UNKOWN_WALL_MASK){
			
			int wall = GetWallInfo();

			maze_solver.adachi.SetMap(mouse_pos_x,mouse_pos_y,wall,mouse_dir);
			}
			Dirction pre_mouse_dir = mouse_dir;
			
			Dirction next_dir = maze_solver.adachi.GetNextDirection(mouse_pos_x,mouse_pos_y,mouse_dir);
			switch(next_dir){
			case North:
				mouse_dir=North;
				mouse_pos_y++;
				break;
			case West:
				mouse_dir=West;
				mouse_pos_x--;
				break;
			case South:
				mouse_dir=South;
				mouse_pos_y--;
				break;
			case East:
				mouse_dir=East;
				mouse_pos_x++;
				break;
			}

			
			switch( (int)next_dir - (int)pre_mouse_dir ){
			case -3:
				delete trajectory;
				trajectory=new MultTrajectory(
					new Line(0.0, 35.0, 0.0, v_max, v_max, v_max, 10000.0, 0.0),
					new Slalom(90,50.0, 0, v_max, 0, 1),
					new Line(0.0, 35.0, 0.0, v_max, v_max, v_max, 10000.0, 0.0)
					);
	//			run_plan[index]=TurnLeft;
				break;
			case -2:
				delete trajectory;
				trajectory=new MultTrajectory(
					new Line(0.0, 180.0/2.0, 0.0, v_max, v_max, v_max, 4000.0, 0.0),
					new Rotate(180, 0, turn_v_max, 0, 1),
					new Line(0.0, 180/2.0, 0.0, v_max, v_max, v_max, 10000.0, 0.0)
					);
	//			run_plan[index]=Turn;
				break;
			case -1:
				delete trajectory;
				trajectory=new MultTrajectory(
					new Line(0.0, 35.0, 0.0, v_max, v_max, v_max, 10000.0, 0.0),
					new Slalom(-90,50.0, 0, v_max, 0, 1),
					new Line(0.0, 35.0, 0.0, v_max, v_max, v_max, 10000.0, 0.0)
					);
	//			run_plan[index]=TurnRight;
				break;
			case 0:
				delete trajectory;
				trajectory=new Line(0.0, 180.0, 0.0, v_max, v_max, v_max, 10000.0, 0.0);
	//			run_plan[index]=Forward;
				break;
			case 1:
				delete trajectory;
				trajectory=new MultTrajectory(
					new Line(0.0, 35.0, 0.0, v_max, v_max, v_max, 10000.0, 0.0),
					new Slalom(90,50.0, 0, v_max, 0, 1),
					new Line(0.0, 35.0, 0.0, v_max, v_max, v_max, 10000.0, 0.0)
					);
	//			run_plan[index]=TurnLeft;
				break;
			case 2:
				delete trajectory;
				trajectory=new MultTrajectory(
					new Line(0.0, 180.0/2.0, 0.0, v_max, v_max, v_max, 4000.0, 0.0),
					new Rotate(180, 0, turn_v_max, 0, 1),
					new Line(0.0, 180/2.0, 0.0, v_max, v_max, v_max, 10000.0, 0.0)
					);
	//			run_plan[index]=Turn;
				break;
			case 3:
				delete trajectory;
				trajectory=new MultTrajectory(
					new Line(0.0, 35.0, 0.0, v_max, v_max, v_max, 10000.0, 0.0),
					new Slalom(-90,50.0, 0, v_max, 0, 1),
					new Line(0.0, 35.0, 0.0, v_max, v_max, v_max, 10000.0, 0.0)
					);
	//			run_plan[index]=TurnRight;
				break;
			default:
				delete trajectory;
				trajectory=new Stop();
				break;
			}
		}
	}
	trajectory->GetTargetPosition(&target_x,&target_y,&target_theta);
	trajectory->GetTargetVelocity(&target_vx,&target_vy,&target_omega);

	float Kp_wall=0.001;
	float wall_control=Kp_wall*wall_sensor->GetError();
	if(trajectory->GetTragType()==line)target_omega+=wall_control;

	Jacobian(target_vy,target_omega,&target_velocity_r,&target_velocity_l);
	
	motorR_PID->SetTarget(target_velocity_r);
	motorL_PID->SetTarget(target_velocity_l);

	velocity_r=GetVelociryR_mm_s();
	velocity_l=GetVelociryL_mm_s();
	V_r=motorR_PID->Update(velocity_r);
	V_l=motorL_PID->Update(velocity_l);

	SetVoltageR(V_r);
	SetVoltageL(V_l);
}
*/

int count=0;
void Mouse::Interrupt_1ms(){
	encorders->Update();
	buzzer->Update();
	if(count>1){
		count=0;	
		imu->Update();
	}else{
		count++;
	}
//	localization->Update();
//	localization->GetPosition(&current_x,&current_y,&current_theta);

}

void Mouse::Interrupt_125us(){
	wall_sensor->Update();
	battery_check->Update();
	ui->SetRBLED((battery_check->GetBatteryVoltage_V()-7)/(8.4-7)*100);
	ui->Update();

}

void Mouse::Loop(){
}
