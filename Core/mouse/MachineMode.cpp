#include "MachineMode.hpp"
#include "debug.hpp"
#include "Jacobian.hpp"

bool end_serch_flag;
bool return_start_flag;
float v_max;
float turn_v_max;
float Kp_wall=0.001;


Trajectory* trajectryUpdate(Mouse* mouse){
		Trajectory* traj;		
			if(return_start_flag==true){
				//next_mode=modeSelect_mode;
				end_serch_flag=true;
				return new Stop();
			}

			mouse->ui->SetLED( mouse->wall_sensor->GetWallL() <<3 |  
					   mouse->wall_sensor->GetWallFL()<<2 |
					   mouse->wall_sensor->GetWallFR()<<1 |  
					   mouse->wall_sensor->GetWallR()       );
			mouse->maze_solver->adachi.MakeStepMap(mouse->goal_pos_x,mouse->goal_pos_y,mouse->wall_mask);
	 		
			if(mouse->mouse_pos_x==mouse->goal_pos_x && mouse->mouse_pos_y==mouse->goal_pos_y ){
				mouse->goal_time++;
				if(mouse->goal_time%2==1){
					mouse->goal_pos_x = 0;
					mouse->goal_pos_y = 0;
				}else if(mouse->goal_time%2==0){
					//mouse->goal_pos_x = GOAL_X;
					//mouse->goal_pos_y = GOAL_Y;
					//mouse->wall_mask=UNUSE_UNKOWN_WALL_MASK;
					return_start_flag=true;
				}
				switch(mouse->mouse_dir){
				case North:
					mouse->mouse_dir=South;
					mouse->mouse_pos_y--;
					break;
				case West:
					mouse->mouse_dir=East;
					mouse->mouse_pos_x++;
					break;
				case South:
					mouse->mouse_dir=North;
					mouse->mouse_pos_y++;
					break;
				case East:
					mouse->mouse_dir=West;
					mouse->mouse_pos_x--;
					break;
				}
				if(return_start_flag!=true){
					mouse->buzzer->On_ms(2400,500);
					//delete trajectory;
					traj=new DoubleTrajectory(
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
					mouse->buzzer->On_ms(2400,500);
					//delete trajectory;
					traj=new MultTrajectory(
							new Line(0.0, 180.0/2, 0.0 , v_max, v_max, 0.0, 10000.0, 0.0),
							new Rotate(180, 0, turn_v_max, 0, 1),
							new Stay(500)
						);
				}
			}else{

				mouse->buzzer->On_ms(2000,100);
				int wall = mouse->GetWallInfo();
				mouse->maze_solver->adachi.SetMap(mouse->mouse_pos_x,mouse->mouse_pos_y,wall,mouse->mouse_dir);

				Dirction pre_mouse_dir = mouse->mouse_dir;
				
				Dirction next_dir = mouse->maze_solver->adachi.GetNextDirection(mouse->mouse_pos_x,mouse->mouse_pos_y,mouse->mouse_dir);
				switch(next_dir){
				case North:
					mouse->mouse_dir=North;
					mouse->mouse_pos_y++;
					break;
				case West:
					mouse->mouse_dir=West;
					mouse->mouse_pos_x--;
					break;
				case South:
					mouse->mouse_dir=South;
					mouse->mouse_pos_y--;
					break;
				case East:
					mouse->mouse_dir=East;
					mouse->mouse_pos_x++;
					break;
				default:
					mouse->buzzer->On_ms(1500,100);
					break;
				}

				
				switch( (int)next_dir - (int)pre_mouse_dir ){
				case -3:
					//delete trajectory;
					traj=new MultTrajectory(
						new Line(0.0, 35.0, 0.0, v_max, v_max, v_max, 10000.0, 0.0),
						new Slalom(90,50.0, 0, v_max, 0, 1),
						new Line(0.0, 35.0, 0.0, v_max, v_max, v_max, 10000.0, 0.0)
						);
		//			run_plan[index]=TurnLeft;
					break;
				case -2:
					//delete trajectory;
					traj=new MultTrajectory(
						new Line(0.0, 180.0/2.0, 0.0, v_max, v_max, v_max, 4000.0, 0.0),
						new Rotate(180, 0, turn_v_max, 0, 1),
						new Line(0.0, 180/2.0, 0.0, v_max, v_max, v_max, 10000.0, 0.0)
						);
		//			run_plan[index]=Turn;
					break;
				case -1:
					//delete trajectory;
					traj=new MultTrajectory(
						new Line(0.0, 35.0, 0.0, v_max, v_max, v_max, 10000.0, 0.0),
						new Slalom(-90,50.0, 0, v_max, 0, 1),
						new Line(0.0, 35.0, 0.0, v_max, v_max, v_max, 10000.0, 0.0)
						);
		//			run_plan[index]=TurnRight;
					break;
				case 0:
					//delete trajectory;
					traj=new Line(0.0, 180.0, 0.0, v_max, v_max, v_max, 10000.0, 0.0);
		//			run_plan[index]=Forward;
					break;
				case 1:
					//delete trajectory;
					traj=new MultTrajectory(
						new Line(0.0, 35.0, 0.0, v_max, v_max, v_max, 10000.0, 0.0),
						new Slalom(90,50.0, 0, v_max, 0, 1),
						new Line(0.0, 35.0, 0.0, v_max, v_max, v_max, 10000.0, 0.0)
						);
		//			run_plan[index]=TurnLeft;
					break;
				case 2:
					//delete trajectory;
					traj=new MultTrajectory(
						new Line(0.0, 180.0/2.0, 0.0, v_max, v_max, v_max, 4000.0, 0.0),
						new Rotate(180, 0, turn_v_max, 0, 1),
						new Line(0.0, 180/2.0, 0.0, v_max, v_max, v_max, 10000.0, 0.0)
						);
		//			run_plan[index]=Turn;
					break;
				case 3:
					//delete trajectory;
					traj=new MultTrajectory(
						new Line(0.0, 35.0, 0.0, v_max, v_max, v_max, 10000.0, 0.0),
						new Slalom(-90,50.0, 0, v_max, 0, 1),
						new Line(0.0, 35.0, 0.0, v_max, v_max, v_max, 10000.0, 0.0)
						);
		//			run_plan[index]=TurnRight;
					break;
				default:
					//delete trajectory;
					traj=new Stop();
					mouse->buzzer->On_ms(1500,100);
					break;
				}
			}
	return traj;
}

/////////////////////////////
SerchRun::SerchRun(Mouse* _mouse)
:MachineMode(_mouse),
idle(true),
gesture_flag(false),
no_hand_flag(false),
timer(0)
{

};
void SerchRun::Loop(){
	if(idle){
		SCI_printf("sens:%d,%d,%d,%d\r\n",mouse->wall_sensor->GetLeft(),mouse->wall_sensor->GetFrontL(),mouse->wall_sensor->GetFrontR(),mouse->wall_sensor->GetRight());
	}else{
		SCI_printf("(%d,%d)%d,%d|%d,%d\r\n",(int)mouse->mouse_pos_x,mouse->mouse_pos_y,(int)target_velocity_r,(int)target_velocity_l,(int)velocity_r,(int)velocity_l);
	}
}

void SerchRun::Init(){
	current_mode=serchRun_mode;
	next_mode=serchRun_mode;
	
	delete trajectory;
	trajectory=new Stop();

	mouse->motorR_PID->Reset();
	mouse->motorL_PID->Reset();
	
	v_max=400;
	turn_v_max=300;
	mouse->mouse_pos_x=0;
	mouse->mouse_pos_y=0;
	mouse->mouse_dir=North;
	mouse->goal_pos_x=GOAL_X;
	mouse->goal_pos_y=GOAL_Y;
	mouse->goal_time=0;
	mouse->wall_mask=USE_UNKOWN_WALL_MASK;	
	
	idle=true;
	end_serch_flag=false;
	return_start_flag=false;
}


void SerchRun::Interrupt_1ms(){
	if(idle){
		static int gesture_sensor_th=250;
		if((mouse->wall_sensor->GetFrontR() > gesture_sensor_th || mouse->wall_sensor->GetFrontL() > gesture_sensor_th )){
			gesture_flag=true;
		}
		if((no_hand_flag==false) && gesture_flag && (mouse->wall_sensor->GetFrontR()< gesture_sensor_th && mouse->wall_sensor->GetFrontL()< gesture_sensor_th )){
			no_hand_flag=true;
			mouse->buzzer->On_ms(4000,40);
		}
		if(no_hand_flag)timer++;
		if(timer>1000){
			idle=false;
			//mouse->trajectory=new Stop();
			mouse->ui->SetLED(15);
			delete trajectory;
			trajectory=new Line(0.0, 180.0/2.0, 0.0, 0.0, v_max, v_max, 4000.0, 0.0);
			mouse->mouse_pos_y++;
			//trajectory=new Rotate(90, 0, 100.0, 0, 1);
			trajectory->Update();

		}
	}else{
		if(trajectory->Update()){
			delete trajectory;
			trajectory = trajectryUpdate(mouse);

		}else{

			trajectory->GetTargetPosition(&target_x, &target_y, &target_theta);
			trajectory->GetTargetVelocity(&target_vx,&target_vy,&target_omega);

			float wall_control=Kp_wall*mouse->wall_sensor->GetError();
			if(trajectory->GetTragType()==line)target_omega+=wall_control;

			Jacobian(target_vy,target_omega,&target_velocity_r,&target_velocity_l);
			
			mouse->motorR_PID->SetTarget(target_velocity_r);
			mouse->motorL_PID->SetTarget(target_velocity_l);
	//		mouse->motorR_PID->SetTarget(200);
	//		mouse->motorL_PID->SetTarget(200);

			velocity_r=mouse->encorders->GetVelociryR_mm_s();
			velocity_l=mouse->encorders->GetVelociryL_mm_s();
			V_r=mouse->motorR_PID->Update(velocity_r);
			V_l=mouse->motorL_PID->Update(velocity_l);

			mouse->motors->SetVoltageR(V_r);
			mouse->motors->SetVoltageL(V_l);
		}

	}
	if(end_serch_flag){
		SCI_printf("deleat\r\n");		
		delete trajectory;
		next_mode=modeSelect_mode;
	}
}

/////////////////////////////
FastRun::FastRun(Mouse* _mouse):
MachineMode(_mouse),
idle(true),
gesture_flag(false),
no_hand_flag(false),
timer(0)
{
};
void FastRun::Loop(){
	if(idle){
		SCI_printf("sens:%d,%d,%d,%d\r\n",mouse->wall_sensor->GetLeft(),mouse->wall_sensor->GetFrontL(),mouse->wall_sensor->GetFrontR(),mouse->wall_sensor->GetRight());
	}else{
		SCI_printf("(%d,%d)%d,%d|%d,%d\r\n",(int)mouse->mouse_pos_x,mouse->mouse_pos_y,(int)target_velocity_r,(int)target_velocity_l,(int)velocity_r,(int)velocity_l);
	}
}
void FastRun::Init(){
	current_mode=fastRun_mode;
	next_mode=fastRun_mode;

	delete trajectory;
	trajectory=new Stop();

	mouse->motorR_PID->Reset();
	mouse->motorL_PID->Reset();
	
	v_max=600;
	turn_v_max=200;
	mouse->mouse_pos_x=0;
	mouse->mouse_pos_y=0;
	mouse->mouse_dir=North;
	mouse->goal_pos_x=GOAL_X;
	mouse->goal_pos_y=GOAL_Y;
	mouse->goal_time=0;
	mouse->wall_mask=UNUSE_UNKOWN_WALL_MASK;	
	
	idle=true;
	end_serch_flag=false;
	return_start_flag=false;
}

void FastRun::Interrupt_1ms(){
	if(idle){
		static int gesture_sensor_th=250;
		if((mouse->wall_sensor->GetFrontR() > gesture_sensor_th || mouse->wall_sensor->GetFrontL() > gesture_sensor_th )){
			gesture_flag=true;
		}
		if((no_hand_flag==false) && gesture_flag && (mouse->wall_sensor->GetFrontR()< gesture_sensor_th && mouse->wall_sensor->GetFrontL()< gesture_sensor_th )){
			no_hand_flag=true;
			mouse->buzzer->On_ms(4000,40);
		}
		if(no_hand_flag)timer++;
		if(timer>1000){
			idle=false;
			//mouse->trajectory=new Stop();
			mouse->ui->SetLED(15);
			delete trajectory;
			trajectory=new Line(0.0, 180.0/2.0, 0.0, 0.0, v_max, v_max, 8000.0, 0.0);
			mouse->mouse_pos_y++;
			//trajectory=new Rotate(90, 0, 100.0, 0, 1);
		}
	}else{
		if(trajectory->Update()){
			delete trajectory;
			trajectory = trajectryUpdate(mouse);

		}else{

			trajectory->GetTargetPosition(&target_x, &target_y, &target_theta);
			trajectory->GetTargetVelocity(&target_vx,&target_vy,&target_omega);

			float wall_control=Kp_wall*mouse->wall_sensor->GetError();
			if(trajectory->GetTragType()==line)target_omega+=wall_control;

			Jacobian(target_vy,target_omega,&target_velocity_r,&target_velocity_l);
			
			mouse->motorR_PID->SetTarget(target_velocity_r);
			mouse->motorL_PID->SetTarget(target_velocity_l);
	//		mouse->motorR_PID->SetTarget(200);
	//		mouse->motorL_PID->SetTarget(200);

			velocity_r=mouse->encorders->GetVelociryR_mm_s();
			velocity_l=mouse->encorders->GetVelociryL_mm_s();
			V_r=mouse->motorR_PID->Update(velocity_r);
			V_l=mouse->motorL_PID->Update(velocity_l);

			mouse->motors->SetVoltageR(V_r);
			mouse->motors->SetVoltageL(V_l);
		}

	}
	if(end_serch_flag){
		SCI_printf("deleat\r\n");		
		delete trajectory;
		next_mode=modeSelect_mode;
	}
}

/////////////////////////////
LowBattery::LowBattery(Mouse* _mouse):MachineMode(_mouse){
	current_mode=lowBattery_mode;
	next_mode=lowBattery_mode;
};
void LowBattery::Loop(){
	SCI_printf("Battery Voltege is Low! %d\n\r",(int)(1000*mouse->battery_check->GetBatteryVoltage_V()));
	mouse->motors->SetVoltageR(0);
	mouse->motors->SetVoltageL(0);
};
void LowBattery::Init(){
	mouse->motors->SetVoltageR(0);
	mouse->motors->SetVoltageL(0);
};
void LowBattery::Interrupt_1ms(){
	mouse->motors->SetVoltageR(0);
	mouse->motors->SetVoltageL(0);

	mouse->buzzer->SetFrequency(2000);
	mouse->buzzer->On();
};

/////////////////////////////
ModeSelect::ModeSelect(Mouse* _mouse):MachineMode(_mouse){
	current_mode=modeSelect_mode;
	next_mode=modeSelect_mode;
	mode_val=0;
}
void ModeSelect::Init(){
	sw1=mouse->ui->GetSW1();
	pre_sw1=sw1;
	sw2=mouse->ui->GetSW2();
	pre_sw2=sw2;
	sw3=mouse->ui->GetSW3();
	pre_sw3=sw3;
	SCI_printf("%d\n\r",mode_val);
	

}
void ModeSelect::Loop(){
	//SCI_printf("%d\n\r",mode_val);
}
void ModeSelect::Interrupt_1ms(){
	mouse->motors->SetVoltageR(0);
	mouse->motors->SetVoltageL(0);

	pre_sw1=sw1;
	pre_sw2=sw2;
	pre_sw3=sw3;
	sw1=mouse->ui->GetSW1();
	sw2=mouse->ui->GetSW2();
	sw3=mouse->ui->GetSW3();
	
	if(sw1<pre_sw1){
		if(mode_val<15){
			mouse->buzzer->On_ms(4000,100);
			mode_val++;
			SCI_printf("%d\n\r",mode_val);
		}
	}
	if(sw3<pre_sw3){
		if(mode_val>0){
			mouse->buzzer->On_ms(3000,100);
			mode_val--;
			SCI_printf("%d\n\r",mode_val);
		}
	}
	if(sw2<pre_sw2){

		switch(mode_val){
		case 0:
			next_mode=serchRun_mode;
			mouse->buzzer->On_ms(2000,500);
			break;
		case 1:
			next_mode=fastRun_mode;
			mouse->buzzer->On_ms(2000,500);
			break;
		case 2:
			next_mode=debug_mode;
			mouse->buzzer->On_ms(2000,500);
			break;
		case 3:
			next_mode=parameterSetting_mode;
			mouse->buzzer->On_ms(2000,500);
			break;
		case 4:
			next_mode=sensorCheck_mode;
			mouse->buzzer->On_ms(2000,500);
			break;
		case 5:
			break;
		case 6:
			break;
		case 7:
			break;
		}
	}
	mouse->ui->SetLED(mode_val);

}

/////////////////////////////
SensorCheck::SensorCheck(Mouse* _mouse):MachineMode(_mouse){
	current_mode=sensorCheck_mode;
	next_mode=sensorCheck_mode;
}
void SensorCheck::Loop(){
	SCI_printf("\x1b[2J");		//クリアスクリーン[CLS]
	SCI_printf("\x1b[0;0H");	//カーソルを0,0に移動
	SCI_printf("%d,%d,%d,%d,%d\r\n %5d,%5d\r\n",
		(int)(1000*mouse->battery_check->GetBatteryVoltage_V()),
		mouse->wall_sensor->GetLeft(),
		mouse->wall_sensor->GetFrontL(),
		mouse->wall_sensor->GetFrontR(),
		mouse->wall_sensor->GetRight(),
		(int)(mouse->encorders->GetVelociryL_mm_s()),
		(int)(mouse->encorders->GetVelociryR_mm_s())
		);
}

void SensorCheck::Init(){
	SCI_printf("Start Sensor Check mode!\n\r");
}
void SensorCheck::Interrupt_1ms(){

	if(mouse->ui->GetSW1()==0){
		mouse->buzzer->On_ms(3000,100);
		next_mode=modeSelect_mode;
	}
};

/////////////////////////////
Debug::Debug(Mouse* _mouse):MachineMode(_mouse){
};
void Debug::Loop(){
}
void Debug::Init(){
	current_mode=debug_mode;
	next_mode=debug_mode;

	SCI_printf("Start debud mode!\n\r");
	v_max=550;
	turn_v_max=300;
	
	idle=true;
	end_serch_flag=false;
	return_start_flag=false;
	gesture_flag=false;


}
void Debug::Interrupt_1ms(){
	if(idle){
		static int gesture_sensor_th=250;
		if((mouse->wall_sensor->GetFrontR() > gesture_sensor_th || mouse->wall_sensor->GetFrontL() > gesture_sensor_th )){
			gesture_flag=true;
		}
		if((no_hand_flag==false) && gesture_flag && (mouse->wall_sensor->GetFrontR()< gesture_sensor_th && mouse->wall_sensor->GetFrontL()< gesture_sensor_th )){
			no_hand_flag=true;
			mouse->buzzer->On_ms(4000,40);
		}
		if(no_hand_flag)timer++;
		if(timer>1000){
			idle=false;
			//mouse->trajectory=new Stop();
			mouse->ui->SetLED(15);
			delete trajectory;
			trajectory=new DoubleTrajectory(
				new MultTrajectory(
						new Line(0.0, 180/2+35.0, 0.0, 0, v_max, v_max, 10000.0, 0.0),
						new Slalom(-90,50.0, 0, v_max, 0, 1),
						new Line(0.0, 180/2+35.0, 0.0, v_max, v_max, 0, 10000.0, 0.0)
						),
				new Stay(2000));
			mouse->mouse_pos_y++;
			//trajectory=new Rotate(90, 0, 100.0, 0, 1);
		}
	}else{
		if(trajectory->Update()){
			delete trajectory;
			trajectory = new Stop();
			end_serch_flag=true;

		}else{

			trajectory->GetTargetPosition(&target_x, &target_y, &target_theta);
			trajectory->GetTargetVelocity(&target_vx,&target_vy,&target_omega);

			float wall_control=Kp_wall*mouse->wall_sensor->GetError();
			if(trajectory->GetTragType()==line)target_omega+=wall_control;

			Jacobian(target_vy,target_omega,&target_velocity_r,&target_velocity_l);
			
			mouse->motorR_PID->SetTarget(target_velocity_r);
			mouse->motorL_PID->SetTarget(target_velocity_l);
	//		mouse->motorR_PID->SetTarget(200);
	//		mouse->motorL_PID->SetTarget(200);

			velocity_r=mouse->encorders->GetVelociryR_mm_s();
			velocity_l=mouse->encorders->GetVelociryL_mm_s();
			V_r=mouse->motorR_PID->Update(velocity_r);
			V_l=mouse->motorL_PID->Update(velocity_l);

			mouse->motors->SetVoltageR(V_r);
			mouse->motors->SetVoltageL(V_l);
		}

	}
	if(end_serch_flag){
		SCI_printf("deleat\r\n");		
		delete trajectory;
		next_mode=modeSelect_mode;
	}

}

