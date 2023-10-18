#ifndef _TRAJECTORY_HPP_
#define _TRAJECTORY_HPP_

typedef enum {
	none=-1,
	stop=0,
	line=1,
	rotate=2,
	stay=3,
	slalom=4,
	clothoid=5
}TrajType;

class Trajectory{
public:
	Trajectory();
	~Trajectory(){}
	virtual int Update(){return 0;};
	void GetTargetPosition(float* _x,float* _y, float* _theta);
	void GetTargetVelocity(float* _vx,float* _vy, float* _omega);
	void GetTargetAcceleration(float* _ax,float* _ay, float* _atheta);
	TrajType GetTragType(){return traj_type;}
protected:
	float period_s;
	float x,y,theta;
	float vx,vy,omega;
	float v0,vmax,vf;
	float ax,ay,atheta;
	TrajType traj_type;
};

class DoubleTrajectory:public Trajectory{
public:
	int Update();
	DoubleTrajectory(Trajectory* _traj1, Trajectory* _traj2);
	~DoubleTrajectory(){}
private:
	Trajectory* traj1;
	Trajectory* traj2;
	
	int phase;
};

class MultTrajectory:public Trajectory{
public:
	int Update();
	MultTrajectory(Trajectory* _traj1, Trajectory* _traj2, Trajectory* _traj3);
	~MultTrajectory(){}
private:
	Trajectory* traj1;
	Trajectory* traj2;
	Trajectory* traj3;
	
	int phase;
};

class Slalom:public Trajectory{
public:
	int Update();
	Slalom(float _theta_deg,float radius,float _v0, float _vmax, float _vf, float _a);
	~Slalom(){}
private:
	float t_s; //[s]
	float t1;
	int dir;
	float r;
};


class Rotate:public Trajectory{
public:
	int Update();
	Rotate(float _theta_deg,float _v0, float _vmax, float _vf, float _a);
	~Rotate(){}
private:
	float t_s; //[s]
	float t1;
};
class Line:public Trajectory{
public:
	int Update();
	Line(float _x, float _y, float _theta, float _v0, float _vmax, float _vf, float _a, float _j);
	~Line(){} 
private:
	float target_x,target_y,target_theta;//[mm]
	float l; //[mm]
	float vmax_x,vmax_y,omega_max; //[mm/s]
	float a,j; //[mm/s/s]
	float t_s; //[s]
	float t1,t2,t3; //[ms]
	
	float v; //[mm/s]
	float pos; //[mm]

};

class Stop:public Trajectory{
public:
	int Update();
	Stop();
	~Stop(){} 
private:
};

class Stay:public Trajectory{
public:
	int Update();
	Stay(int t_ms);
	~Stay(){} 
private:
	float t_s; //[s]
	float t1;

};

class Clothoid:public Trajectory{
public:
	int Update();
	Clothoid(float _t1,float _t2,float _t3,float _v,float _omega_max,int _cw_ccw);
	~Clothoid(){} 
private:
int cw_ccw;
float t1,t2,t3;
float v;
float omega_max;
float omega;

};


#endif //_TRAJECTORY_HPP_