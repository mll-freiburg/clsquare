#include "mllcartpole.h"
#include "global.h"
#include <math.h>
#include "valueparser.h"

#define THETA  0
#define THETAP 1
#define X      2
#define XP     3


MLLCartPole_SysModelParams::MLLCartPole_SysModelParams()
{
  mc = 1.000;
  mp = 0.100;
  lp = 0.225;

  bp = 0.0;

  umax  = 0.3 * 24.0;
  k2    = 0.0389;
  k3    = 0.0389;
  k3_0  = 0.006;
  Ra    = 1.23;
  Jm    = 0.0001667;
  r     = 0.02;
  n     = 4.8;

  delay = 0.0;
}

MLLCartPole_SysModel::MLLCartPole_SysModel(double _delta_t, int _num_integration_steps, 
						 const MLLCartPole_SysModelParams& _modelparams)
  : tools::SysModel(4 , _delta_t, _num_integration_steps, 1, 2)
{
  modelparams = _modelparams;

  //  modelparams.print(std::cout);
}

void MLLCartPole_SysModel::sysmodel_derivs(double t, const double *x, double *dxdt, const double *u)
{
  double uwirk;
  double uact;
  double k3_wirk = modelparams.k3;

  if (t < modelparams.delay * sysmodel_delta_t) uact = u[1];
  else uact = u[0];
  uwirk = uact * modelparams.umax;

  double wm    = x[XP] * modelparams.n / modelparams.r;

  if ( ( (modelparams.k2 / modelparams.Ra) * uwirk * wm >= 0 ) && ( fabs( (modelparams.k2 / modelparams.Ra) * uwirk) < fabs(((modelparams.k2  * k3_wirk )/ modelparams.Ra) * wm )))
    k3_wirk = modelparams.k3_0; 
  
  double fwirk = ( (modelparams.k2 / modelparams.Ra) * uwirk - ((modelparams.k2  * k3_wirk)/ modelparams.Ra) * wm ) * (modelparams.n / modelparams.r);
  


  /*
  dxdt[X]  = x[XP];
  dxdt[XP] = ( fwirk + modelparams.mp * x[THETAP] * x[THETAP] * modelparams.lp * sin(x[THETA])) 
    / (modelparams.mc + modelparams.mp + (modelparams.Jm /(modelparams.r*modelparams.r)) );
  
  dxdt[THETA]  = x[THETAP];
  dxdt[THETAP] = ( modelparams.mp * modelparams.lp * 9.81 * sin(x[THETA])
		   - modelparams.mp * modelparams.lp * dxdt[XP] * cos(x[THETA])
		   - modelparams.bp * x[THETAP])
    / ( modelparams.mp * modelparams.lp * modelparams.lp);
  */

  dxdt[X]  = x[XP];
  dxdt[XP] = ( fwirk
	       + (modelparams.mp * modelparams.lp * x[THETAP] * x[THETAP] * sin(x[THETA]) )  
	       - (modelparams.mp * cos(x[THETA]) * sin(x[THETA]) * 9.81) ) 
    / (modelparams.mc + modelparams.mp + (modelparams.Jm /(modelparams.r*modelparams.r)));
  dxdt[XP] /= ( 1 - ( (modelparams.mp * cos(x[THETA]) * cos(x[THETA])) / (modelparams.mc + modelparams.mp + (modelparams.Jm /(modelparams.r*modelparams.r))) ) );
  
  dxdt[THETA]  = x[THETAP];
  dxdt[THETAP] = ( ( 9.81 * sin(x[THETA])  - cos(x[THETA]) * dxdt[XP] ) / modelparams.lp) 
    - (modelparams.bp * x[THETAP] / ( modelparams.mp * modelparams.lp * modelparams.lp));
  
  
}


MLLCartPoleParams::MLLCartPoleParams()
{
  delta_t               = 0.020;
  num_integration_steps = 100;
}

bool MLLCartPole::get_next_plant_state(const double *plant_state, const double *action, double *next_plant_state)
{
  double modelaction[2];
  modelaction[0] = action[0] *action_scale_factor;
  modelaction[1] = lastu;

  bool res = sysmodel->sysmodel_nextState(plant_state, modelaction, next_plant_state);

  while (next_plant_state[THETA] >  M_PI ) next_plant_state[THETA] -= 2 * M_PI;
  while (next_plant_state[THETA] < -M_PI ) next_plant_state[THETA] += 2 * M_PI;

  lastu = action[0]*action_scale_factor;

  if(stop_if_turnover >0){
    if(turnover_state == 0){ // initial state
      if(fabs(next_plant_state[THETA]) >2.5)
	turnover_state = 1; 
    }
    else if(turnover_state == 1){ // has been down
      if(fabs(next_plant_state[THETA]) <0.1) // now is up
	turnover_state = 2;
    }
    else if(turnover_state == 2){ // has been  down and up
      if(fabs(next_plant_state[THETA]) >2.5) // now it's down again
	turnover_state = 3;
    }
    else if(turnover_state == 3){ // has been  down and up
      if(fabs(next_plant_state[THETA]) <0.1) // second time up
	turnover_state = 4;
    }
    else if(turnover_state == 4){ // has been  down and up
      if(fabs(next_plant_state[THETA]) >2.5) // second time down
	turnover_state = 5;
    }
    if(turnover_state >= 3 && stop_if_turnover == 1){
      // cout<<"Plant detected Turnover. STOP. "<<endl;
      turnover_state = 0;
      return false;
    }
    if(turnover_state >= 5 && stop_if_turnover > 1){
      // cout<<"Plant detected second Turnover. STOP. "<<endl;
      turnover_state = 0;
      return false;
    }
  }

  return res;
}

bool MLLCartPole::get_measurement(const double *plant_state, double *measurement)
{
  if(observation_mode == 2){
    measurement[0]=sin(plant_state[0]);
    measurement[1]=cos(plant_state[0]) - 1.0;
    for (int i=1; i<__plant_state_dim; i++) {
      measurement[1+i]=plant_state[i];
    }
  }
  else{
    for (int i=0; i<__plant_state_dim; i++) {
      measurement[i]=plant_state[i];
    }
  }
  if(observation_mode == 1){
    measurement[THETA] += M_PI;  // turn pole to bottom
    while (measurement[THETA] >  M_PI ) measurement[THETA] -= 2 * M_PI;
  }


  return true;
}



bool MLLCartPole::check_initial_state(double *initial_plant_state)
{
  lastu=0.0;
  return true;
}

bool MLLCartPole::init(int& plant_state_dim,int& measurement_dim, int& action_dim, double& delta_t, const char *fname, const char *chapter)
{
  stop_if_turnover = 0;
  turnover_state = 0;
  observation_mode = 0;
  action_scale_factor = 1.0;

  if (!read_options(fname,chapter)) return false;

  sysmodel = new MLLCartPole_SysModel(plantparams.delta_t, plantparams.num_integration_steps, modelparams);

  plant_state_dim = 4;
  measurement_dim = plant_state_dim;
  if(observation_mode == 2)
    measurement_dim = plant_state_dim +1;
  action_dim      = 1;
  delta_t         = plantparams.delta_t;

  __delta_t = delta_t;
  __plant_state_dim = plant_state_dim;
  __measurement_dim = plant_state_dim;
  

  return true;
}

void MLLCartPole::deinit()
{
  if (sysmodel !=0) delete sysmodel;
  sysmodel = 0;
}

void MLLCartPole::notify_episode_starts(){
  turnover_state = 0;
}


bool  MLLCartPole::read_options(const char* fname, const char* chapter) 
{
  ValueParser vp(fname,chapter==0?"Plant":chapter);

  vp.get("num_integration_steps", plantparams.num_integration_steps);
  vp.get("delta_t", plantparams.delta_t);
  vp.get("observation_mode", observation_mode);
  vp.get("action_scale_factor", action_scale_factor);

  vp.get("mc", modelparams.mc);
  vp.get("mp", modelparams.mp);
  vp.get("lp", modelparams.lp);

  vp.get("bp", modelparams.bp);
  
  vp.get("umax", modelparams.umax);
  vp.get("k2", modelparams.k2);
  vp.get("k3", modelparams.k3);
  vp.get("Ra", modelparams.Ra);
  vp.get("Jm", modelparams.Jm);
  vp.get("r", modelparams.r);
  vp.get("n", modelparams.n);
  vp.get("k3_0", modelparams.k3_0);
  vp.get("delay", modelparams.delay);
  vp.get("stop_if_turnover", stop_if_turnover);

  return true;
}
  
REGISTER_PLANT(MLLCartPole, "A simulated version of the Machine Learning Lab CartPole system.")
