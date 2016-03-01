******************************
* AR.Drone Quadrotor Simulator
*
* Property of PRECISE Center, UPenn
* Authors: Nicola Bezzo (nicbezzo@seas.upenn.edu), 
*          Yanwei Du (duyanwei@seas.upenn.edu), 
*          Kartik Mohta (kmohta@seas.upenn.edu)
* Date: 03/10/2015
*
******************************

Instructions

1. define way points in "def_waypts.m" function;

2. set "noise_flag" variable in "runsim.m" function to add/remove noise;

3. run "runsim.m" function to start simulation;

4. press "q" anytime to quit the simulation and visualize pose data; 

5. you can tune the PID values in "nanoplus.m" function;


***************************************************************************

# "nanoplus.m", "quadEOM_readonly.m" functions will help you understand 
   the model of AR.Drone quadrotor; 

# "LinearController.m" function defines the controller in the simulation;

# modify "add_noise.m" to change noise type and value;

# modify "v_max" variable in "trajectory_generator.m" function to fly faster;
