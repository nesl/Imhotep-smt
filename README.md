*****************************************************************************************
            					INTRODUCTION
*****************************************************************************************

Imhotep-SMT is a Satisfiability-Modulo-Theory (SMT) based estimator for secure estimation 
under adversarial sensor attacks. Imhotep-SMT takes as input a mathematical description of
the system dynamics along with its input signals and output (sensor) measurements. The tool
can operate in two modes: (i) offline system specification, and (ii) online state 
estimation. In the first mode, Imhotep-SMT analyzes and diagnoses the mathematical 
description of the dynamical system, by characterizing its security index, that is, the 
maximum number of attacked sensors that the system can tolerate. In the second mode, 
Imhotep-SMT performs attack detection and secure state estimation at runtime.  

*****************************************************************************************
      				APIs FOR OFFLINE SOLVER CONFIGURATION
*****************************************************************************************


ImhotepSMT()
=============
smt = ImhotepSMT();

This is the constructor which constructs an object from the solver.


init()
=============
status = <objectName>.init(sys, securityIndex, safeSensors, noiseBound)

This method intializes the solver. It takes as input:
1- sys: an object of type ss which represnts the dynamics of the system
2- securityIndex: an object of type int8 describing the security index of 
the system (maximum number of sensors that can be under attack).
3- safeSensors: an array os sensors which the user ensures its safety.
4- noiseBound: an array of length p (where p = number of sensors) describing the 2-norm of 
the noise signal affecting each individual sensor.

The method initializes the solver and the internal buffer. It also checks the structure 
of the system and performs a check on the specified security index. If the user specified
security index does not match the theoretical upper bound, the init() method will return
and error message along with suggestions of how to increase the security index.

This method returns a status:
1 	--> initialization is successful
-1 	--> an error happened while initializing the solver.
0 	--> the security index is equal to zero. The solver is working as a least squares estimator

NOTE: once the solver is initialized, the solver will refuse any further calls on the
init() method.

NOTE: Even if the solver is initialized correctly, the correction of solver outputs
are not guaranteed until the checkObservabilityCondition() method is called successfully.

checkObservabilityCondition()
==============================
status = <objectName>.checkObservabilityCondition()

This method runs the combinatorial sparse observability test. This test ensures that
the system is observable after removing all combinations of (2 x securityIndex) sensors
from the system. This condition is the sufficient and necessary condition to guarantee
correctness of the results.

This method returns:
1 	--> sparse observability test passed
-1 	--> sparse observability test failed


calculateMaxEstimationError()
===============================
estimationError = <objectName>.calculateMaxEstimationError()

This method calculates the upper bound on the estimation error (which is the 2-norm of
the error between the real system state and the state estimated by Imhotep-SMT). This
estimation error depends on:
1- System dynamics
2- Noise bounds
3- Solver tolerance

NOTE: This method can not be called until the checkObservabilityCondition() is called
successfully.


*****************************************************************************************
      				APIs FOR ONLINE SECURE STATE ESTIMATION
*****************************************************************************************


addInputsOutputs()
===================
[xhat, sensorsUnderAttack] = <objectName>.addInputsOutputs(inputStream, outputStream)

This method is used in the online mode to feed the solver with the inputs and output 
streams coming from the dynamical system. This method takes two inputs:
1- inputStream: a vector of input signals fed to the system
2- outputStream: a vector of measurements collected from the system

This method returns both the estimate of the system state as well as a vector showing
the indices of sensors under attack.

NOTE: the input and output streams are assumed to be synchronized and to follow the 
sampling time of the system model. 


flushBuffers()
===============
<objectName>.flushBuffers()

This method is used to clear the internal input/output stream buffers.


