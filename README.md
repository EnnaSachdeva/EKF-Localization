# EKF-Localization

The files description are as following:

1. runMotionModel.m - updates the robot’s state, using the control command, and the motion model
2. computeJacobianState.m - computes the jacobian of the motion model
with respect to the state
3. computeJacobianControl.m - computes the jacobian of the motion model
with respect to the control
4. getMeasurements.m - fires the sensor and receives measurements
5. computeJacobianObs.m - computes the jacobian of the observation with
respect to the state
6. runIncrementalEKF.m - the main file which implements the EKF localization in the incremental update mode
7. runBatchEKF.m - the main file which implements the EKF localization in
the batch update mode
8. perfAnalysis.m - computes the localization error, to analyze the performance of the EKF
9. sane mvnrnd.m - sample sanely from mvnrnd (refer to the previous assignment for details)
10. get error ellipse.m - plots the uncertainty ellipse for a 2D gaussian


Basic workflow:
Two main scripts are: runIncrementalEKF.m  and runBatchEKF.m
The EKF Cycle runs as following:
An EKF can be viewed as a two-step process - a state prediction step, and a
state update step. Here, we assume that the robot’s state is characterized by
a multivariate gaussian distribution. Hence, a state is represented by a mean
vector and a covariance matrix.

Prediction Step:
Robot’s do not execute their controls perfectly. We model this by incorporating
noise in the robots motion. In our code, we have the robot’s state stored in the
structure robot.pose. This struct has three elements x, y, and theta, which
store the robot’s (x; y) location and the heading θ respectively.
To simulate a discrete time step, we run a loop from 1 to T, where T is the
time limit upto which EKF is to be run. In each time step, the robot does the
following:
• Run the motion model to predict the next state
• Update the state covariance using the jacobians of the motion model
• Run the prediction step 

To run the motion model, we first take the next control, and corrupt it by noise
(that noise must be sanely sampled from the control noise distribution). However, the robot is unaware of what exact noise occured in the control. According
to the robot, it has executed the ideal control. Use the motion model and the
ideal control to update the robot’s state. Intuitively speaking, this state is where
the robot thinks it is. For the purposes of measuring localization error and thus
analyzing performance, we would also want to store where the robot actually is.
This can be again obtained by keeping track of all true states in another
variable.
Once the robot has predicted its new state, we linearize the motion model
about the mean of the estimate. This is done since the kalman filter’s update
step works only for linear models. We linearize the motion model by Taylor
series expansion. To this end, implement the functions
that compute the jacobian of the motion model with respect to the state and
the control.
Using the computed jacobians, update the covariance of the robot’s state. This
wraps up the prediction step of the EKF.

Update Step:
To update the robot’s estimate of state, we make use of measurements (a.k.a.
observations or sensor readings). We assume that the robot is equipped with a
range-bearing sensor that can measure all landmarks within sensing range. Its
uncertainty is characterized by a gaussian with zero-mean and some covariance
matrix R. We fire the sensor and get the true sensor measurements, i.e., we get the range
and bearing measurements to all landmarks within a stipulated sensor radius.
We then corrupt the observations by the observation covariance. These measurements can intuitively be thought of as what the robot actually sees.
In the above procedure, note that the measurements are obtained from the true
pose of the robot, not from where the robot thinks it is.
We now compute expected measurements, i.e., what the robot expects to see from
where the robot thinks it is. This serves as a second estimate of state, which can
be used to refine the estimate obtained from the motion model.
Run the EKF update equations. At a coarse level, the update involves,
computing the Kalman gain, using the jacobian of the observation model, and
then computing the updated mean and covariance (using the Kalman gain).
Note that there are two primary ways in which updates can be done, viz. incremental and batch modes. In the incremental mode, you perform the update on a per-landmark basis, i.e., you run a loop for each observation (inside the
loop that runs for every time step) and carry out Kalman gain computation,
etc. inside the loop. In the batch update, you compute a joint update for all
landmarks visible in a particular time step. 

After this, run the updataSimulation.m script to update the GUI.

Three different motion models have been implemented here:
1. alpha_T_beta : considers the controls as rotation followed by translation followed by rotation.
2. T_phi : considers the controls as translation followed by rotation.
3. T_T : considers the controls as translation along x followed by translation along y. Under this motion model, we assume that the robot is holonomic (in a simplistic sense, the robot does not have to rotate to move between
any two points in the plane). 
