### PID Controller

A Proportional-Integral-Derivative (PID) controller is a control loop feedback mechanism. What does this mean really? 

A PID controller is used to apply a correction to a system based on a calculated error value. The error value is the difference between a measured process variable and a desired setpoint value.

In our case the correction we apply is to set the steering angle for the vehicle. We are given the error value directly as the cross track error (CTE). The CTE is the difference between the vehicle's current position and the center of the lane we are trying to follow. In general PID controller terms our desired setpoint value is 0.

A PID controller is defined by three gain constants and three corresponding terms. Each term is based on the CTE value(s).

1. A proportional term that is based solely on the current CTE value and the proportional gain, K<sub>p</sub>. The proportional term is intended to allow the vehicle to respond directly to changes in CTE. Normally this would mean that the vehicle needs to turn to follow the road.
1. An integral term that is based on the cumulative sum of all CTE vales seen so far and the integral gain, K<sub>i</sub>. The integral term is intended to account for bias in the steering mechanism. For example, out-of-alignment wheels.
1. A derivative or differential term that is the product of the difference between the current CTE and the previous CTE and the derivative gain, K<sub>d</sub>. The differential/derivative term is intended to smooth out oscillations in the steering angle.

The formula itself is quite simple:

steering_angle = - K<sub>p</sub> * CTE - K<sub>i</sub> * SumCTE - K<sub>d</sub> * DeltaCTE

#### Proportional Term

The proportional term depends only on the the current CTE value. The proportional gain, K<sub>p</sub>, determines the ratio of the updated steering value to this current CTE value. Increasing the proportional gain will increase the response time of the PID controller to changes in CTE (i.e., when the vehicle needs to make a sharper turn to stay on the road). However, if the proportional gain value is too large the output steering angle will oscillate and the system may eventually become unstable causing the vehicle to leave the road. I saw this many times during the process of manually finding values for the gain constants.

#### Integral Term

The integral term uses the cumulative sum of all CTE values seen so far. Therefore this term increases over time unless the error term is zero. So the effect of this term is to continually drive the controller towards a CTE value of 0. In practice this term can be used to account for bias in the vehicle control and/or sensor system, such as out-of-alignment wheels, that would cause the vehicle to drift in one direction or another in spite of the steering angle.

#### Derivative Term

The differential or derivative term depends on the change in CTE from one iteration to the next. The differential gain, K<sub>d</sub>, is therefore proportional to the rate of change of the CTE value. This term acts as a damper on the effect of the proportional term. That is, if the CTE (and therefore the proportional term) changes rapidly the differential term can slow down the PID controller response to these rapid changes. The effect of this term is to dampen oscillations caused by the proportional term leding to smoother changes in steering angle and also reducing the tendency of a larger proportional gain value to cause the system to become unstable.

#### References:

[Wikipedia - PID controller](https://en.wikipedia.org/wiki/PID_controller)</br>
[National Instruments whitepaper "PID Theory Explained"](http://www.ni.com/white-paper/3782/en/)
