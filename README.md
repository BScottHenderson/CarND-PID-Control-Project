
# CarND-Controls-PID

## Self-Driving Car Engineer Nanodegree Program

## Project 4: PID Controller

---

This goal of this project is to implement a PID controller to set a steering angle for a vehicle. The PID algorithm itself is quite simple and easy to implement. Therefore the entire difficulty for this project is setting values for the three gain contants: proportional gain (k<sub>p</sub>), integral gain, (k<sub>i</sub>), and differential gain (k<sub>d</sub>). In additional to a PID controller for the steering angle I added a second controller used to set the throttle value. The interaction between these two controllers proved interesting. For example, the vehicle would speed up on straight stretches of road and slow down for curves, even braking as necessary for sharper curves. Overall quite an interesting project.

### Usage

1. Run the executable resulting from building the code in this repository.
1. Run the Udacity term2 simulator. Select the "Project 4: PID Controller" option.

No further action is required from the user. If all goes well the vehicle will drive around the track and remain on the road at all times.

### PID Controller

Proportional-Integral-Derivative

control loop feedback mechanism

PID controller is used to apply a correction to a system based on a calculated error value. The error value is the difference between a measured process variable and a desired setpoint value.

In our case we are given the cross track error (CTE) as the difference between the vehicle's current position and the center of the lane we are trying to follow. The correction we apply based on the PID controller is the steering angle for the vehicle. Rather than trying to minimize a difference between two values, the steering angle application attempts to minimize the CTE directly as the goal is a CTE of 0.


A simple agorithm designed to minimize an error value. In our case we're using the cross track error (CTE) which represents the distance of an autonomous vehicle from the center of a lane. The result of the PID formula is the steering angle for the vehicle.

A PID controller is defined by three constants and three corresponding terms:

1. A proportional term that is simply a product of the CTE and the "P" constant.
1. A derivative or differential term that is the product of the difference between the current CTE and the previous CTE and the "D" constant.
1. An integral term that is the product of the sum of all CTE values and the "I" constant.


The proportional term serves to respond quickly to changes in CTE, e.g., when the car is traveling around a curve.
Setting the constant for this term to higher values results in quicker response but can also lead to oscillation. Excessive oscillation can lead to the car running off the road.

The differential/derivative term is intended to smooth out oscillations in the steering angle.

The integral term is intended to account for bias in the steering mechanism. For example, out-of-alignment wheels.


The proportional term depends only on the CTE. The proportional gain, K<sub>p</sub>, determines the ratio of the updated steering value to the current CTE. Increasing the proportional gain will increase the response time of the PID controller to changes in CTE (i.e., when the vehicle needs to make a sharper turn to stay on the road). However, if the proportional gain value is too large the output steering angle will oscillate and the system may eventually become unstable causing the vehicle to leave the road. I saw this many times during the process of manually finding values for the gain constants.

The differential or derivative term depends on the change in CTE from one iteration to the next. The differential gain, K<sub>d</sub>, is therefore proportional to the rate of change of the CTE value. This term acts as a damper on the effect of the proportional term. That is, if the CTE (and therefore the proportional term) changes rapidly the differential term can slow down the PID controller response to these rapid changes. The effect of this term is to dampen oscillations caused by the proportional term.

The integral term uses the cumulative sum of all CTE values seen so far. Therefore this term increases over time unless the error term is zero. So the effect of this term is to continually drive the controller towards a CTE value of 0. In practice this term can be used to account for bias in the vehicle control and/or sensor system, such as out-of-alignment wheels that would cause the vehicle to drift in one director or another in spite of the steering angle.

References:</br>
[Wikipedia - PID controller](https://en.wikipedia.org/wiki/PID_controller)</br>
[National Instruments whitepaper "PID Theory Explained"](http://www.ni.com/white-paper/3782/en/)

### Twiddle

I attempted to implement the twiddle algorithm to determine values for the PID constants but was ultimately not successful.

In the end I used the PID constants from the Udacity lessons as a starting point and then gradually modified the values via trial and error.

For the throttle PID controller I started with all constants set to 0.5 and gradually modified the values to obtain the final constants.

I am not entirely sure why I was unable to make the twiddle algorithm work. The algorithm itself is simple enough but perhaps does not work as well with 2D data? The example used in the lectures was for a vehicle traveling in a straight line - no curves. The "real" problem, of course, has curves and the vehicle will stall or crash (either way effectively coming to permanent rest) if the CTE becomes too large. It seems likely that some other parameter estimation mechanism would work better in this case. Trial and error was interesting but quite time consuming and it is not at all clear that I ended up with an optimal solution.

Manually setting gain constants.

1. Make Kp large enough that the controller can respond quickly when the vehicle needs to turn. But a value that is too large will cause excessive oscillations and may lead to the vehicle leaving the road. If Kp is too large the vehicle will weave back and forth across the center of the lane and even larger values will cause these oscillations to grow until the vehicle leaves the road.

1. Set Kd to a value that allows us to set Kp to a higher value without excessive oscillations. Higher values for Kp are desirable for the controller to have a fast response time but, as explained above, this can lead to excessive oscillations and eventually to the vehicle leaving the road. The differential term, via the contant Kd, can dampen these oscillations thus allowing the proportional gain, Kp, to be set to a higher value than would otherwise be practical.

1.


### The Code

Following the directory structure in the original CarND-PID-Control-Project repository, all C++ source code for the PID controller project is in the 'src' subdirectory.

The PID class is implemented in PID.[h|cpp]. No changes were made to the class interface. I simply added implementation code.

Some changes were made to the main() function in main.cpp. Primarily to initialize the PID controller for steering and to use it to calculate a steering angle for each iteration. One addition I made was to normalize the resulting steering angle to the range [-1, 1]. I also added PID controller for the throttle. Initial tests showed that the vehicle would not move and may actually attempt to travel backwards. Therefore I added a small constant to the throttle regardless of the value returned by the PID controller.

Most of my time for this project was spent on the Twiddle class implemented in Twiddle.[h|cpp] While ultimately unsuccessful I believe the lack of success has more to do with the non-linear nature of driving a vehicle around an irregular course rather than an issue with the algorithm (explained above) and code. It was slightly challenging to implement the standard twiddle algorithm using the socket connection to the simulator as this mechanism does not easily allow a distinct and atomic "run" to obtain an error value.

In addition to code in the 'src' subdirectory there are Visual Studio solution and project files under the 'PIDControl' subdirectory. All other scripts were left intact so the project will still build in the original *nix environment.
