
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

In this project we use a PID controller to set the steering angle for a vehicle based on cross track error (CTE) values. At each step we receive a CTE and use that to update the PID controller, which then provides an updated steering angle.

The algorithm itself uses three error terms along with three gain constants to transform a CTE value into a steering angle.

1. A proportional term that is based solely on the current CTE value and the proportional gain, K<sub>p</sub>. The proportional term is intended to allow the vehicle to respond directly to changes in CTE. Normally this would mean that the vehicle needs to turn to follow the road.
1. An integral term that is based on the cumulative sum of all CTE vales seen so far and the integral gain, K<sub>i</sub>. The integral term is intended to account for bias in the steering mechanism. For example, out-of-alignment wheels.
1. A derivative or differential term that is the product of the difference between the current CTE and the previous CTE and the derivative gain, K<sub>d</sub>. The differential/derivative term is intended to smooth out oscillations in the steering angle.

The formula itself is quite simple:

steering_angle = - K<sub>p</sub> * CTE - K<sub>i</sub> * SumCTE - K<sub>d</sub> * DeltaCTE

The entirety of the difficulty in setting up and using a PID controller is determining how to set the three gain constants.

<small>For more detail on PID controllers, in particular as applied to this project, see [this document](doc/PID-Controller.md)</small>

### Gain Constants

There are many ways of setting values for the gain constants. One is to use the very simple twiddle algorithm. See [this document](doc/Twiddle.md) for details. Ultimately I was not able to make twiddle work with this project. In the end I started with the gain values used for the in-class demonstration and began to iteratively change each value until I obtained the desired result (i.e., the car successfully navigating one lap around the track without running off of the road)

#### Manually Setting Gain Constants

Since I was ultimately unable to make the twiddle algorithm produce usable results for this project, I reverted to the tried and true method of manually adjusting the gain constants, trying them out and repeating the process until the vehicle was able to successfully travel around the track without running off of the road.

I began with the gain values used for the simple 1D test case in the class lectures. This was a good start but I was sure I could do better. I went through several rounds of trying different gain values, mainly adjusting the proportional, K<sub>p</sub>, and derivative gain values, K<sub>d</sub>. I also added a second PID controller for the throttle value. When this was added I had to readjust the steering values to some extent.

##### Steering Controller

1. Make K<sub>p</sub> large enough that the controller can respond quickly when the vehicle needs to turn. But a value that is too large will cause excessive oscillations and may lead to the vehicle leaving the road. If K<sub>p</sub> is too large the vehicle will weave back and forth across the center of the lane and even larger values will cause these oscillations to grow until the vehicle leaves the road. In the end  In the end I settled on a value that is slightly smaller than the initial value obtained from the lecture. It became necessary to reduce this gain value when a throttle controller was added to the system as the vehicle tended to be moving faster and would therefore tend to oscillate more readily.

1. I found that K<sub>i</sub> had very little affect on the steering. But, while negligible, the effect was not zero. I think the reasoning may have something to do with the inherent delay in the response time of the vehicle to steering angle changes. This can present as a kind of bias which is exactly what this integral term is designed to account for. In any case I ended up setting this value a little smaller than the value obtained from the lecture. Setting it to 0.0 to effectively ignore the integral error term did not work well and led to excessive oscillation when this change was made in isolation.

1. Set K<sub>d</sub> to a value that allows us to set K<sub>p</sub> to a higher value without excessive oscillations. Higher values for K<sub>p</sub> are desirable for the controller to have a fast response time but, as explained above, this can lead to excessive oscillations and eventually to the vehicle leaving the road. The differential term, via the contant K<sub>d</sub>, can dampen these oscillations thus allowing the proportional gain, K<sub>p</sub>, to be set to a higher value than would otherwise be practical. I ended up setting this value a bit higher than the value obtained from the lecture. Even with a smaller proportional gain the system still needed more help to dampend oscillations in steering angle. In ths case the "help" takes the form of a higher value for the derivative gain.

##### Throttle Controller

Since I was not able to make the twiddle algorithm work effectively for this project I really had to just pick a starting point for the throttle controller gain values. I chose to start with all three values set to 0.5. This proved quite ineffective. I gradually reduced the values and ended upsetting the proportional gain to half of the original value, 0.25, and the derivative gain to 1/10 of that value, 0.025. I tried setting the integral gain to 0.0 since it seemed to me that the throttle control did not need to "know" about previous CTE values to work effectively. In the end a very small value, but non-zero, proved more effective in smoothing out the throttle and steering response (since the two are necessarily related).

I also found it necessary to add a small constant to the throttle value returned by the throttle controller. Otherwise the vehicle would either not move at all or would even try to move in reverse (negative throttle) at the start of the simulation.

### The Code

Following the directory structure in the original CarND-PID-Control-Project repository, all C++ source code for the PID controller project is in the 'src' subdirectory.

The PID class is implemented in PID.[h|cpp]. No changes were made to the class interface. I simply added implementation code.

Some changes were made to the main() function in main.cpp. Primarily to initialize the PID controller for steering and to use it to calculate a steering angle for each iteration. One addition I made was to normalize the resulting steering angle to the range [-1, 1]. I also added PID controller for the throttle. Initial tests showed that the vehicle would not move and may actually attempt to travel backwards. Therefore I added a small constant to the throttle regardless of the value returned by the PID controller.

Most of my time for this project was spent on the Twiddle class implemented in Twiddle.[h|cpp] While ultimately unsuccessful I believe the lack of success has more to do with the non-linear nature of driving a vehicle around an irregular course rather than an issue with the algorithm (explained above) and code. It was slightly challenging to implement the standard twiddle algorithm using the socket connection to the simulator as this mechanism does not easily allow a distinct and atomic "run" to obtain an error value.

In addition to code in the 'src' subdirectory there are Visual Studio solution and project files under the 'PIDControl' subdirectory. All other scripts were left intact so the project will still build in the original *nix environment.

[Project Instructions (original README file)](doc/project.md)
