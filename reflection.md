##Reflection

###Setup
The setup of this project was very simple, I only had to fill in the initialization code for the `PID` class and define the `PID::UpdateError()` and `PID::TotalError()` functions and implement them in `main.cpp`.

###Parameter tuning -Manual
Here is the meat of the project, I initially set `PID.Kp`, `PID.Ki` and `PID.Kd` to 0's and ran the simulator to confirm that the car would not steer at all. Sure enough, this is what happened.

My strategy then was to run a 'manual twiddle' algorithm by doing the following:
* 1) Increase the value of one parameter at a time, then let the car in the simulator complete one lap of the track. Meanwhile, visually observe how far the car deviates from the center of the road to get a rough sense of the cross track error (CTE)
* 2) If the increase in value of the parameter in the previous step caused the car to produce a higher CTE, reduce the value of the parameter by 2x the previously increased amount
* 3) If the increase in parameter's value in step 1) caused a reduction of the CTE, then continue to increase the value of the parameter, and this time by a larger threshold, 1.1x the previous increased value
* 4) If the resulting CTE still does not decrease, then reduce the value added to/subtracted from the parameter, so that it's 0.9x the previously changed amount

Using this approach, I noticed that a high `PID::Kp` parameter would cause the car to rapidly steer the car back and forth, while the `PID::Kd` parameter tends to smooth the steering. However, the presence of the `PID::Ki` parameter seems to make the car drive worse than before. Since the `PID::Ki` parameter is primarily used for vehicles with a steering bias and the simulator does not have such a bias, I decided to remove the `PID::Ki` parameter altogether from my parameter tuning by setting it to 0.

I also noticed that by setting upper and lower bounds on steering values to 1 and -1, respectively, helped me reduce the some of the unwanted excessive swings.

I gradully arrived at the optimal level of `PID::Kp`=0.3, `PID::Ki`=0 and `PID::Kd`=2.0. The car was driving at roughly 30 mph on average.

###Parameter tuning -Twiddle
Although the manual approach helped me find a reasonable set of parameters, I wanted to automate parameter tuning by implementing twiddle in code as Sebastian explained in the lectures.

Since the code is set up to constantly listen to the simulator, this is essentially equivalent to calling `robot.run()` continuously in the lecture. This means we cannot follow the lecture's approach to let the twiddle function call `robot.run()` without completely revamping the code, but rather the twiddle logic has to be inside `robot.run()`. This required setting up additional global parameters to keep track of where we are with the loops, and a decision needs to be made on how frequently we want to tune the parameters by comparing `best_err` and `tot_err`, or total CTE: too infrequent means the algorithm will take a long time to run, while too often would introduce unnecessary bias since the car would not have a chance to sample all characteristics of the track to get a fair `tot_err` value (e.g. certain parts of the track could be more straight than others, hence producing a smaller than average total CTE). In the end, I settled on tuning parameters every 200 measurements, which is roughly 1/6 of the total track length. Here is a code sample of how I set up twiddle:

```c++
// Global variables used for twiddle
int measurements = 0; // this increments when the simulator sends an update
double best_err = 0;
double tot_err = 0;

int param_inc = 0; //used to alternate the 2 parameters
bool second_pass = false; // switch variable keep track of whether to add dp[param_inc] to p[param_inc] or use multiplier

vector<double> p;
vector<double> dp;

int main(int argc, char *argv[]) // pass arguments in the command line
{
  uWS::Hub h;

  // initialize twiddle parameter vectors
  dp.push_back(0.1);
  dp.push_back(0.1);

  p.push_back(atof(argv[1]));
  p.push_back(atof(argv[3]));

...

h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
...
  tot_err += fabs(cte);
          
  measurements = measurements + 1;
  
  // perform twiddle at every 200 measurements
  if (measurements % 200 == 0) {
    // implement twiddle
    if (best_err == 0) {
      // initial setup for best_err, do not perform parameter tuning
      best_err = tot_err;
    } else {
      double sum = dp[0] + dp[1] + dp[2];

      if (sum > 0.001) {
        if (param_inc == 2) {
          param_inc = 0;
        }

        if (tot_err < best_err) {
          best_err = tot_err;
          dp[param_inc] *= 1.1;
          param_inc++;
          p[param_inc] += dp[param_inc];
          second_pass = false;
        } else if (second_pass) {
          p[param_inc] += dp[param_inc]; //consider removing
          dp[param_inc] *= 0.9;
          param_inc++;
          second_pass = false;
        } else {
          p[param_inc] -= 2 * dp[param_inc];
          second_pass = true;
          // do not increment param_inc;
        }

        tot_err = 0;

        pid.Kp = p[0];
        pid.Kd = p[2];
      }
    }
  }
```

I quickly realized that the starting `dp` values (1,1,1) used in lecture, i.e. how to tune each parameter in each iteration, are way too large. Using these values would cause the car to steer off the track completely and never get back on track again, rendering the rest of the tuning worthless. So I reset the `dp` values to (0.1,0.1,0.1).

Using this approach, I arrived at a solution quite close to what I got by tuning parameters manually, and my final parameters were `PID::Kp`=0.3, `PID::Ki`=0 and `PID::Kd`=2.0.