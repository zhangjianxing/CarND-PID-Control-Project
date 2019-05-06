# Write Up for PID Control Project
### The PID procedure follows what was taught in the lessons.
I use the algorithm taught in class, but limit the return value in range [-1, 1].
```c++
double total_error = -(p[0] * p_error + p[1] * d_error + p[2] * i_error);
if (total_error < -1) {return -1;}
if (total_error > 1) {return 11;}
return total_error;
```

### Describe the effect each of the P, I, D components had in your implementation.
I store tau_p, tau_i, tau_d in  `vector<double> p` in my PID class.
Each iteration, I record PID by new cte.
```c++
void PID::UpdateError(double cte) {
    d_error = cte - p_error;
    p_error = cte;
    i_error += cte;
    twiddle_error += cte*cte;
}
``` 
![](https://wikimedia.org/api/rest_v1/media/math/render/svg/cd581e5c8539ce46453574d1188bd9d52a610fe0)

The PID control scheme is named after its three correcting terms, 
whose sum constitutes the manipulated variable (MV). 
The proportional ( `e(t)` or `p_error`), integral(`d e(t)/dt` or `d_error`), 
and derivative terms(`i_error`) are summed to calculate the output of the PID 
controller. 

We need to choose proper `tau_p, tau_i, tau_d` to fit this equation. So that 
our car can drive on target trajectory with low bouncing rate. And because we
have little drifting rate. We cannot 100% relay on our output steering angle.
That is why we need to train parameters to control our car drive on target
trajectory. 

I finally choose tau_p = .2, tau_i = 1e-4, tau_d = 3.0.


### Describe how the final hyperparameters were chosen.

The final hyper parameters were chosen by twiddle algorithm. 

In the main function, I set `use_twiddle` to be `true`. The twiddle will 
start running and prints logs. Example of log:
```text
=============new round =========
twiddle_i:     0
p: 1.31833 0.0100802 2.89388e-05 
new p: 1.36938 0.0100802 2.89388e-05 

Connected!!!
twiddle_i:     0
twiddleError:     0.145218
twiddleBestError: 0.104706
dp: 0.05105 0.00417682 4.17682e-06 
p: 1.36938 0.0100802 2.89388e-05 
new p: 1.26728 0.0100802 2.89388e-05 

Connected!!!
twiddle_i:     0
twiddleError:     0.136091
twiddleBestError: 0.104706
dp: 0.05105 0.00417682 4.17682e-06 
p: 1.26728 0.0100802 2.89388e-05 
new p: 1.31833 0.0100802 2.89388e-05 
new dp: 0.045945 0.00417682 4.17682e-06 

new p: 1.31833 0.0100802 2.89388e-05 
``` 

Finally, I can choose the one with lowest twiddleError.

### The vehicle must successfully drive a lap around the track.
No tire may leave the drivable portion of the track surface. 
The car may not pop up onto ledges or roll over any surfaces that 
would otherwise be considered unsafe (if humans were in the vehicle).

