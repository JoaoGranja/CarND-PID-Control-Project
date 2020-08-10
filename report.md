First trial was finding a good proportional gain Kp with remaining Ki and Kd with zero values. The goal was to find Kp so that the car start to oscillates.

I find Kp = 0.1 the car oscillates around the center of the lane line. After that I started increasing the Kd to smooth the oscillation. I find Kd = 1 as a good gain for the diferential gain.

I implemented the twiddle algorithm adapted to this situation which means the control gains need to be found during the simulation.