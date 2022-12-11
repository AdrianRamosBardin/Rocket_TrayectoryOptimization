# Rocket_TrayectoryOptimization
This simple Python script uses 'Casadi' a very powerful optimization library to optimize the control inputs of a TVC rocket. You can choose the initial and final conditions for the rocket and the optimizer will find the optimum control inputs.

I've always loved optimization, I feel is a really elegant way to solve problems (especially in control engineering and dynamic systems). In this case, the final conditions are met when the rocket has no velocity and height is cero, this means finding the optimum control inputs for landing the rocket given an initial condition.

This script is just a test for me to see how viable it could be to make an MPC controller with non-linear optimization in the loop. This seems much more difficult than I originally planned because this optimization takes more than 15 seconds to run. Even if I optimized the code (Changing to something like C++) and tuned the parameters to suit the necessary precision running this real-time seems like a dream.

I will probably just stick with a less cooler MPC with Jacobian linearization and LQR controller for each time step. Maybe implement the same principle but for a state observer alos, which for some reason is called Extended Kalman Filter... (I don't see the filter there, for me is a non-linear state observer xd).

NOTE: I did this just for fun so don't take this as a good approach to functional optimization and trajectory planning. I am not even close to considering myself an amateur in this field so please check everything here with great detail :)
