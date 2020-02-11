# Water-Rocket-Simulator 🚀
Simulate the trajectory of a water rocket.
# Description.
This program simulates the ascending trajectory of a water rocket, showing to the user the different parameters, like the altitude, velocity, Flight Path angle, pressure of the air inside the tank, as well as a computation of the total time elapsed between the launching and the point of maximum altitude.
This simulator also includes an optimization process by which the optimum volume of water is computed based on the inputs.
The trajectory has 3 phases:

• First phase, corresponding to the water propulsion phase. The calculations are based on Bernoulli's equation and on the adiabatic expansion of the air inside the tank.

• Second phase, corresponding to the thrust produced by the residual air inside the tank. Computations are based on Navier-Stokes equations, together with some simplifications, like ideal air inside and outside the tank.

• Third phase, corresponding to a free flight, where no thrust is applied.

# Requirements.
The required packages are:

• `numpy`

• `scipy v.1.4.1` or greater.
