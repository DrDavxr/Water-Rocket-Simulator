"""
Trajectory simulator of the H2O rocket for the Course on Rocket Motors.
"""

# Import the libraries.
import numpy as np
from Integration import Time_Integration
from scipy.optimize import minimize
from scipy.optimize import Bounds


# %% SOLVE FOR THE TRAJECTORY OF THE ROCKET.


def main(x, *args):
    # Definition of the initial state parameters.
    P_atm, P_max, D, d, alpha, delta, init_h, init_FP, init_v = args
    state_vector = [init_h, init_FP, init_v]
    Trajectory = Time_Integration(state_vector, D, d, x, P_atm, P_max, alpha,
                                  delta)
    print(Trajectory[0][-1])
    return -Trajectory[0][-1]


# %% INTRODUCE THE INITIAL VALUES OF THE STATE PARAMETERS.
init_v = 0.1  # Initial velocity [m/s].
init_FP = np.radians(90)
init_z = 665  # Initial Altitude (Leganés) w.r.t SL [m]
R_Earth = 6371000  # Earth Radius [m]
T_0 = 288.15  # Reference Temperature for ISA [K]
P_0 = 101325  # Reference pressure for ISA [Pa]
rho_0 = 1.225  # Reference density for an ISA day [kg/m^3]
rho_H2O = 998.2  # Density of tap water [kg/m^3]

# Transform the altitude (z) into geopotential altitude (h).
init_h = (init_z)/(1+init_z/R_Earth)

# Compute the ISA temperature and pressure at Leganés.
Delta = 1 - 2.25569*1e-5*init_h  # Non-dimensional temperature ratio T/T_0
T_amb = T_0 * Delta  # Temperature at Leganés for an ISA day [K]
P_atm = P_0 * Delta**5.2561  # Pressure at Leganés for an ISA day [Pa]
rho_amb = rho_0 * Delta**4.2561  # Air density at Leganés for ISA day [kg/m^3]

# Define tank maximum pressure.
P_max = 2.83e5  # [Pa]

# Define Flight initial parameters.
alpha = np.radians(2)
delta = np.radians(0)

# Define Geometry Characteristics.
D = 10.2e-2  # Bottle diameter [m]
d = 8e-3  # Nozzle throat diameter [m]

# %% COMPUTE THE TRAJECTORY OF THE ROCKET.
args = (P_atm, P_max, D, d, alpha, delta, init_h, init_FP, init_v)

solution = minimize(main, 0.8e-3, args = args)
