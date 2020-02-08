"""
Trajectory simulator of the H2O rocket for the Course on Rocket Motors.
"""

# Import the libraries.
import numpy as np
import Forces


# %% INTRODUCE THE INITIAL VALUES OF THE STATE PARAMETERS.
init_v = 1  # Initial velocity [m/s].
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
P_amb = P_0 * Delta**5.2561  # Pressure at Leganés for an ISA day [Pa]
rho_amb = rho_0 * Delta**4.2561  # Air density at Leganés for ISA day [kg/m^3]

# %% SOLVE FOR THE TRAJECTORY OF THE ROCKET.


def main(x, *args):
    # Definition of the initial state parameters.
    P_amb, P_max, D, D_e = args
    A_e = np.pi*D_e**2 / 4
    init_H = (4*x) / (np.pi*D**2)
    init_v_e = Forces.Exhaust_Velocity(x, D, P_max, P_amb)
    m_dot = Forces.MassFlowRate(init_v_e, A_e)