"""
Trajectory simulator of the H2O rocket for the Course on Rocket Motors.
"""

# Import the libraries.
import numpy as np
from Integration import Simulation
from scipy.optimize import minimize_scalar
import matplotlib.pyplot as plt


# %% SOLVE FOR THE TRAJECTORY OF THE ROCKET.


def main(x, *args):
    # Definition of the initial state parameters.
    init_h, init_v, init_FP, V, init_P_air, step, alpha, delta, g, D, d, m_wo_H2O, P_amb, T_init = args
    init_V_air = V - x
    state_vector = [init_h, init_v, init_FP, init_V_air, init_P_air]
    m_tot = x * 1000 + m_wo_H2O
    Trajectory = Simulation(x, state_vector, step, alpha, delta, g, D, d,
                            m_tot, P_amb, init_P_air, T_init)
    return -Trajectory[0][-1]


# %% INTRODUCE THE INITIAL VALUES OF THE STATE PARAMETERS.
init_v = 1  # Initial velocity [m/s].
init_FP = np.radians(45)
init_z = 665  # Initial Altitude (Leganés) w.r.t SL [m]
R_Earth = 6371000  # Earth Radius [m]
T_0 = 288.15  # Reference Temperature for ISA [K]
P_0 = 101325  # Reference pressure for ISA [Pa]
rho_0 = 1.225  # Reference density for an ISA day [kg/m^3]

# Transform the altitude (z) into geopotential altitude (h).
init_h_ISA = (init_z)/(1+init_z/R_Earth)

# Compute the ISA temperature and pressure at Leganés.
Delta = 1 - 2.25569*1e-5*init_h_ISA  # Non-dimensional temperature ratio T/T_0
T_amb = T_0 * Delta  # Temperature at Leganés for an ISA day [K]
P_atm = P_0 * Delta**5.2561  # Pressure at Leganés for an ISA day [Pa]
rho_amb = rho_0 * Delta**4.2561  # Air density at Leganés for ISA day [kg/m^3]

# Define tank maximum pressure.
P_max = 2.83e5  # [Pa]
T_init = 30     # [ºC]

# Define Flight initial parameters.
alpha = np.radians(1)
delta = np.radians(1)

# Define Geometry Characteristics.
D = 10.2e-2  # Bottle diameter [m]
d = 8e-3  # Nozzle throat diameter [m]

# Define payload and structural mass.
m_pl = 12e-3  # Payload mass [kg]
m_str = 1.5*46.7e-3  # Structural mass [kg]

m_wo_H2O = m_pl + m_str  # Initial mass of the rocket without water.

# Redefine the initial altitude w.r.t the ground.
init_h_g = 0  # [m]

# Define the maximum volume of the bottle.
V = 2e-3

# Define the gravity.
g = 9.80655  # [m/s^2]

# Define the step of integration.
step = 0.1

# %% COMPUTE THE TRAJECTORY OF THE ROCKET.
args = (init_h_g, init_v, init_FP, V, P_max, step, alpha, delta, g,
        D, d, m_wo_H2O, P_atm, T_init)

# Obtain the optimized value of the initial volume.
solution = minimize_scalar(main, args=args, method='bounded',
                           bounds=(0.5e-3, V))

state_vector = [init_h_g, init_v, init_FP, V - solution.x, P_max]
Trajectory = Simulation(solution.x, state_vector, step, alpha, delta, g, D, d,
                        solution.x * 1000 + m_wo_H2O, P_atm, P_max, T_init)

print(f'Maximum Altitude: {Trajectory[0][-1]} m.\nV_H2O = {solution.x*1e3} L')
print(f'Time elapsed during water propulsive phase: {Trajectory[-1][0]} s.')
print(f'Time elapsed during air propulsive phase: {Trajectory[-1][1]-Trajectory[-1][0]} s.')
print(f'Time elapsed during free flight: {Trajectory[-1][-1]-Trajectory[-1][1]} s.')

t_vec = np.linspace(0, Trajectory[-1][-1], len(Trajectory[0]))
mpl = plt.figure()
plt.plot(t_vec, Trajectory[0])
plt.title('Altitude')
mpl = plt.figure()
plt.plot(t_vec, Trajectory[1])
plt.title('Speed')
mpl = plt.figure()
plt.plot(t_vec, Trajectory[2])
plt.title('Flight Path Angle')
mpl = plt.figure()
plt.plot(t_vec, Trajectory[4])
plt.title('Pressure')
