import numpy as np
from scipy.integrate import solve_ivp

# %% DESCRIPTION OF THE MODULE
"""
This file contains the Equations of the Tank of the rocket. These will
be integrated using the Runge-Kutta 4th order method.
"""


def DensityDerivative(t, y, rho, v_nozzle, d, m_0):
    drhodt = -(rho**2*v_nozzle*np.pi*d**2/4)/m_0
    return drhodt


def DensityComputation(rho, v_nozzle, step, d, m_0):
    """
    Compute the density of the air inside the tank given initial
    conditions and the time step of integration.
    """
    rho = solve_ivp(DensityDerivative, (0, step), [rho],
                    args=(rho, v_nozzle, d, m_0))
    return rho.y[0][-1]


def TankPressure(P_max, V, init_H2O, V_H2O, gamma=1.4):
    """
    Compute the air pressure inside the tank.
    """
    # p = P_max*(rho/rho_max)**self.gamma
    p = P_max * ((V - init_H2O) / (V - V_H2O))**gamma

    return p


def WaterVolume(V_init, mdot, step, rho_w=1000):
    """
    Compute the volume of water inside the tank based on an adiabatic
    expansion.
    """
    return V_init - mdot*(step/rho_w)
