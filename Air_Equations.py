import numpy as np
from scipy.integrate import solve_ivp

# %% DESCRIPTION OF THE MODULE
"""
This file contains the Equations of the Tank of the rocket. These will
be integrated using the Runge-Kutta 4th order method.
"""

def TankRhoDer(t, y, rho_t,v_n,A_e,V):

    # V is the volume of the bottle
    drhodt = - rho_t*v_n*A_e/V;
    return drhodt


def TankDensComp(rho_t, v_n, A_e, V, step):
    """
    Compute the density of the air inside the tank given initial
    conditions and the time step of integration.
    """
    rho_t = solve_ivp(TankRhoDer, (0, step), [rho_t],
                      args=(rho_t, v_n, A_e, V))
    return rho_t.y[0][-1]


def TankPressDer(t, y, P_atm, A_e, v_n, V, p, g=1.4):

    dpdt = 0.5*((g-1)*p - (g+1)*P_atm)*v_n*A_e/V

    return dpdt


def TankPressComp(P_atm, A_e, v_n, V, p, step):
    """
    Compute the pressure of the air inside the tank given initial
    conditions and the time step of integration.
    """
    p = float(p)
    p = solve_ivp(TankPressDer, (0, step), [p], args=(P_atm, A_e, v_n, V, p))
    return p.y[0][-1]


def TempComp(p, rho, Rg = 287):

    T = Rg*p/rho

    return T


def VnozzleComp(p, P_atm, rho_t):

    v_n = np.sqrt((p - P_atm)/rho_t)
    return v_n




