# Import the required libraries.
import numpy as np
from scipy.integrate import solve_ivp

# %% DESCRIPTION OF THE MODULE
"""
This file contains the Equations of Motion used in the trajectory. These will
be integrated using the Runge-Kutta 4th order method.
"""


def Altitude_Derivative(t, y, v, FP):
    """
    Function computing the altitude derivative.
    """
    dhdt = v*np.sin(FP)
    return dhdt


def Altitude_Computation(state_vector, step):
    Altitude = solve_ivp(Altitude_Derivative, (0, step), [state_vector[0]],
                         args=(state_vector[1], state_vector[2]))
    return Altitude.y[0][-1]


def Velocity_Derivative(t, y, FP, T, m, alpha, delta, Drag, g):
    """
    Function computing the Velocity Derivative.
    """
    dvdt = (T/m)*np.cos(alpha+delta) - Drag/m - g*np.sin(FP)
    return dvdt


def Velocity_Computation(state_vector, step, T, m, alpha, delta, Drag, g):
    FP = state_vector[2]
    Velocity = solve_ivp(Velocity_Derivative, (0, step), [state_vector[1]],
                         args=(FP, T, m, alpha, delta, Drag, g))
    return Velocity.y[0][-1]


def FP_Derivative(t, y):
    """
    Function computing the Flight Path Angle Derivative.
    """
    # dFPdt = (T/(m*v)) * np.sin(alpha+delta) + Lift/(m*v) - g*np.cos(FP)
    return 0


def FP_Computation(state_vector, step):
    FP = solve_ivp(FP_Derivative, (0, step), [state_vector[2]])
    return FP.y[0][-1]
