# Import the required libraries.
import numpy as np
from scipy.integrate import solve_ivp

# %% DESCRIPTION OF THE MODULE
"""
This file contains the Equations of Motion used in the trajectory. These will
be integrated using the Runge-Kutta 4th order method.
"""


def Altitude_Derivative(t, y, state_vector):
    """
    Function computing the altitude derivative.
    """
    v = state_vector[1]
    FP = state_vector[2]
    dhdt = v*np.sin(FP)
    return dhdt


def Altitude_Computation(state_vector, step):
    Altitude = solve_ivp(Altitude_Derivative, (0, step), [state_vector[0]],
                         args=state_vector)
    return Altitude


def Velocity_Derivative(t, y, state_vector, T, m, alpha, delta, Drag, g):
    """
    Function computing the Velocity Derivative.
    """
    FP = state_vector[2]
    dvdt = (T/m)*np.cos(alpha+delta) - Drag/m - g*np.sin(FP)
    return dvdt


def Velocity_Computation(state_vector, step, T, m, alpha, delta, Drag, g):
    Velocity = solve_ivp(Velocity_Derivative, (0, step), [state_vector[1]],
                         args=(state_vector, T, m, alpha, delta, Drag, g))
    return Velocity


def FP_Derivative(t, y):
    """
    Function computing the Flight Path Angle Derivative.
    """
    # dFPdt = (T/(m*v)) * np.sin(alpha+delta) + Lift/(m*v) - g*np.cos(FP)
    return 0


def FP_Computation(state_vector, step):
    FP = solve_ivp(FP_Derivative, (0, step), [state_vector[2]])
    return FP
