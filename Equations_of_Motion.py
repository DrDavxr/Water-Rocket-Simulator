# Import the required libraries.
import numpy as np

# %% DESCRIPTION OF THE MODULE
"""
This file contains the Equations of Motion used in the trajectory. These will
be integrated using the Runge-Kutta 4th order method.
"""


def Dynamics(state_vector, T, Drag, Lift, m, alpha, delta, g=9.80665):
    """Define the Equations of Motion of the Rocket Vehicle.The effects of
    latitude, longitude, heading and Earth's rotation have been ignored for
    this analysis.

    Parameters
    ----------
    state_vector : ARRAY
        1xN Array containing the state parameters.
    T : FLOAT/INTEGER
        Instantaneous thrust [N].
    Drag : FLOAT
        Instantaneous Drag [N].
    Lift : FLOAT
        Instantaneous Lift [N].
    m : FLOAT
        Instantaneous mass of the launcher [kg].
    alpha: FLOAT
        Angle of Attack [radians].
    delta: FLOAT
        Thrust deflection angle [radians]

    Returns
    -------
    ARRAY
        1xN Array containing the definitions of the Equations of Motion.

    """
    h = state_vector[0]  # [m]
    FP = state_vector[1]  # [radians]
    v = state_vector[2]  # [m/s]

    def Altitude_Derivative(t, y):
        """
        Function computing the altitude derivative.
        """
        dhdt = v*np.sin(FP)
        return dhdt

    def FP_Derivative(t, y):
        """
        Function computing the Flight Path Angle Derivative.
        """
        dFPdt = (T/(m*v)) * np.sin(alpha+delta) + Lift/(m*v) - g*np.cos(FP)
        return dFPdt

    def Velocity_Derivative(t, y):
        """
        Function computing the Velocity Derivative.
        """
        dvdt = (T/m)*np.cos(alpha+delta) - Drag/m - g*np.sin(FP)
        return dvdt

    function_array = [Altitude_Derivative, FP_Derivative, Velocity_Derivative]

    return function_array
