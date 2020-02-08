import numpy as np
from Density_Der import DensityDerivative
from scipy.integrate import solve_ivp

# %% DESCRIPTION OF THE MODULE
"""
This file contains the Equations of the Tank of the rocket. These will
be integrated using the Runge-Kutta 4th order method.
"""


class TankFlow(object):

    def __init__(self, D, d, p_atm, init_V_H2O, V=2e-3, rho_w=1000, gamma=1.4):
        """
        Initialize the Tank class.

        Parameters
        ----------
        state_vector : ARRAY
            Array containing the state parameters.
        v_nozzle : FLOAT
            Velocity at the exit of the nozzle [m/s].
        D : FLOAT
            Diameter of the bottle [m].
        d : FLOAT
            Diameter of the nozzle throat [m].
        p_atm : FLOAT
            Atmospheric (ambient) pressure [Pa].
        rho_w : FLOAT
            Water density [kg/m^3].
        gamma : FLOAT
            Specific heat ratio [-].

        Returns
        -------
        None.

        """
        self.D = D
        self.d = d
        self.p_atm = p_atm
        self.rho_w = rho_w
        self.gamma = gamma
        self.V = V
        self.V_0 = V - init_V_H2O  # Initial volume of air [m^3]

    def DensityComputation(self, state_vector, v_nozzle, step):
        """
        Compute the density of the air inside the tank given initial
        conditions and the time step of integration.
        """
        init_rho = state_vector
        rho = solve_ivp(DensityDerivative, (0, step), init_rho,
                         args=(init_rho[0], v_nozzle, self.d))
        return rho.y[0][-1]

    def TankPressure(self, P_max, rho_max, rho):
        """
        Compute the air pressure inside the tank.
        """

        p = P_max*(rho/rho_max)**self.gamma

        return p

    def WaterVolume(self, V_init, mdot, step):
        """
        Compute the volume of water inside the tank based on an adiabatic
        expansion.
        """
        return V_init - mdot*(step/self.rho_w)
    