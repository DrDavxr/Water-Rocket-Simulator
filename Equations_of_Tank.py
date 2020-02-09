import numpy as np
from Density_Der import DensityDerivative
from scipy.integrate import solve_ivp

# %% DESCRIPTION OF THE MODULE
"""
This file contains the Equations of the Tank of the rocket. These will
be integrated using the Runge-Kutta 4th order method.
"""


class TankFlow(object):

    def __init__(self, D, d, p_atm, init_V_H2O, init_rho_air, V=2e-3,
                 rho_w=1000, gamma=1.4):
        """
        Initialize the Tank class.

        Parameters
        ----------
        D : FLOAT
            Diameter of the bottle [m].
        d : FLOAT
            Diameter of the nozzle throat [m].
        p_atm : FLOAT
            Atmospheric (ambient) pressure [Pa].
        init_V_H2O : FLOAT
            Initial volume of water [m^3]

        Returns
        -------
        None.

        """
        self.D = D
        self.d = d
        self.p_atm = p_atm
        self.rho_w = rho_w
        self.init_rho_air = init_rho_air
        self.gamma = gamma
        self.V = V
        self.V_0 = V - init_V_H2O  # Initial volume of air [m^3]
        self.m_0 = self.init_rho_air*self.V_0

    def DensityComputation(self, rho_vect, v_nozzle, step):
        """
        Compute the density of the air inside the tank given initial
        conditions and the time step of integration.
        """
        rho = solve_ivp(DensityDerivative, (0, step), rho_vect,
                        args=(rho_vect[0], v_nozzle, self.d, self.D, self.m_0))
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
