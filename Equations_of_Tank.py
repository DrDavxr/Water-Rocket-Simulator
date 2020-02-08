import numpy as np
from Density_Der import DensityDerivative
from scipy.integrate import solve_ivp

# %% DESCRIPTION OF THE MODULE
"""
This file contains the Equations of the Tank of the rocket. These will
be integrated using the Runge-Kutta 4th order method.
"""

class TankFlow(object):
    
    def __init__(self,state_vector, v_nozzle, D, d, p_atm, rho_w, gamma):
        self.state_vector = state_vector
        self.v_nozzle = v_nozzle
        self.D = D
        self.d = d
        self.p_atm = p_atm
        self.rho_w = rho_w
        self.gamma = gamma
        


    
    """Define the Equations of evolution of air inside the tank,
    assuming adiabatic expansion.

    Parameters
    ----------
    state_vector : ARRAY
        1xN Array containing the state parameters.
    rho_air: FLOAT
        Density of air [kg/m3].
    v_nozzle : FLOAT
        Velocity of water at the nozzle [m/s].
    d : FLOAT
        Diameter of throat [m].

    Returns
    -------
    ARRAY
        1xN Array containing the definitions of the Equations of Motion.
        
    """

    def DensityComp(self,step):
        rho = self.state_vector
        rho2 = solve_ivp(DensityDerivative,(0,step),rho,args=(rho,self.v_nozzle,self.d))
        return rho2
    
    def PresTank(self):
        
        p = self.p_atm + 0.5*self.rho_w*self.v_nozzle**2*(1 - (self.d/self.D)**4)
        
        return p
    
        
        