"""
Module containing the Forces class, which will contain all the necessary
methods related with the computation of any force that appears during the
simulation of the trajectory.
"""

# Import the libraries.
import numpy as np


class Forces(object):

    def __init__(self, rho_H2O, rho_air):
        """
        Initialization method for the Forces class.

        Parameters
        ----------
        rho_H2O : FLOAT
            Density of the water in kg/m^3.
        rho_air : FLOAT
            Density of the air in kg/m^3

        Returns
        -------
        None.

        """
        self.rho_H2O = rho_H2O
        self.rho_air = rho_air

    @staticmethod
    def Exhaust_Velocity(V_H2O, d, D, P_1, P_amb, g=9.80665, rho_H2O=1000):
        """
        Compute the exhaust velocity for a given instant.

        Parameters
        ----------
        V_H2O : FLOAT
            Volume of water at a given instant [m^2].
        D : FLOAT
            Diameter of the bottle [m].
        P_1 : FLOAT
            Pressure of the air inside the tank [Pa].
        P_amb : FLOAT
            Ambient pressure [Pa].
        g : FLOAT, optional
            Gravity acceleration [m/s^2]. The default is 9.80665.
        rho_H2O : FLOAT, optional
            Water density [kg/m^3]. The default is 1000.

        Returns
        -------
        FLOAT
            Exhaust speed [m/s^2].

        """
        print(P_1)
        return np.sqrt(2*(g*((4*V_H2O)/(np.pi*D**2)) + (P_1-P_amb)/rho_H2O)/(1-(d/D)**4))

    @staticmethod
    def MassFlowRate(v_e, A_e, rho_H2O=1000.0):
        """
        Compute the mass flow rate through the nozzle.

        Parameters
        ----------
        v_e : FLOAT
            Exhaust velocity [m/s].
        A_e : FLOAT
            Exit area of the nozzle [m^2].
        rho_H2O : FLOAT, optional
            Water density [kg/m^3]. The default is 1000

        Returns
        -------
        FLOAT
            Mass flow rate through the nozzle [kg/s].

        """
        return rho_H2O * v_e * A_e

    @classmethod
    def Thrust(cls, m_dot, v_e):
        """
        Compute the instantaneous thrust.

        Parameters
        ----------
        m_dot : FLOAT
            Mass flow rate [kg/s].
        v_e : TYPE
            Exhaust velocity [m/s].
        A_e : FLOAT
            Area of the nozzle exit [m^2].

        Returns
        -------
        FLOAT
            Thrust generated [N].

        """
        return m_dot * v_e

    @classmethod
    def Aerodynamic_Forces(cls, S_ref, alpha, v, C_D=0.5, rho_air=1.225):
        """
        Compute the aerodynamic forces.

        Parameters
        ----------
        S_ref : FLOAT
            Reference surface for the aerodynamic computations [m^2].
        alpha : FLOAT
            Angle of attack of the rocket [radians].
        v : FLOAT
            True Air Speed of the rocket [m/s].
        C_D : FLOAT, optional
            Drag coefficient of the vehicle. The default is 0.5.
        rho_air : FLOAT, optional
            Density of the surrounding air [kg/m^3]. The default is 1.225.

        Returns
        -------
        Drag : FLOAT
            Drag force [N].
        Lift : FLOAT
            Lift force [N].

        """
        Drag = 0.5*rho_air*v**2*C_D*S_ref
        C_L = 2*np.pi*alpha
        Lift = 0.5*rho_air*v**2*C_L*S_ref

        return Drag, Lift
