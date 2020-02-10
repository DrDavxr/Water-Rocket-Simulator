"""
Integration module.
"""
# Import the modules.
import numpy as np
from Water_Forces import Water_Forces
import Equations_of_Motion as EOM
import Water_Equations as WE
import Air_Equations as AE


def Simulation(x, state_vector, step, alpha, delta, g, D, d, m_tot, P_amb, P_1,
               T_init, V=2e-3):
    """
    Simulation of the trajectory of a water rocket.

    Parameters
    ----------
    x : FLOAT
        Initial Volume of Water [m^3].
    state_vector : ARRAY
        ARRAY containing all the parameters to be integrated.
    step : FLOAT
        Time step of integration [s].
    alpha : FLOAT
        Angle of Attack [radians].
    delta : FLOAT
        Thrust vectoring angle [radians].
    g : FLOAT
        Gravity acceleration [m/s^2].
    D : FLOAT
        Diameter of the bottle [m].
    d : FLOAT
        Nozzle throat diameter [m].
    m_tot : FLOAT
        Full mass of the rocket [kg].
    P_amb : FLOAT
        Ambient pressure [Pa].
    P_1 : FLOAT
        Initial pressure of the air inside the tank [Pa].
    T_init : FLOAT
        Initial temperature of the air inside the tank [ºC].
    V : FLOAT, optional
        Bottle volume [m^3]. The default is 2e-3.

    Returns
    -------
    list
        List containing all the necessary parameters.

    """
    """
    Structure of the state vector:
        h = state_vector[0]
        v = state_vector[1]
        FP = state_vector[2]
        V_air = state_vector[3]
        P_air = state_vector[4]
    """

    # Create empty vectors.
    h = np.array([])
    v = np.array([])
    FP = np.array([])
    V_air = np.array([])
    P_air = np.array([])
    t_stages = np.array([])  # Store the instant at which each stage stops.
    t = 0

    # Define the initial volume of water (first guess).
    V_H2O = x

    # Initial conditions.
    h = np.append(h, state_vector[0])
    v = np.append(v, state_vector[1])
    FP = np.append(FP, state_vector[2])
    V_air = np.append(V_air, state_vector[3])
    P_air = np.append(P_air, state_vector[4])

    rho_air = P_1 / (287*(T_init+273.15))
    m_air = V_air * rho_air
    A_e = np.pi*d**2/4
    S_ref = np.pi*D**2/4

    # %% FIRST STAGE: PROPULSIVE PHASE (WATER THRUST).

    while V_H2O >= 0 and state_vector[4] >= P_amb:

        # Compute the inputs.
        v_nozzle = Water_Forces.Exhaust_Velocity(V_H2O, d, D, P_air[-1], P_amb)
        m_dot = Water_Forces.MassFlowRate(v_nozzle, A_e)
        T = Water_Forces.Thrust(m_dot, v_nozzle)
        Drag, _ = Water_Forces.Aerodynamic_Forces(S_ref, alpha,
                                                  state_vector[1])

        # Update the state vector.
        state_vector[0] = EOM.Altitude_Computation(state_vector, step)
        state_vector[1] = EOM.Velocity_Computation(state_vector, step, T,
                                                   m_tot, alpha, delta, Drag,
                                                   g)
        state_vector[2] = EOM.FP_Computation(state_vector, step)
        rho_air = WE.DensityComputation(rho_air, v_nozzle, step, d, m_air)
        state_vector[3] = m_air / rho_air
        V_H2O = V - state_vector[3]
        state_vector[4] = WE.TankPressure(P_1, V, x, V_H2O)

        m_tot -= m_dot*step

        if V_H2O >= 0:
            h = np.append(h, state_vector[0])
            v = np.append(v, state_vector[1])
            FP = np.append(FP, state_vector[2])
            V_air = np.append(V_air, state_vector[3])
            P_air = np.append(P_air, state_vector[4])
            t += step
    t_stages = np.append(t_stages, t)

    # %% SECOND STAGE: PROPULSIVE PHASE (AIR THRUST).
    while state_vector[4] >= P_amb:

        # Compute the inputs.
        v_n = AE.VnozzleComp(state_vector[4], P_amb, m_air/V)
        rho_air = AE.TankDensComp(rho_air, v_n, A_e, V, step)
        T = rho_air*v_n**2*A_e
        m_air = rho_air*V
        Drag, _ = Water_Forces.Aerodynamic_Forces(S_ref, alpha,
                                                  state_vector[1])
        # Update the state vector.
        state_vector[0] = EOM.Altitude_Computation(state_vector, step)
        state_vector[1] = EOM.Velocity_Computation(state_vector, step, T,
                                                   m_tot, alpha, delta, Drag,
                                                   g)
        state_vector[2] = EOM.FP_Computation(state_vector, step)
        state_vector[3] = V
        state_vector[4] = AE.TankPressComp(P_amb, A_e, v_n, V, state_vector[4],
                                           step)
        if state_vector[4] >= P_amb:
            h = np.append(h, state_vector[0])
            v = np.append(v, state_vector[1])
            FP = np.append(FP, state_vector[2])
            V_air = np.append(V_air, state_vector[3])
            P_air = np.append(P_air, state_vector[4])
            t += step
    t_stages = np.append(t_stages, t)

    # %% THIRD STAGE: NO THRUST.

    T = 0

    while state_vector[1] >= 0:

        # Compute the inputs.
        Drag, _ = Water_Forces.Aerodynamic_Forces(S_ref, alpha,
                                                  state_vector[1])
        # Update the state vector.
        state_vector[0] = EOM.Altitude_Computation(state_vector, step)
        state_vector[1] = EOM.Velocity_Computation(state_vector, step, T,
                                                   m_tot, alpha, delta, Drag,
                                                   g)
        state_vector[2] = EOM.FP_Computation(state_vector, step)
        state_vector[3] = V
        state_vector[4] = P_amb

        if state_vector[2] >= 0:
            h = np.append(h, state_vector[0])
            v = np.append(v, state_vector[1])
            FP = np.append(FP, state_vector[2])
            V_air = np.append(V_air, state_vector[3])
            P_air = np.append(P_air, state_vector[4])
            t += step
    t_stages = np.append(t_stages, t)
    return [h, v, FP, V_air, P_air, t_stages]
