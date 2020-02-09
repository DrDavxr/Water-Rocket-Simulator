"""
Integration module.
"""
# Import the modules.
import numpy as np
import Water_Forces
import Equations_of_Motion as EOM
import Water_Equations as WE
import Air_Equations as AE


<<<<<<< Updated upstream
def Time_Integration(state_vector, D, d, init_V_H2O, P_atm, P_max, T_init,
                     alpha, delta, m_tot, step=0.1):
=======
def Simulation(x, state_vector, step, alpha, delta, g, D, d, m_tot, P_amb, P_1,
               T_init, V=2e-3):
>>>>>>> Stashed changes
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

    # Define the initial volume of water (first guess).
    V_H2O = x

    # Initial conditions.
    h = np.append(h, state_vector[0])
    v = np.append(v, state_vector[1])
    FP = np.append(FP, state_vector[2])
    V_air = np.append(V_air, state_vector[3])
    P_air = np.append(P_air, state_vector[4])

    rho_air = P_1 / (287*T_init)
    m_air = V_air * rho_air
    A_e = np.pi*d**2/4
    S_ref = np.pi*D**2/4

    # %% FIRST STAGE: PROPULSIVE PHASE (WATER THRUST).

    while V_H2O >= 0:

        # Compute the inputs.
        v_nozzle = Water_Forces.Exhaust_Velocity(V_H2O, d, D, P_air[-1], P_amb)
        m_dot = Water_Forces.MassFlowRate(v_nozzle, A_e)
        T = Water_Forces.Thrust(m_dot, v_nozzle)
        Drag = Water_Forces.Aerodynamic_Forces(S_ref, alpha, state_vector[1])

        # Update the state vector.
        state_vector[0] = EOM.Altitude_Computation(state_vector, step)
        state_vector[1] = EOM.FP_Computation(state_vector, step, T, m_tot,
                                             alpha, delta, Drag, g)
        state_vector[2] = EOM.Velocity_Computation(state_vector, step)
        state_vector[3] = m_air / WE.DensityComputation(state_vector[3],
                                                        v_nozzle, step, d,
                                                        m_air)
        state_vector[4] = WE.TankPressure(P_1, V, x, V_H2O)

        V_H2O = V - state_vector[3]
        m_tot -= m_dot*step

        if V_H2O >= 0:
            h = np.append(h, state_vector[0])
            v = np.append(v, state_vector[1])
            FP = np.append(FP, state_vector[2])
            V_air = np.append(V_air, state_vector[3])
            P_air = np.append(P_air, state_vector[4])
    return [h, v, FP, V_air, P_air]




     %% SECOND STAGE: PROPULSIVE PHASE (AIR THRUST).
     
     
     while P_air[-1] >= P_amb:

         # Compute the inputs.
        v_n = AE.VnozzleComp(p, P_amb, m_air/V)
        rho = AE.TankDensComp(rho, v_n, A_e, V, step)
        T = rho*v_n**2*A_e
        m_air = rho*V
        Drag = Water_Forces.Aerodynamic_Forces(S_ref, alpha, state_vector[1])
         
         # Update the state vector.
        state_vector[0] = EOM.Altitude_Computation(state_vector, step)
        state_vector[1] = EOM.FP_Computation(state_vector, step, T, m_tot,
                                             alpha, delta, Drag, g)
        state_vector[2] = EOM.Velocity_Computation(state_vector, step)
        state_vector[3] = V
        state_vector[4] = AE.TankPressComp(P_amb, A_e, v_n, V, state_vector[4], step)


         if P_air[-1] >= P_amb:
             h = np.append(h, state_vector[0])
             v = np.append(v, state_vector[1])
             FP = np.append(FP, state_vector[2])
             V_air = np.append(V_air, state_vector[3])
             P_air = np.append(P_air, state_vector[4])
            
    return [h, v, FP, V_air, P_air]

