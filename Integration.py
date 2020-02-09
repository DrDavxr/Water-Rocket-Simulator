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
<<<<<<< Updated upstream

    # CONSTANTS DEFINITIONS.
    A_e = np.pi*d**2 / 4
    S_ref = np.pi*D**2 / 4
    rho_w = 1000  # Density of the fluid (water) [kg/m^3]
    m_0 = rho_w * init_V_H2O + m_tot
    # V_tot = 2e-3
    seconds = 0
    stop = False

    # Data Preallocation.
    h_sol = np.array([state_vector[0]])
    FP_sol = np.array([state_vector[1]])
    v_sol = np.array([state_vector[2]])
    m_sol = np.array(m_0)
    sol = np.array([])

    # Define the Tank object.
    rho_air = P_max/(287*(T_init + 273))  # Initial air density
    Tank = TankFlow(D, d, P_atm, init_V_H2O, rho_air)

    while not stop:

        if seconds == 0:

            # Define the initial data:
            v = v_sol[-1]
            V_H2O = init_V_H2O
            v_e = Forces.Exhaust_Velocity(V_H2O, d, D, P_max, P_atm)
            m_dot = Forces.MassFlowRate(v_e, A_e)
            T = Forces.Thrust(m_dot, v_e)
            Drag, Lift = Forces.Aerodynamic_Forces(S_ref, alpha, v)

            # Preallocate.
            T_sol = np.array([])
            Drag_sol = np.array([])
            Lift_sol = np.array([])
            rho_air_sol = np.array([])
            V_H2O_sol = np.array([])

            # Define the initial mass
            m = m_0

        else:

            # Redefine the velocity
            v = v_sol[-1]

            # Redefine the altitude of the rocket.
            h = h_sol[-1]

            # Redefine the Flight Path Angle:
            FP = FP_sol[-1]

            # Redefine the water volume.
            V_H2O = Tank.WaterVolume(V_H2O_sol[-1], m_dot, step)

            # Redefine the Pressure of the air inside the tank.
            rho_air = Tank.DensityComputation([rho_air_sol[-1]], v_e, step)
            P_1 = Tank.TankPressure(P_max, P_max/287/(T_init + 273), rho_air)
            print(P_1)

            if V_H2O <= 0:
                break
            else:
                pass

            v_e = Forces.Exhaust_Velocity(V_H2O, d, D, P_1, P_atm)
            m_dot = Forces.MassFlowRate(v_e, A_e)

            # Calculate the remaining mass:
            m -= m_dot*step

            # Recalculate the aerodynamic forces:
            Drag, Lift = Forces.Aerodynamic_Forces(S_ref, alpha, v)

            # Redefine the thrust.
            T = Forces.Thrust(m_dot, v_e)

            state_vector = np.array([h, FP, v])

        sol = np.array([])

        # Obtain the outputs from the differential equations.
        EOM = Dynamics(state_vector, T, Drag, Lift, m, alpha, delta)

        n = len(EOM)
        for eq_index in np.arange(0, n):
            y0_state = np.array([state_vector[eq_index]])
            fun = lambda t, y: EOM[eq_index](t, y)
            instant_sol = solve_ivp(fun, (seconds, seconds+step), y0_state)
            sol = np.append(sol, instant_sol.y[0][-1])

        # Retrieve the obtained solutions.
        h_sol = np.append(h_sol, sol[0])
        FP_sol = np.append(FP_sol, sol[1])
        v_sol = np.append(v_sol, sol[2])
        rho_air_sol = np.append(rho_air_sol, rho_air)
        V_H2O_sol = np.append(V_H2O_sol, V_H2O)
        T_sol = np.append(T_sol, T)
        Drag_sol = np.append(Drag_sol, Drag)
        Lift_sol = np.append(Lift_sol, Lift)
        m_sol = np.append(m_sol, m)
        return_list = [h_sol, v_sol, FP_sol, V_H2O_sol, T_sol, Drag_sol,
                       Lift_sol, m_sol]

        seconds += step
    return return_list
=======
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

    # %% SECOND STAGE: PROPULSIVE PHASE (AIR THRUST).
    # while P_air[-1] >= P_amb:

    #     # Compute the inputs.
    #     v_nozzle = Forces.Exhaust_Velocity(V_H2O, d, D, P_air[-1], P_amb)
    #     m_dot = Forces.MassFlowRate(v_nozzle, A_e)
    #     T = Forces.Thrust(m_dot, v_nozzle)
    #     Drag = Forces.Aerodynamic_Forces(S_ref, alpha, state_vector[1])

    #     # Update the state vector.
    #     state_vector[0] = EOM.Altitude_Computation(state_vector, step)
    #     state_vector[1] = EOM.FP_Computation(state_vector, step, T, m_tot,
    #                                          alpha, delta, Drag, g)
    #     state_vector[2] = EOM.Velocity_Computation(state_vector, step)
    #     state_vector[3] = m_air / WE.DensityComputation(state_vector[3],
    #                                                     v_nozzle, step, d,
    #                                                     m_air)
    #     state_vector[4] = WE.TankPressure(P_1, V, x, V_H2O)

    #     V_H2O = V - state_vector[3]
    #     m_tot -= m_dot*step

    #     if V_H2O >= 0:
    #         h = np.append(h, state_vector[0])
    #         v = np.append(v, state_vector[1])
    #         FP = np.append(FP, state_vector[2])
    #         V_air = np.append(V_air, state_vector[3])
    #         P_air = np.append(P_air, state_vector[4])
>>>>>>> Stashed changes
