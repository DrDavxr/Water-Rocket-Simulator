# Import the required libraries.
import numpy as np
from scipy.integrate import solve_ivp
from Equations_of_Motion import Dynamics
from Forces import Forces
from Equations_of_Tank import TankFlow


def Time_Integration(state_vector, D, d, init_V_H2O, P_atm, P_max, T_init, alpha,
                     delta, m_tot, step=0.1):

    # CONSTANTS DEFINITIONS.
    A_e = np.pi*d**2 / 4
    S_ref = np.pi*D**2 / 4
    rho_w = 1000  # Density of the fluid (water) [kg/m^3]
    m_0 = rho_w * init_V_H2O + m_tot
    V_tot = 2e-3
    seconds = 0
    stop = False

    # Data Preallocation.
    h_sol = np.array([state_vector[0]])
    v_sol = np.array([state_vector[1]])
    FP_sol = np.array([state_vector[2]])
    m_sol = np.array(m_0)
    sol = np.array([])

    # Define the Tank object.
    Tank = TankFlow(D, d, P_atm, init_V_H2O)

    while not stop:

        if seconds == 0:

            # Define the initial data:
            v = v_sol[-1]
            V_H2O = init_V_H2O
            v_e = Forces.Exhaust_Velocity(V_H2O, d, D, P_max, P_atm)
            m_dot = Forces.MassFlowRate(v_e, A_e)
            T = Forces.Thrust(m_dot, v_e)
            Drag, Lift = Forces.Aerodynamic_Forces(S_ref, alpha, v)
            rho_air = P_max/287/(T_init + 273)

            # Preallocate.
            T_sol = np.array([])
            Drag_sol = np.array([])
            Lift_sol = np.array([])
            rho_air_sol  = np.array([rho_air])
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

            # Redefine density air in tank:
            rho_air = rho_air_sol[-1]
            
            # Redefine the water volume.
            V_H2O = Tank.WaterVolume(V_H2O_sol[-1], m_dot, step)

            # Redefine the Pressure of the air inside the tank.
            rho_air = Tank.DensityComputation([rho_air_sol[-1]],v_e,step)
            P_1 = Tank.TankPressure(P_max, P_max/287/(T_init + 273), rho_air)

            v_e = Forces.Exhaust_Velocity(V_H2O, d, D, P_1, P_atm)
            m_dot = Forces.MassFlowRate(v_e, A_e)

            if V_H2O <= 0:
                break
            else:
                pass

            # Calculate the remaining mass:
            m -= m_dot*step

            # Recalculate the aerodynamic forces:
            Drag, Lift = Forces.Aerodynamic_Forces(S_ref, alpha, v)

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
        v_sol = np.append(v_sol, sol[2])
        FP_sol = np.append(FP_sol, sol[1])
        rho_air_sol = np.append(rho_air_sol,rho_air)
        V_H2O_sol = np.append(V_H2O_sol, V_H2O)
        T_sol = np.append(T_sol, T)
        Drag_sol = np.append(Drag_sol, Drag)
        Lift_sol = np.append(Lift_sol, Lift)
        m_sol = np.append(m_sol, m)
        return_list = [h_sol, v_sol, FP_sol, V_H2O_sol, T_sol, Drag_sol,
                       Lift_sol, m_sol]

        seconds += step
    return return_list
