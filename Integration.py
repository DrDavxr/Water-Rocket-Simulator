# Import the required libraries.
import numpy as np
from scipy.integrate import solve_ivp
from Equations_of_Motion import Dynamics
import Forces


def Time_Integration(state_vector, m_dot, m_0, alpha, delta, step=0.1):
    """
    Integrate a state vector for a given step.

    Parameters
    ----------
    Engine : ENGINE OBJECT OF ENGINE MODULE
        Object containing all Engine parameters.
    Forces : FORCES OBJECT OF FORCES MODULE
        Object containing all the required forces methods.
    step : FLOAT
        Step of integration.
    state_vector : ARRAY
        Array containing the initial state parameters.
    alpha_T : FLOAT
        Thrust vectoring angle.

    Returns
    -------
    return_list : ARRAY
        Array containing all the integrated state parameters (i.e. state
        parameters at time=init_time+step)

    """
    # CONSTANTS DEFINITIONS.
    g = 9.80665  # Gravity Acceleration [m/s^2]

    # Data Preallocation.
    h_sol = np.array([state_vector[0]])
    v_sol = np.array([state_vector[1]])
    FP_sol = np.array([state_vector[2]])
    m_sol = np.array(m_0)
    sol = np.array([])


    # Define the maximum time of integration.
    max_time = Engine.tb[Engine.counter]

    for seconds in np.arange(0, max_time, step):

        if seconds == 0:

            # Define the initial data:
            v = v-sol[-1]
            T = Forces.Thrust(m_dot, v_e)
            Drag, Lift = Forces.Aerodynamic_Forces(S_ref, alpha, v)

            # Preallocate.
            T_sol = np.array([])
            Drag_sol = np.array([])
            Lift_sol = np.array([])

            # Define the mass of the current stage and the mass flow rate.
            m = m_0

        else:

            # Redefine the velocity
            v = v_sol[-1]

            # Redefine the altitude of the rocket.
            h = h_sol[-1]

            # Redefine the Flight Path Angle:
            FP = FP_sol[-1]

            # Redefine the thrust:
            T = Forces.Thrust(m_dot, v_e)

            # Calculate the remaining mass:
            m -= mdot*step

            # Recalculate the aerodynamic forces:
            Drag, Lift = Forces.Aerodynamic_Forces(S_ref, alpha, v)

            state_vector = np.array([h_sol[-1], v, FP])

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
        v_sol = np.append(v_sol, sol[1])
        FP_sol = np.append(FP_sol, sol[2])
        T_sol = np.append(T_sol, T)
        Drag_sol = np.append(Drag_sol, Drag)
        Lift_sol = np.append(Lift_sol, Lift)
        m_sol = np.append(m_sol, m)
        return_list = [h_sol, v_sol, FP_sol, T_sol, Drag_sol, Lift_sol, m_sol,
                       max_time]
    return return_list
