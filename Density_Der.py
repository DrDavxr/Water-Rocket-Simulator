import numpy as np


def DensityDerivative(t, y, rho, v_nozzle, d, D, init_m):
    drhodt = -rho**2*v_nozzle*(1-(d/D)**2)*np.pi*d**2/4
    # drhodt = -(np.pi*d**2/4 * v_nozzle * rho**2)/init_m

    return drhodt
