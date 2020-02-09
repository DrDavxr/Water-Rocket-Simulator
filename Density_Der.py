import numpy as np

def DensityDerivative (t,y,rho,v_nozzle,d,D):
    drhodt = -rho**2*v_nozzle*(1-(d/D)**2)*np.pi*d**2/4

    return drhodt