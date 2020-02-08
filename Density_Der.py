import numpy as np

def DensityDerivative (t,y,rho,v_nozzle,d):
    drhodt = rho**2*v_nozzle*np.pi*d**2/4
    
    return drhodt