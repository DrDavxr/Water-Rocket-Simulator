from Equations_of_Tank import TankFlow
import numpy as np

state_vector = np.array([3.37])

tank = TankFlow(state_vector, 0.1, 0.102, 0.008, 101325, 1000, 1.4)

density = tank.DensityComp(0.1)