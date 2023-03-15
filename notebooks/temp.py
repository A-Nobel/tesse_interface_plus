from tesse.env import Env
from tesse.msgs import *

import time
import matplotlib.pyplot as plt
import defusedxml.ElementTree as ET
import numpy as np


     

env = Env()

#response = env.request(SceneRequest(scene_index))
#if response is not None:
#    print(response.metadata)
#response =env.request(RemoveObjectsRequest([2,4,6,8,10,12,14,16,18,20]))
#response = env.request(RemoveObjectsRequest([2,4,6,8,10,12,14,16,18,20]))

#if response is not None:
#    print(response.metadata)



x = -6
y = .5
z = 0 #8.5
radius = 1.5
orientation = [ 0.4619398, 0.1913417, 0.4619398, 0.7325378 ]
orientation = [0, 0, 0, 1]
env.send(Reposition(x, y, z, 0, 0, 0, 0))
time.sleep(.2)

for angle in range(0, 360, 10):
    response = env.request(SpawnObjectRequest(4,
                                ObjectSpawnMethod.USER,
                                x + radius*np.cos(angle*np.pi/180), 
                                y, 
                                z + radius*np.sin(angle*np.pi/180),
                                *orientation)
            )

#env.request(RemoveObjectsRequest())

