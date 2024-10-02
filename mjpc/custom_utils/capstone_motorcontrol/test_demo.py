from NeuroLocoMiddleware.SoftRealtimeLoop import SoftRealtimeLoop
import numpy as np
import time
from src.can_mit_limit import TMotorManager_mit_can
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation  
import math

loop = SoftRealtimeLoop(dt=0.001, report=True, fade=0)
for t in loop :
    print(math.floor(t%2))