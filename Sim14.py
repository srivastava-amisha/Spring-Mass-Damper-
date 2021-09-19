import sys
sys.path.append('..')  # add parent directory
import matplotlib.pyplot as plt
import numpy as np
import Param as P
from Dynamics import Dynamics
from Controller import Controller
from SignalGenerator import SignalGenerator
from Animation import Animation
from dataPlotter import dataPlotter
from dataPlotterObserver import dataPlotterObserver

# instantiate arm, controller, and reference classes
spring = Dynamics(alpha=0.2)
controller = Controller()
reference = SignalGenerator(amplitude=1,
                            frequency=0.05)
disturbance = SignalGenerator(amplitude=0.25)
noise = SignalGenerator(amplitude=0.01)

# instantiate the simulation plots and animation
dataPlot = dataPlotter()
dataPlotObserver = dataPlotterObserver()
animation = Animation()

t = P.t_start  # time starts at t_start
y = spring.h()  # output of system at start of simulation

while t < P.t_end:  # main simulation loop
    # Get referenced inputs from signal generators
    # Propagate dynamics in between plot samples
    t_next_plot = t + P.t_plot

    # updates control and dynamics at faster simulation rate
    while t < t_next_plot: 
        r = reference.square(t)
        d = disturbance.step(t)  # input disturbance
        n = noise.random(t)  # simulate sensor noise

        # update controller
        u, xhat, dhat = controller.update(r, y + n)  
        y = spring.update(u + d)  # propagate system
        t = t + P.Ts  # advance time by Ts

    # update animation and data plots
    animation.update(spring.state)
    dataPlot.update(t, r, spring.state, u)
    dataPlotObserver.update(t, spring.state, xhat, d, dhat)
    plt.pause(0.0001)  

# Keeps the program from closing until the user presses a button.
print('Press key to close')
plt.waitforbuttonpress()
plt.close()
