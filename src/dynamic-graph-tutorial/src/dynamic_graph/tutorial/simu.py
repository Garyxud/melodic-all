import dynamic_graph as dg
import dynamic_graph.tutorial as dgt
import matplotlib.pyplot as pl
import numpy as np


def build_graph():
    # define inverted pendulum
    a = dgt.InvertedPendulum("IP")
    a.setCartMass(1.0)
    a.setPendulumMass(1.0)
    a.setPendulumLength(1.0)

    b = dgt.FeedbackController("K")

    # plug signals
    stateOut = a.signal('state')
    forceIn = a.signal('force')
    stateIn = b.signal('state')
    forceOut = b.signal('force')

    dg.plug(stateOut, stateIn)
    dg.plug(forceOut, forceIn)

    # Set value of state signal
    s = stateOut
    f = forceIn

    s.value = (0.0, 0.1, 0.0, 0.0)

    gain = ((
        0.0,
        27.0,
        0.001,
        0.001,
    ), )
    b.setGain(gain, )

    return s, f, a


def play(nbSteps):
    s, f, a = build_graph()
    timeStep = 0.001
    timeSteps = []
    values = []
    forces = []

    # Loop over time and compute discretized state values
    for x in range(nbSteps):
        t = x * timeStep
        timeSteps.append(t)
        values.append(s.value)
        forces.append(f.value)
        a.incr(timeStep)

    # Convert into numpy array
    x = np.array(timeSteps)
    y = np.array(values).transpose()

    fig = pl.figure()
    ax1 = fig.add_subplot(121)
    ax2 = fig.add_subplot(122)

    # plot configuration variables
    ax1.plot(x, y[0])
    ax1.plot(x, y[1])

    # plot velocity variables
    ax2.plot(x, y[2])
    ax2.plot(x, y[3])
    ax2.plot(x, forces)

    ax1.legend(("x", "theta"))
    ax2.legend(("dx", "dtheta", "force"))

    pl.show()


if __name__ == '__main__':
    play(100)
