# write a function to solve any equation in numerical way using scipy
import math
import numpy as np
from scipy.optimize import fsolve
import matplotlib.pyplot as plt

def constraint_equation(theta, l1, d1, m, n, theta_p, theta_r, h1, h2, r):
    return - 2*l1*r*np.cos(theta)*np.cos(theta_p) \
            + l1**2*np.cos(theta_p)**2 \
                + d1**2*(1-np.cos(theta_r))**2 \
                    + (l1*np.sin(theta_p)-d1*np.sin(theta_r)-m)**2 \
                        - 2*(l1*np.sin(theta_p)-d1*np.sin(theta_r)-m)*r*np.sin(theta) \
                            +r**2 \
                                -h1**2

def draw_function(func):
    x = np.linspace(-np.pi, np.pi, 100)
    y = func(x)
    plt.plot(x, y)
    plt.show()

if __name__ == "__main__":
    h1 = 358.74 - (-25.06)
    h2 = 263.51 - (-25.06)

    m = 316.55
    n = 232.27

    r = math.sqrt((-69-(-22))**2 + (263.51-232.27)**2 ) # might be wrong
    l1 = 69 - 22

    d1 = 35

    draw_function(lambda x: constraint_equation(x, l1, d1, m, n, math.pi/6, 0, h1, h2, r))

    theta = fsolve(constraint_equation, 0, args=(l1, d1, m, n, math.pi/6, 0, h1, h2, r))
    print(theta)
    print(constraint_equation(theta, l1, d1, m, n, math.pi/6, 0, h1, h2, r))