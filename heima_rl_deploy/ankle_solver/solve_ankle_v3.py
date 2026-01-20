import numpy as np
import math
from scipy.optimize import fsolve


class AnkleSolver:
    def __init__(self, l1, d1, m, n, h1, h2, r_E, r_F):
        self.l1 = l1
        self.d1 = d1
        self.m = m
        self.n = n

        self.h1 = h1
        self.h2 = h2

        self.r_E = r_E
        self.r_F = r_F

    def solve(self, pitch, roll):
        def constraint_equation(theta_f):
            t = []
            for i in theta_f:
                v = self.get_distance(self.get_D_after_rotation(pitch, roll), self.get_F(i)) - self.h1
                t.append(v)
            return np.array(t)
        theta_f = fsolve(constraint_equation, 0)
        print(f"theta_f: {theta_f}")
        print(f"constraint_equation(theta_f): {constraint_equation(theta_f)}")

        def constraint_equation(theta_e):
            t = []
            for i in theta_e:
                v = self.get_distance(self.get_E(i), self.get_C_after_rotation(pitch, roll)) - self.h2
                t.append(v)
            return np.array(t)
        theta_e = fsolve(constraint_equation, 0)
        print(f"theta_e: {theta_e}")
        print(f"constraint_equation(theta_e): {constraint_equation(theta_e)}")

        return theta_f, theta_e
    
    def get_DF(self, pitch, roll, theta_f):
        return self.get_distance(self.get_D_after_rotation(pitch, roll), self.get_F(theta_f))
    
    def get_CE(self, pitch, roll, theta_e):
        return self.get_distance(self.get_C_after_rotation(pitch, roll), self.get_E(theta_e))

    def get_distance(self, coordinate1, coordinate2):
        return np.linalg.norm(coordinate1 - coordinate2)

    def get_E(self, theta):
        temp = np.array([-self.r_E, self.d1, 0])
        temp = self.apply_rotation_y(temp, theta)
        temp = temp + np.array([-22, 0, self.n])
        return temp

    def get_F(self, theta):
        temp = np.array([-self.r_F, -self.d1, 0])
        temp = self.apply_rotation_y(temp, theta)
        temp = temp + np.array([-22, 0, self.m])
        return temp

    def get_D_after_rotation(self, pitch, roll):
        temp_d = np.array([-69, -35, -25.06])
        return self.get_coord_after_pitch_roll(temp_d, pitch, roll)
    
    def get_C_after_rotation(self, pitch, roll):
        temp_c = np.array([-69, 35, -25.06])
        return self.get_coord_after_pitch_roll(temp_c, pitch, roll)
    
    def get_coord_after_pitch_roll(self, coordinate, pitch, roll):
        coordinate = self.apply_rotation_x(coordinate, roll)
        coordinate = self.apply_rotation_y(coordinate, pitch)
        return coordinate

    def apply_rotation_x(self, coordinate, angle):
        # multiply the coordinate by rotaion matrix around x axis
        rotation_matrix = np.array([
            [1, 0, 0], 
            [0, math.cos(angle), -math.sin(angle)], 
            [0, math.sin(angle), math.cos(angle)]
            ])
        return rotation_matrix @ coordinate

    def apply_rotation_y(self, coordinate, angle):
        # multiply the coordinate by rotaion matrix around y axis
        rotation_matrix = np.array([
            [math.cos(angle), 0, math.sin(angle)],
            [0, 1, 0],
            [-math.sin(angle), 0, math.cos(angle)]
            ])
        return rotation_matrix @ coordinate

if __name__ == "__main__":

    h1 = 358.74 - (-25.06)
    h2 = 263.51 - (-25.06)

    m = 316.55
    n = 232.27

    r_E = math.sqrt((-69-(-22))**2 + (263.51-232.27)**2 ) # might be wrong
    r_F = math.sqrt((-69-(-22))**2 + (358.74-316.55)**2 ) # might be wrong
    l1 = 69 - 22

    d1 = 35

    ankle_solver = AnkleSolver(l1, d1, m, n, h1, h2, r_E, r_F)
    coordinate_c = np.array([-l1, d1, 0])
    print(f"coordinate_c: {coordinate_c}")

    coordinate_c_after_pitch_roll = ankle_solver.get_coord_after_pitch_roll(coordinate_c, math.pi/6, 0)
    print(f"coordinate_c_after_pitch_roll: {coordinate_c_after_pitch_roll}")

    coordinate_e = ankle_solver.get_E(0)
    print(f"coordinate_e: {coordinate_e}")

    coordinate_f = ankle_solver.get_F(0)
    print(f"coordinate_f: {coordinate_f}")

    theta_f, theta_e = ankle_solver.solve(0, 0)
    print(f"theta_f: {theta_f}, theta_e: {theta_e}")

    # draw df
    import matplotlib.pyplot as plt
    df = []
    for i in np.linspace(-math.pi, math.pi, 100):
        df.append(ankle_solver.get_CE(0, 0, i))
    plt.plot(df)
    plt.show()

