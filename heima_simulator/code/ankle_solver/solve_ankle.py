import math

def solve_eq(a, b, c):
    # solve a*cos(theta) + b*sin(theta) = c
    print(f"a: {a}, b: {b}, c: {c}")
    delta = 4*b**2*c**2 - 4*(c**2-a**2)*(a**2+b**2)
    print(f"delta: {delta}")
    sin_theta_1 = (2*b*c+math.sqrt(delta))/(2*(a**2+b**2))
    sin_theta_2 = (2*b*c-math.sqrt(delta))/(2*(a**2+b**2))
    print(f"sin_theta_1: {sin_theta_1}, sin_theta_2: {sin_theta_2}")
    if sin_theta_1 > 1 or sin_theta_1 < -1:
        sin_theta_1 = None
    if sin_theta_2 > 1 or sin_theta_2 < -1:
        sin_theta_2 = None
    return sin_theta_1, sin_theta_2

def solve_theta1(l1, d1, m, n, theta_p, theta_r, h1, h2):
    print(f"l1: {l1}, d1: {d1}, m: {m}, n: {n}, theta_p: {theta_p}, theta_r: {theta_r}, h1: {h1}, h2: {h2}")
    a = -2*l1**2*math.cos(theta_p)
    b = -2*(l1*math.sin(theta_p)-d1*math.sin(theta_r)-m)*l1
    c = h1**2 - l1**2 - l1**2*math.cos(theta_p)**2 - d1**2*(1-math.cos(theta_r))**2 - (l1*math.sin(theta_p)-d1*math.sin(theta_r)-m)**2

    sin_theta_1_1, sin_theta_1_2 = solve_eq(a, b, c)
    print(sin_theta_1_1, sin_theta_1_2)

    a = -2*l1**2*math.cos(theta_p)
    b = -2*(l1*math.sin(theta_p)+d1*math.sin(theta_r)-n)*l1
    c = h2**2 - l1**2 - l1**2*math.cos(theta_p)**2 - d1**2*(1-math.cos(theta_r))**2 - (l1*math.sin(theta_p)+d1*math.sin(theta_r)-n)**2

    sin_theta_2_1, sin_theta_2_2 = solve_eq(a, b, c)
    print(sin_theta_2_1, sin_theta_2_2)
    
if __name__ == "__main__":
    # verify the solve_eq function
    # sin_theta_1, sin_theta_2 = solve_eq(2, 1, math.sqrt(3)+0.5)
    # print(sin_theta_1, sin_theta_2)
    # print(sin_theta_1 + 2*math.cos(math.asin(sin_theta_1)))

    h1 = 358.74 - (-25.06)
    h2 = 263.51 - (-25.06)

    m = 316.55
    n = 232.27

    l1 = math.sqrt((-69-(-22))**2 + (263.51-232.27)**2 ) # might be wrong

    d1 = 35

    solve_theta1(l1, d1, m, n, math.pi/6, 0, h1, h2)