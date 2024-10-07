import numpy as np
from casadi import *
from constants import *
from dynamics import *
from kinematics import *
import matplotlib.pyplot as plt
import json
import armPresets as AP
import filewriter
from constraints import Constraint


def convert_to_json(j1, j2, dt, name):
    data = []
    for i in range(len(j1)):
        data.append({'j1': j1[i], 'j2': j2[i], 'a1': 0, 'a2': 0})

    with open("paths/" + name + ".json", 'w') as f:
        json.dump({'dt': dt, 'traj': data}, f, indent=2)


def solver(j1I, j2I, j1F, j2F, name, constraint_list=[]):
    opti = Opti()

    N = 30  # controls

    T = opti.variable()
    dt = T / N

    x_n = 2  # [j1, j2]
    X = opti.variable(N + 1, x_n)

    opti.set_initial(T, 5)
    opti.subject_to(T > 0.0)

    # init_posX = fkx(X[0, 0], X[0, 1])
    # init_posY = fky(X[0, 0], X[0, 1])

    # final_posX = fkx(X[N, 0], X[N, 1])
    # final_posY = fky(X[N, 0], X[N, 1])

    opti.subject_to(X[0, 0] == j1I)
    opti.subject_to(X[0, 1] == j2I)

    opti.subject_to(X[N, 0] == j1F)
    opti.subject_to(X[N, 1] == j2F)
    t = []
    ##Angle constraints
    for i in range(0, N + 1):
        t.append(i)
        opti.subject_to(X[i, 0] > 0)
        opti.subject_to(X[i, 0] < 3.1415)

    x0A = [-0.69, 1.11, 0.3]
    y0A = [-0.3, 0.9, 0]
    x1A = [0.13, 1.1, 1]
    y1A = [0, 1.03, 0.55]

    for i in range(0, N + 1):
        x = fkx(X[i, 0], X[i, 1])
        y = fky(X[i, 0], X[i, 1])
        # opti.subject_to(y > 0)
        # opti.subject_to(y < 1.98 - 0.2)

        opti.set_initial(X[i, 0], j1I + i * (j1F - j1I) / N)
        opti.set_initial(X[i, 1], j2I + i * (j2F - j2I) / N)

        # opti.subject_to(((x - 0.5)**6)/(0.08**6) + ((y - 0.2)**6)/(0.33**6) > 1) #cone node
        j1 = X[i, 0]
        j2 = X[i, 1]
        # opti.subject_to((((j1 - 0.2)**2) / (1.5**2)) + (((j2 + 1.8)**2)/2) > 1)
        # opti.subject_to(((j1 - 3)**2) + (((j2 + 3.6)**2)/2.5) > 1)

        # x_0 = -0.5
        # y_0 = -0.1
        # x_1 = -0.3
        # y_1 = 0.2
        for q in constraint_list:
            opti.subject_to(
                fmax(
                    fabs((2 * (x - ((q[0] + q[1]) / 2)) / (q[1] - q[0]))),
                    fabs((2 * (y - ((q[2] + q[3]) / 2)) / (q[3] - q[2])))
                ) > 1
            )

        # opti.subject_to((((j1 - 3)**2) / (1**2)) + (((j2 + 4.2)**2)/1.5) > 1)
    for i in range(0, N + 1):
        preDex = 0 if i == 0 else i - 1
        nexDex = N if i == N else i + 1

        print(preDex, i, nexDex)

        dx1 = X[i, 0] - X[preDex, 0]
        dx2 = X[i, 1] - X[preDex, 1]

        dx1_2 = X[nexDex, 0] - X[i, 0]
        dx2_2 = X[nexDex, 1] - X[i, 1]

        v1 = dx1 / dt
        v2 = dx2 / dt

        v1_2 = dx1_2 / dt
        v2_2 = dx2_2 / dt

        a1 = (v1_2 - v1) / dt
        a2 = (v2_2 - v2) / dt

        if (i > 0 and i < N):
            V = ff(X[i, 0], X[i, 1], v1, v2, a1, a2)

            opti.subject_to(V[0, 0] < 12)
            opti.subject_to(V[0, 0] > -12)
            opti.subject_to(V[1, 0] < 12)
            opti.subject_to(V[1, 0] > -12)

        if (i == 1 or i == N):
            opti.subject_to(v1 == 0)
            opti.subject_to(v2 == 0)

        if (i > 0):
            opti.subject_to(v1 < free_speed * j1GR)
            opti.subject_to(v1 > -free_speed * j1GR)
            opti.subject_to(v2 < free_speed * j2GR)
            opti.subject_to(v2 > -free_speed * j2GR)

    opti.minimize(T)
    opti.solver("ipopt", {}, {"mu_init": 1e-6})
    sol = opti.solve()

    convert_to_json(sol.value(X)[:, 0], sol.value(X)[:, 1], sol.value(dt), name)


# solver(pi/2, -pi, 2.455, -(2*pi - 1.705), "thingy")
# solver(pi/2, -pi, 2.5151979515, -(2*pi-1.5870385134), "thingy")

def solve():
    AP.reload()

    solve_paths = filewriter.importData("config.json")

    for a in solve_paths:
        if solve_paths[a]["type"] == "all":
            for start in solve_paths[a]["paths"]:
                for end in solve_paths[a]["paths"]:
                    if start != end:

                        j1I = AP.presets[start][0]
                        j2I = AP.presets[start][1]

                        j1F = AP.presets[end][0]
                        j2F = AP.presets[end][1]

                        solver(j1I, j2I, j1F, j2F, start + "_to_" + end, solve_paths[a]["constraints"])
        elif solve_paths[a]["type"] == "sequential":
            for i in range(len(solve_paths[a]["paths"]) - 1):
                point1 = solve_paths[a]["paths"][i]
                point2 = solve_paths[a]["paths"][i+1]
                j1I = AP.presets[point1][0]
                j2I = AP.presets[point1][1]

                j1F = AP.presets[point2][0]
                j2F = AP.presets[point2][1]
                solver(j1I, j2I, j1F, j2F, point1 + "_to_" + point2, solve_paths[a]["constraints"])

    print("All paths solved")


# solver(j1I, j2I, j1F, j2F, pos_to_pos)

if __name__ == "__main__":
    solve()
