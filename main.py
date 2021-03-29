import numpy
import matplotlib.pyplot as plt
import time
from model import *
import cvxpy as cp

# Grid setup
h = 300
w = 300
grid = [[0 for i in range(h)] for j in range(w)]

# Goal
goal = [100,100]

# obstacles
obs = [[15,15],[30,30],[50,50],[70,70]]
r = [1,3,5,2]

# time step in s
time_step = 0.5

# Prediction horizon
N = 5

# time_out
TIME_INIT = time.time()
TIME_OUT = 200

# input max
MAX_INP = [10,10]
NORM_MAX = 10

def generate_n_inputs(robot):
    inputs = [[0,0] for i in range(N)]
    n = 2
    m = 2
    T = N
    alpha = 0.2
    beta = 5
    A = np.eye(n)
    B = np.eye(n)
    C = np.eye(n)/2
    x = cp.Variable((n, T + 1))
    u = cp.Variable((m, T))
    v = cp.Variable((m, T + 1))

    cost = 0
    constr = []
    for t in range(T):
        cost += cp.sum_squares(x[:,t+1] - [goal[0], goal[1]]) # +  cp.sum_squares(u[:,t])
        constr += [x[:,t+1] == A@x[:,t] + B@u[:,t], v[:,t+1] == u[:,t], cp.norm(u[:,t], 'inf') <= NORM_MAX]
    # sums problem objectives and concatenates constraints.
    constr += [x[:,0] == robot.get_pos()]
    problem = cp.Problem(cp.Minimize(cost), constr)
    problem.solve(solver=cp.ECOS)

    arr = []
    if u.value is not None:
        arr = u.value.T;
        for i in range(N):
            for j in range(len(arr[i])):
                inputs[i][j] = arr[i][j]

    return inputs

def vis_grid(robot, cnt, his):
    print("Visualizing")
    '''
    '''
    circle1 = plt.Circle((10, 10), 20, color='r')
    circle2 = plt.Circle((100, 100), 20, color='blue')
    circle3 = plt.Circle((50, 50), 50, color='g', clip_on=False)

    fig, ax = plt.subplots() # note we must use plt.subplots, not plt.subplot
    ax.set_aspect('equal', adjustable='box')
    # (or if you have an existing figure)
    # fig = plt.gcf()
    # ax = fig.gca()
    for i in range(len(obs)):
        circs = plt.Circle((obs[i][0],obs[i][1]), r[i], color='red')
        ax.add_patch( circs )

    for j in range(len(his) - 1):
        ax.plot((his[i][0], his[i][1]), (his[i + 1][0], his[i][1]),'b-')

    plt.xlim([0, w])
    plt.ylim([0, h])
    #ax.add_patch(circle1)
    #ax.add_patch(circle2)
    #ax.add_patch(circle3)

    circs = plt.Circle((goal[0],goal[1]), 10, color='green')
    ax.add_patch( circs )
    plt.savefig("image" + str(cnt) + ".png")

def get_distance(x1, x2):
    np.random.seed(1)
    return ((x1[0] - x2[0])**2 + (x1[1] - x2[1])**2)**0.5

def mpc_loop(robot):
    cnt = 0
    while time.time() - TIME_INIT <= 200:
        if get_distance(robot.get_pos(), goal) < 2:
            print("REACHED GOAL")
            break

        # Generate N inputs
        inputs = generate_n_inputs(robot)
        
        # Apply the first one
        robot.move_me(inputs[0])

        # Visualize the difference
        vis_grid(robot, cnt)
        cnt = cnt + 1

        print(robot.get_pos())

        # time tick
        time.sleep(0.5)

if __name__ == "__main__":
    robot = Robot([0,0], [0,0])
    
    mpc_loop(robot)
