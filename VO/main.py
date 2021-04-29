import numpy as np
import time
import math
import matplotlib.pyplot as plt

robot = [0,0]
r_robot = 0.1

goal = [20,10]

obs_init = [[6,0], [0,6], [4,0],[2,1]]#, [25,]]#, [0,7], [7,0],[10,10]]
obs_vel = [[-0.1,.1],[.24,0], [-.1,0],[0,-.1]]#, [0,-1], [-1,0], [-1,-1]]
r_obs = [0.2,0.2,0.2,.2]#,0.1,0.1,0.2]

# You probably won't need this if you're embedding things in a tkinter plot...
plt.ion()

robot_vel = [1,1]

fig = plt.figure()
ax = fig.add_subplot(111)
ax.set_aspect('equal')

def distance(arr1, arr2):
	return ((arr1[0] - arr2[0]) ** 2 + (arr1[1] - arr2[1]) ** 2) ** 0.5

def dot(arr1, arr2):
	return arr1[0] * arr2[0] + arr1[1] * arr2[1]

def norm(arr1):
	return distance(arr1,[0,0])

def compute_vel(obs_init, r_obs):
	vel = [0,0]
	maxi = 1e11
	for ii in range(- 10, + 10):
		for kk in range(- 10, + 10):
			i = robot[0] + ii * 0.01
			k = robot[1] + kk * 0.01
			# check 
			rvel = [i,k]
			flag = 0
			rvel_ac = [ rvel[0] - robot[0], rvel[1] - robot[1] ]
			for j in range(len(obs_init)):
				if rvel[0] == (robot[0] + obs_vel[j][0]) and rvel[1] == (robot[1] + obs_vel[j][1]):
					flag = 1
					break
				# check if in collision cone for next 1s
				vel_ac = [ rvel[0] - robot[0] - obs_vel[j][0], rvel[1] - robot[1] - obs_vel[j][1]]
				pij = [obs_init[j][0] - robot[0], obs_init[j][1] - robot[1]]
#				print(rvel,robot,obs_vel[j])
#				print(norm(vel_ac))
				ctheta = dot(vel_ac, pij)/(norm(pij) * norm(vel_ac))
#				print(ctheta)
				theta = np.arccos([ctheta])[0]
				stheta = (r_robot + r_obs[j])/(distance(obs_init[j], robot))
				theta0 = np.arcsin([stheta])[0]
				if abs(theta) < abs(theta0):
#					print(theta)
#					print(theta0)
#					print()
					flag = 1
			if maxi > distance(goal, rvel) and flag is 0:
				# add to potential
				vel = rvel_ac
				maxi = distance(goal, rvel)
	
	print(vel)
	print("AA")
	print(robot)
	return vel

for i in range(10000):
	if distance(robot, goal) < 1:
		print(distance(robot, goal))
		break
	ax.set_xlim([-10,30])
	ax.set_ylim([-10,30])
	#ax.plot([robot[0]],[robot[1]], 'bo')
	circle1 = plt.Circle((robot[0], robot[1]), r_robot, color='b')
	ax.add_patch(circle1)
	#ax.scatter(goal[0],goal[1],'g')
	#ax.scatter(robot[0],robot[1],'b')
	for j in range(len(obs_init)):
		temp = []
		temp.append(obs_init[j][0])
		temp.append(obs_init[j][1])
		circle1 = plt.Circle((obs_init[j][0], obs_init[j][1]), r_obs[j] * 3, color='r')
		ax.add_patch(circle1)
		obs_init[j][0] += obs_vel[j][0]
		obs_init[j][1] += obs_vel[j][1]
#		print([temp[0], obs_init[j][0]])
#		print([temp[1], obs_init[j][1]])
		print("AA")
		#ax.plot([temp[0], obs_init[j][0]], [temp[1], obs_init[j][1]], 'b-')
	circle1 = plt.Circle((goal[0], goal[1]), r_obs[j] * 3, color='g')
	ax.add_patch(circle1)
	robot_vel = compute_vel(obs_init, r_obs)
	#print(robot_vel)
	temp = [robot[0], robot[1]]
	robot[0] += robot_vel[0]
	robot[1] += robot_vel[1]
	ax.plot([temp[0], robot[0]], [temp[0], temp[1]])
	fig.canvas.draw()
	plt.show()
	plt.savefig(str(i) + '.png')
	time.sleep(0.1)
	fig.canvas.flush_events()
	ax.cla()


print("DONE")	
