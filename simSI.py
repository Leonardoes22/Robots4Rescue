
#%%
import numpy as np
import matplotlib.pyplot as plt

#%%
# Parameters

# Robot Class
class Robot():

    def __init__(self, 
                 pos=np.array([0.0, 0.0]), 
                 vel=np.array([0.0, 0.0])):
        
        self.pos = pos
        self.vel = vel

    def __repr__(self) -> str:
        return f"Robot({self.pos})"


# Simple Integrator Simulator
class simSI():

    simTime = 10
    stepTime = 0.1

    robots = {}

    def __init__(self):
        self._nrobots = 0


    def addRobot(self, 
                 pos=np.array([0.0, 0.0]), 
                 vel=np.array([0.0, 0.0])):
        
        self.robots[self._nrobots] = Robot(pos, vel)
        self._nrobots += 1


# %%


# Simulate 
sim = simSI()

sim.addRobot(pos=np.array([1.0, 1.0]))
sim.addRobot(pos=np.array([10.0, 1.0]))
sim.addRobot(pos=np.array([10.0, 10.0]))
#sim.robots[0].vel = np.array([2.0, 1.0])
#sim.robots[1].vel = np.array([0, 0.5])


TT = np.linspace(0, sim.simTime, int(sim.simTime/sim.stepTime))

robots_trajectory = {}
for robot in sim.robots:
    robots_trajectory[robot] = np.zeros([len(TT),2])
    robots_trajectory[robot][0,:] = sim.robots[robot].pos

for k in range(len(TT)):

    if k>0:

        # Control law
        sim.robots[0].vel = -sim.robots[0].pos + sim.robots[1].pos
        sim.robots[1].vel = -sim.robots[1].pos + sim.robots[0].pos
        sim.robots[2].vel = -sim.robots[2].pos + sim.robots[1].pos
    
        # Update robot position
        for robot in sim.robots:
            sim.robots[robot].pos += sim.robots[robot].vel*sim.stepTime


        # Save data
        for robot in sim.robots:
            robots_trajectory[robot][k,:] = sim.robots[robot].pos
    


# %%

# Show Trajectory
plot = plt.figure()

for robot in sim.robots:
    plt.plot(robots_trajectory[robot][:,0], robots_trajectory[robot][:,1])
    plt.plot(robots_trajectory[robot][0,0], robots_trajectory[robot][0,1], 'ro')
    plt.plot(robots_trajectory[robot][-1,0], robots_trajectory[robot][-1,1], 'gx')


plt.xlabel('x (m)')
plt.ylabel('y (m)')
plt.title('Trajectory')
plt.show()

# Plot Trajectory
plot = plt.figure(figsize=(8,8))

plt.subplot(2,1,1)
for robot in sim.robots:
    plt.plot(TT, robots_trajectory[robot][:,0])

plt.ylabel('x (m)')
plt.title('Trajectory (x)')


plt.subplot(2,1,2)
for robot in sim.robots:
    plt.plot(TT, robots_trajectory[robot][:,1])


plt.xlabel('Time (s)')
plt.ylabel('y (m)')
plt.title('Trajectory (y)')




# %%
