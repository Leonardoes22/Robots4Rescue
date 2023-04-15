
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
class SimSI():

    # Simulation parameters
    simTime = 10
    stepTime = 0.1

    # Simulation data
    robots = {}

    def __init__(self):
        self._nrobots = 0


    def addRobot(self, 
                 pos=np.array([0.0, 0.0]), 
                 vel=np.array([0.0, 0.0])):
        
        self.robots[self._nrobots] = Robot(pos, vel)
        self._nrobots += 1

    def simulate(self):
        TT = np.linspace(0, self.simTime, int(self.simTime/self.stepTime))

        # Initialize data
        robots_trajectory = {}
        robots_velocity = {}
        for robot in self.robots:
            robots_trajectory[robot] = np.zeros([len(TT),2])
            robots_trajectory[robot][0,:] = self.robots[robot].pos

            robots_velocity[robot] = np.zeros([len(TT),2])
            robots_velocity[robot][0,:] = self.robots[robot].vel

        # Simulation loop
        for k in range(len(TT)):

            if k>0:

                # Control law
                self.robots[0].vel = -self.robots[0].pos + self.robots[1].pos
                self.robots[1].vel = -self.robots[1].pos + self.robots[0].pos
                self.robots[2].vel = -self.robots[2].pos + self.robots[1].pos
            
                # Update robot position
                for robot in self.robots:
                    self.robots[robot].pos += self.robots[robot].vel*self.stepTime


                # Save data
                for robot in self.robots:
                    robots_trajectory[robot][k,:] = self.robots[robot].pos
                    robots_velocity[robot][k,:] = self.robots[robot].vel

        return TT, robots_trajectory, robots_velocity

# %%


sim = SimSI()

sim.simTime = 10
sim.stepTime = 0.1

sim.addRobot(pos=np.array([1.0, 1.0]))
sim.addRobot(pos=np.array([10.0, 1.0]))
sim.addRobot(pos=np.array([10.0, 10.0]))

[TT, robots_trajectory, robots_velocity] = sim.simulate()


# %%



# Show Trajectory
plot = plt.figure()

for robot in sim.robots:
    plt.plot(robots_trajectory[robot][:,0], robots_trajectory[robot][:,1])
    plt.plot(robots_trajectory[robot][0,0], robots_trajectory[robot][0,1], 'ro',label='_nolegend_')
    plt.plot(robots_trajectory[robot][-1,0], robots_trajectory[robot][-1,1], 'gx',label='_nolegend_')


plt.xlabel('x (m)')
plt.ylabel('y (m)')
plt.title('Trajectory')
plt.legend([str(key) for key in sim.robots.keys()])
plt.show()

# Plot Trajectory
plot = plt.figure(figsize=(10,5))

plt.subplot(1,2,1)
for robot in sim.robots:
    plt.plot(TT, robots_trajectory[robot][:,0])

plt.xlabel('Time (s)')
plt.ylabel('x (m)')
plt.title('Trajectory (x)')
plt.legend([str(key) for key in sim.robots.keys()])


plt.subplot(1,2,2)
for robot in sim.robots:
    plt.plot(TT, robots_trajectory[robot][:,1])


plt.xlabel('Time (s)')
plt.ylabel('y (m)')
plt.title('Trajectory (y)')
plt.legend([str(key) for key in sim.robots.keys()])


# Plot Velocity
plot = plt.figure(figsize=(10,5))

plt.subplot(1,2,1)
for robot in sim.robots:
    plt.plot(TT, robots_velocity[robot][:,0])

plt.xlabel('Time (s)')
plt.ylabel('vx (m/s)')
plt.title('Velocity (vx)')
plt.legend([str(key) for key in sim.robots.keys()])

plt.subplot(1,2,2)
for robot in sim.robots:
    plt.plot(TT, robots_velocity[robot][:,1])

plt.xlabel('Time (s)')
plt.ylabel('vy (m/s)')
plt.title('Velocity (vy)')
plt.legend([str(key) for key in sim.robots.keys()])

plt.show()
# %%
