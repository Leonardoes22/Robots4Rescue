import cvxpy as cp
import numpy as np
import matplotlib.pyplot as plt

# System dynamics
A = np.eye(2)
B = np.eye(2)

# MPC parameters
N = 10  # Prediction horizon
dt = 0.1  # Time step
Q = np.eye(2)  # State cost matrix
R = np.eye(2)  # Control input cost matrix

# Hostage and follower positions
hostage_position = np.array([4, 5])
follower_initial_position = np.array([2, 2])

# Initial bot position
bot_position = np.array([0, 0])

# Maximum number of iterations and tolerance
max_iterations = 30
tolerance = 0.1

# Store the bot and follower positions for plotting
bot_positions = [bot_position]
follower_positions = [follower_initial_position]

# Maximum allowed velocity
v_x_max = 0.5
v_y_max = 0.5
max_velocity = np.array([v_x_max, v_y_max])

# Desired distance between leader and follower
desired_distance = 1

# Loop until the bot is sufficiently close to the hostage or reaches max_iterations
for i in range(max_iterations):
    # Optimization variables for leader and follower
    x_leader = cp.Variable((2, N + 1))
    u_leader = cp.Variable((2, N))
    x_follower = cp.Variable((2, N + 1))
    u_follower = cp.Variable((2, N))

    # MPC optimization problem for leader
    cost_leader = 0
    constraints_leader = []
    for t in range(N):
        cost_leader += cp.quad_form(x_leader[:, t] - hostage_position, Q) + cp.quad_form(u_leader[:, t], R)
        constraints_leader += [
            x_leader[:, t + 1] == A @ x_leader[:, t] + B @ u_leader[:, t],
            cp.norm(u_leader[:, t], 2) <= cp.norm(max_velocity, 2),
        ]
    constraints_leader += [x_leader[:, 0] == bot_position]

    # MPC optimization problem for follower
    cost_follower = 0
    constraints_follower = []
    for t in range(N):
        cost_follower += cp.quad_form(x_follower[:, t] - bot_position + np.array([0.5, 0.5]), Q) + cp.quad_form(u_follower[:, t], R)
        constraints_follower += [
            x_follower[:, t + 1] == A @ x_follower[:, t] + B @ u_follower[:, t],
            cp.norm(u_follower[:, t], 2) <= cp.norm(max_velocity, 2),
        ]
    constraints_follower += [x_follower[:, 0] == follower_initial_position]

    # Solve optimization problems
    prob_leader = cp.Problem(cp.Minimize(cost_leader), constraints_leader)
    prob_leader.solve()
    prob_follower = cp.Problem(cp.Minimize(cost_follower), constraints_follower)
    prob_follower.solve()

    # Extract optimal control inputs
    optimal_u_leader = u_leader[:, 0].value
    optimal_u_follower = u_follower[:, 0].value
    

    # Apply control inputs to the system and update the system states
    next_bot_position = A @ bot_position + B @ optimal_u_leader
    next_follower_position = A @ follower_initial_position + B @ optimal_u_follower

    # Store the updated bot and follower positions
    bot_positions.append(next_bot_position)
    follower_positions.append(next_follower_position)

    # Check if the bot is sufficiently close to the hostage
    if np.linalg.norm(next_bot_position - hostage_position) < tolerance:
        print(f'Bot reached the hostage after {i + 1} iterations')
        break

    # Update the bot and follower positions for the next iteration
    bot_position = next_bot_position
    follower_initial_position = next_follower_position
else:
    print('Bot did not reach the hostage within the maximum number of iterations')

bot_positions = np.array(bot_positions)
follower_positions = np.array(follower_positions)

plt.figure()
plt.plot(hostage_position[0], hostage_position[1], 'ro', label='Hostage')
plt.plot(bot_positions[:, 0], bot_positions[:, 1], 'b.-', label='Leader path')
plt.plot(follower_positions[:, 0], follower_positions[:, 1], 'g.-', label='Follower path')
plt.legend()
plt.xlabel('x')
plt.ylabel('y')
plt.title('Bot, Follower, and Hostage Positions')
plt.grid()
plt.show()