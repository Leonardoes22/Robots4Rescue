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

# Hostage position
hostage_position = np.array([4, 5])

# Initial bot position
bot_position = np.array([0, 0])

# Maximum number of iterations and tolerance
max_iterations = 100
tolerance = 0.1

# Store the bot positions for plotting
bot_positions = [bot_position]
# Maximum allowed velocity
v_x_max = 0.5
v_y_max = 0.5
max_velocity = np.array([v_x_max, v_y_max])

# Loop until the bot is sufficiently close to the hostage or reaches max_iterations
for i in range(max_iterations):
    # Optimization variables
    x = cp.Variable((2, N+1))
    u = cp.Variable((2, N))

    # MPC optimization problem
    cost = 0
    constraints = []

    for t in range(N):
        cost += cp.quad_form(x[:, t] - hostage_position, Q) + cp.quad_form(u[:, t], R)
        constraints += [
            x[:, t+1] == A @ x[:, t] + B @ u[:, t],
            cp.norm(u[:, t], 2) <= cp.norm(max_velocity, 2),  # Add velocity constraint
        ]

    # Initial state constraint
    constraints += [x[:, 0] == bot_position]

    # Solve optimization problem
    prob = cp.Problem(cp.Minimize(cost), constraints)
    prob.solve()
    # Extract optimal control input
    optimal_u = u[:, 0].value

    # Apply control input to the system and update the system state
    next_bot_position = A @ bot_position + B @ optimal_u

    # Store the updated bot position
    bot_positions.append(next_bot_position)

    # Check if the bot is sufficiently close to the hostage
    if np.linalg.norm(next_bot_position - hostage_position) < tolerance:
        print(f'Bot reached the hostage after {i + 1} iterations')
        break

    # Update the bot position for the next iteration
    bot_position = next_bot_position
else:
    print('Bot did not reach the hostage within the maximum number of iterations')

# Convert the bot_positions list to a NumPy array
bot_positions = np.array(bot_positions)

# Plot the bot and the hostage
plt.figure()
plt.plot(hostage_position[0], hostage_position[1], 'ro', label='Hostage')
plt.plot(bot_positions[:, 0], bot_positions[:, 1], 'b.-', label='Bot path')
plt.legend()
plt.xlabel('x')
plt.ylabel('y')
plt.title('Bot and Hostage Positions')
plt.grid()
plt.show()
