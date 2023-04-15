import numpy as np
from scipy.optimize import minimize
import matplotlib.pyplot as plt

def unicycle_model(x, u, dt):
    x_next = x + dt * np.array([u[0] * np.cos(x[2]), u[0] * np.sin(x[2]), u[1]])
    return x_next

# MPC parameters
N = 20
dt = 0.1
Q = np.diag([1, 1, 0.1])
R = np.diag([0.1, 0.1])

# Hostage position
hostage_position = np.array([0, 50])

# Initial bot position and orientation
bot_state = np.array([0, 0, np.pi/4])

# Maximum number of iterations and tolerance
max_iterations = 100
tolerance = 0.1

# Store the bot positions for plotting
bot_positions = [bot_state[:2]]

def cost_function(variables, *args):
    x = variables[:3*(N+1)].reshape(3, N+1)
    u = variables[3*(N+1):].reshape(2, N)

    Q, R, hostage_position, bot_state = args

    cost = 0
    for t in range(N):
        cost += (x[:, t] - np.array([*hostage_position, 0])).T @ Q @ (x[:, t] - np.array([*hostage_position, 0])) + u[:, t].T @ R @ u[:, t]

    return cost

def constraints(variables, *args):
    x = variables[:3*(N+1)].reshape(3, N+1)
    u = variables[3*(N+1):].reshape(2, N)

    _, _, _, bot_state = args

    constraints = []

    for t in range(N):
        constraints.append(unicycle_model(x[:, t], u[:, t], dt) - x[:, t+1])

    constraints.append(x[:, 0] - bot_state)

    return np.concatenate(constraints)

# Loop until the bot is sufficiently close to the hostage or reaches max_iterations
for i in range(max_iterations):
    # Define the initial values for the optimization variables
    initial_values = np.zeros(3*(N+1) + 2*N)

    # Pack the required arguments for the cost function and constraints
    args = (Q, R, hostage_position, bot_state)

    # Define the bounds for the optimization variables
    bounds = [(None, None)] * (3*(N+1) + 2*N)

    # Define the constraint dictionary
    constraint_dict = {'type': 'eq', 'fun': constraints, 'args': args}

    # Solve the optimization problem using scipy.optimize.minimize
    result = minimize(cost_function, initial_values, args=args, method='SLSQP', bounds=bounds, constraints=constraint_dict)

    # Extract the optimal variables from the result
    optimal_variables = result.x
    x_opt = optimal_variables[:3*(N+1)].reshape(3, N+1)
    u_opt = optimal_variables[3*(N+1):].reshape(2, N)

    # Apply the optimal control inputs to the system and update the system states
    next_bot_state = unicycle_model(bot_state, u_opt[:, 0], dt)
    # Store the updated bot positions
    bot_positions.append(next_bot_state[:2])

    # Check if the bot is sufficiently close to the hostage
    if np.linalg.norm(next_bot_state[:2] - hostage_position) < tolerance:
        print(f'Bot reached the hostage after {i + 1} iterations')
        break

    # Update the bot position for the next iteration
    bot_state = next_bot_state


bot_positions = np.array(bot_positions)


final_cost = cost_function(optimal_variables, *args)
print(f'Final cost: {final_cost}')

plt.figure()
plt.plot(hostage_position[0], hostage_position[1], 'ro', label='Hostage')
plt.plot(bot_positions[:, 0], bot_positions[:, 1], 'b.-', label='Bot path')
plt.legend()
plt.xlabel('x')
plt.ylabel('y')
plt.title('Bot and Hostage Positions (Unicycle Model)')
plt.grid()
plt.show()