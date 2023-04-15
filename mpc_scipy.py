import numpy as np
from scipy.optimize import minimize
import matplotlib.pyplot as plt

# System dynamics
A = np.eye(2)
B = np.eye(2)

# MPC parameters
N = 20           #predictionhorizon
dt = 0.1  # Time step
Q = np.eye(2)  # State cost matrix
R = np.eye(2)  # Control input cost matrix

# Hostage position
hostage_position = np.array([50, 0])

# Initial bot position
bot_position = np.array([0, 0])

# Maximum number of iterations and tolerance
max_iterations = 100
tolerance = 0.001

# Store the bot positions for plotting
bot_positions = [bot_position]

# Maximum allowed velocity
v_x_max = 0.5
v_y_max = 0.5
max_velocity = np.array([v_x_max, v_y_max])

def leader_cost_function(variables, *args):
    x_leader = variables[:2*(N+1)].reshape(2, N+1)
    u_leader = variables[2*(N+1):2*(N+1) + 2*N].reshape(2, N)

    Q, R, A, B, hostage_position, bot_position, max_velocity = args

    # Leader cost
    cost_leader = 0
    for t in range(N):
        cost_leader += np.linalg.norm(x_leader[:, t] - hostage_position)**2 + np.linalg.norm(u_leader[:, t])**2

    return cost_leader

def leader_constraints(variables, *args):
    x_leader = variables[:2*(N+1)].reshape(2, N+1)
    u_leader = variables[2*(N+1):2*(N+1) + 2*N].reshape(2, N)

    Q, R, A, B, hostage_position, bot_position, max_velocity = args

    constraints = []

    # Leader constraints
    for t in range(N):
        constraints.append(A @ x_leader[:, t] + B @ u_leader[:, t] - x_leader[:, t+1])



    constraints.append(x_leader[:, 0] - bot_position)

    return np.concatenate(constraints)

# Loop until the bot is sufficiently close to the hostage or reaches max_iterations
for i in range(max_iterations):
    # Define the initial values for the optimization variables
    initial_values = np.zeros(2*(N+1) + 2*N)

    # Pack the required arguments for the cost function and constraints
    args = (Q, R, A, B, hostage_position, bot_position, max_velocity)

    # Define the bounds for the optimization variables
    bounds = [(None, None)] * (2*(N+1) + 2*N)

    # Define the constraint dictionary
    constraint_dict = {'type': 'eq', 'fun': leader_constraints, 'args': args}

    # Solve the optimization problem using scipy.optimize.minimize
    result = minimize(leader_cost_function, initial_values, args=args, method='SLSQP', bounds=bounds, constraints=constraint_dict)

    # Extract the optimal variables from the result
    optimal_variables = result.x
    x_leader_opt = optimal_variables[:2*(N+1)].reshape(2, N+1)
    u_leader_opt = optimal_variables[2*(N+1):2*(N+1) +2*N].reshape(2, N)
    # Apply the optimal control inputs to the system and update the system states
    next_bot_position = A @ bot_position + B @ u_leader_opt[:, 0]

    # Store the updated bot positions
    bot_positions.append(next_bot_position)

    # Check if the bot is sufficiently close to the hostage
    if np.linalg.norm(next_bot_position - hostage_position) < tolerance:
        print(f'Bot reached the hostage after {i + 1} iterations')
        break

    # Update the bot position for the next iteration
    bot_position = next_bot_position

# Apply the optimal control inputs to the system and update the system states
next_bot_position = A @ bot_position + B @ u_leader_opt[:, 0]

# Store the updated bot positions
bot_positions.append(next_bot_position)


bot_positions = np.array(bot_positions)
# Calculate the final cost value
final_cost = leader_cost_function(optimal_variables, *args)
print(f'Final cost: {final_cost}')

plt.figure()
plt.plot(hostage_position[0], hostage_position[1], 'ro', label='Hostage')
plt.plot(bot_positions[:, 0], bot_positions[:, 1], 'b.-', label='Leader path')
plt.legend()
plt.xlabel('x')
plt.ylabel('y')
plt.title('Bot and Hostage Positions')
plt.grid()
plt.show()
