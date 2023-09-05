# 再度必要な関数と変数の設定
import math
import matplotlib.pyplot as plt

# Parameters
R = 10  # radius of the circle
v = 2 * 5 * R  # linear velocity
omega = 10  # angular velocity, now doubled
delta_t = 0.01  # time step in seconds

def simulate_robot_motion(total_time, delta_t, v, omega):
    x = 0
    y = 0
    theta = 0

    x_values = [x]
    y_values = [y]

    # For arrow plotting
    arrow_x = []
    arrow_y = []
    arrow_dx = []
    arrow_dy = []

    for t in range(int(total_time / delta_t)):
        x += v * math.cos(theta) * delta_t
        y += v * math.sin(theta) * delta_t
        theta += omega * delta_t
        theta = (theta + math.pi) % (2 * math.pi) - math.pi

        x_values.append(x)
        y_values.append(y)

        # Every 0.5 seconds, we'll store values for arrow plotting
        if t % 50 == 0:
            arrow_x.append(x)
            arrow_y.append(y)
            arrow_dx.append(math.cos(theta))
            arrow_dy.append(math.sin(theta))

    return x_values, y_values, arrow_x, arrow_y, arrow_dx, arrow_dy

def plot_robot_trajectory(x_values, y_values, arrow_x, arrow_y, arrow_dx, arrow_dy):
    plt.figure(figsize=(10, 10))
    plt.plot(x_values, y_values, label='Robot trajectory')
    plt.quiver(arrow_x, arrow_y, arrow_dx, arrow_dy, angles='xy', scale_units='xy', scale=1, color='r', label="Robot's direction")
    plt.gca().set_aspect('equal', adjustable='box')
    plt.title('Robot Trajectory')
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    plt.legend()
    plt.grid(True)
    plt.show()

# Simulate and plot the trajectory
x_values, y_values, arrow_x, arrow_y, arrow_dx, arrow_dy = simulate_robot_motion(2, delta_t, v, omega)
plot_robot_trajectory(x_values, y_values, arrow_x, arrow_y, arrow_dx, arrow_dy)
