import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation


g = 9.81            
mass_rocket = 1.0  
mass_cart = 1.0     
length = 1.0        
dt = 0.02          


def update_physics(state, force):
   
    x, x_dot, theta, theta_dot = state
    
   
    temp = (force + mass_rocket * length * theta_dot**2 * np.sin(theta)) / (mass_rocket + mass_cart)
    theta_acc = (g * np.sin(theta) - np.cos(theta) * temp) / (length * (4.0/3.0 - mass_rocket * np.cos(theta)**2 / (mass_rocket + mass_cart)))
    x_acc = temp - mass_rocket * length * theta_acc * np.cos(theta) / (mass_rocket + mass_cart)

    
    x = x + x_dot * dt
    x_dot = x_dot + x_acc * dt
    theta = theta + theta_dot * dt
    theta_dot = theta_dot + theta_acc * dt
    
    return [x, x_dot, theta, theta_dot]


def get_control_action(state):
    x, x_dot, theta, theta_dot = state
    
    k_theta = 30.0
    k_theta_dot = 20.0
    k_x = 3.0   
    k_x_dot = 5.0 
    
    
    force = (k_theta * theta) + (k_theta_dot * theta_dot) + (k_x * x) + (k_x_dot * x_dot)
    
    return force


state = [0.0, 0.0, 0.1, 0.0]  
history_theta = []

fig, ax = plt.subplots()
ax.set_xlim(-2, 2)
ax.set_ylim(-0.5, 2)
cart, = ax.plot([], [], 'ks', markersize=20) 
pole, = ax.plot([], [], 'k-', linewidth=4)   
time_text = ax.text(0.05, 0.9, '', transform=ax.transAxes)

def animate(i):
    global state
    
    
    force = get_control_action(state)
    
    
    state = update_physics(state, force)
    
    
    x = state[0]
    theta = state[2]
    
    
    pole_x = [x, x + length * np.sin(theta)]
    pole_y = [0, length * np.cos(theta)]
    
    cart.set_data([x], [0])
    pole.set_data(pole_x, pole_y)
    time_text.set_text(f"Tilt: {np.degrees(theta):.2f} deg")
    
    if abs(theta) > 0.5: 
        state = [0.0, 0.0, np.random.uniform(-0.1, 0.1), 0.0]

ani = animation.FuncAnimation(fig, animate, interval=20)
plt.title("Pendulum Simulation")
plt.grid(True)
plt.show()