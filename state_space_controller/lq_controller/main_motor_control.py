import numpy as np
import matplotlib.pyplot as plt
import support_files_motor as sfm

# Create an object for the support functions
support=sfm.SupportFilesMotor()
constants=support.constants

# Load the constant values needed in the main file
km=support.constants['motor_constant']
R=support.constants['R']
L=support.constants['L']
r=support.constants['r'] # radius of the wheel
J=support.constants['J']
Ts=support.constants['Ts']
time_length=support.constants['time_length'] # duration of the acceleration

# Generate the time steps
time=np.arange(0,time_length+Ts,Ts) # time from 0 to 10 seconds, sample time (Ts=1 second)

# Load the initial states
w=0 # angular velocity
I=0 # current

states=np.array([[w],
                [I]])
# statesTotal=np.zeros((len(time),len(states))) # It will keep track of all your states during the entire manoeuvre
# statesTotal[0][0:len(states)]=states

# Reference Input (Step input to 20 m/s)
ref_input = 20

# Data logging for plotting
y_log = []
u_log = []

A, B, C, D = support.state_space()
k_transpose=support.design_lqr(A, B)
vf=support.calculate_prefilter(A,B,C,k_transpose)

for t in time:
        # A. MEASURE: Get current state (in real life, from sensors)
        # x is available directly here
        
        # B. CONTROL LAW: u = (v * r) - (K * states)
        # (v * ref) is the feedforward part
        # (K @ states) is the feedback part
        # even tho, K seems constant here and this seems like a proportional controller,
        # In reality, this is a PD-controller
        feedforward = vf * ref_input
        feedback = k_transpose @ states
        u = feedforward - feedback
        
        # Limit voltage (optional saturation, e.g., +/- 12V)
        # u = np.clip(u, -12, 12)

        # C. ACTUATE: Update system state
        # dx/dt = Ax + Bu
        # x_new = x_old + dx/dt * dt  (Forward Euler Integration)
        # When I discretize the system matrices (forward euler or zero-order hold), then the eqn changes:
        # x(k+1)=(Ad @ x) + (Bd * u)
        dx = (A @ states) + (B * u)
        states = states + (dx * Ts)
        
        # D. OUTPUT: Calculate y = Cx + Du
        y = (C @ states) + (D * u)
        
        # Log data
        y_log.append(y.item())
        u_log.append(u.item())

# --- Visualization ---
plt.figure(figsize=(10, 8))

# Plot 1: Output Response
plt.subplot(2, 1, 1)
plt.plot(time, y_log, label='Output Speed (m/s)', linewidth=2)
plt.axhline(y=ref_input, color='r', linestyle='--', label='Reference (100)')
plt.title(f'Output Response with Pre-filter (v={vf:.2f})')
plt.ylabel('Speed (rad/s)')
plt.grid(True)
plt.legend()

# Plot 2: Control Effort
plt.subplot(2, 1, 2)
plt.plot(time, u_log, color='g', label='Control Voltage (V)')
plt.title('Control Effort')
plt.xlabel('Time (s)')
plt.ylabel('Voltage (V)')
plt.grid(True)
plt.legend()

plt.tight_layout()
plt.show()










    