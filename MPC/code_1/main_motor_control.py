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
hz=support.constants['hz'] # horizon period
Nu=support.constants['Nu'] # control horizon
act_penalty=support.constants['R_weight']

# Generate the reference signals
t=np.arange(0,time_length+Ts,Ts) # time from 0 to 10 seconds, sample time (Ts=1 second)
reference=20 #[rad/s]
outputs=1
sim_length=len(t) # Number of control loop iterations
refSignals=np.zeros(sim_length*outputs)

#Build up the reference signal vector:
# refSignal = [20/r, 20/r, 20/r, .... etc.]
for i in range(0,len(refSignals),outputs):
    refSignals[i]=reference


# Load the initial states
w=0 # angular velocity
I=0 # current

states=np.array([w,I], dtype=np.float64)
statesTotal=np.zeros((len(t),len(states))) # It will keep track of all your states during the entire manoeuvre
statesTotal[0][0:len(states)]=states
v_opt_total=np.zeros((len(t),hz))

# Load the initial input
Vin=0 # Initial Voltage input
UTotal=np.zeros(len(t)) # To keep track all your inputs over time
UTotal[0]=Vin

# To extract w_opt from predicted x_aug_opt
C_w_opt=np.zeros((hz,(len(states)+np.size(Vin))*hz))
n_vars = 3 # Number of variables per time step
for i in range(hz):
    # i is the row (the time step)
    # i * n_vars is the starting index of that time step's block
    C_w_opt[i][i * n_vars] = 1

# Generate the discrete state space matrices
Ad,Bd,Cd,Dd=support.state_space()


# Generate the compact simplification matrices for the cost function
# The matrices (H, F) stay mostly constant during the simulation.
# Therefore, it is more efficient to generate them here before you start the simulation loop.
# However, in the end of the simulation, the horizon period (hz) will start decreasing.
# That is when the matrices need to be regenerated (done inside the simulation loop)
H,F=support.mpc_simplification(Ad,Bd,Cd,Dd,hz)
m=1 #no. of inputs
H_tilde = H[:, :Nu*m]
# Initiate the controller - simulation loops
k=0
for i in range(0,sim_length-1):

    # Generate the augmented current state and the reference vector
    x_aug_t=np.transpose([np.concatenate((states,[Vin]),axis=0)])

    # From the refSignals vector, only extract the reference values from your [current sample (NOW) + Ts] to [NOW+horizon period (hz)]
    # Example: t_now is 3 seconds, hz = 15 samples, so from refSignals vectors, you move the elements to vector r:
    # r=[w_ref_3.1, w_ref_3.2, ... , w_ref_4.5]
    # With each loop, it all shifts by 0.1 second because Ts=0.1 s
    k=k+outputs
    if k+outputs*hz<=len(refSignals):
        r=refSignals[k:k+outputs*hz]
    else:
        r=refSignals[k:len(refSignals)]
        hz=hz-1

    if hz<constants['hz']: # Check if hz starts decreasing
        # These matrices (H,F) were created earlier at the beginning of the loop.
        # They constant almost throughout the entire simulation. However,
        # in the end of the simulation, the horizon period (hz) starts decreasing.
        # Therefore, the matrices need to be constantly updated in the end of the simulation.
        H,F=support.mpc_simplification(Ad,Bd,Cd,Dd,hz)
        m=1 #no. of inputs
        H_tilde = H[:, :Nu*m]

    H_tilde_t = np.transpose(H_tilde)
    HH=np.matmul(H_tilde_t, H_tilde)
    lamdaI=act_penalty*np.eye(np.size(HH, 0))
    Adb=HH + lamdaI
    Adb_inv=np.linalg.inv(Adb)

    r_t=np.transpose([r])
    Fx=np.matmul(F, x_aug_t)
    error=r_t-Fx
    Herror=np.matmul(H_tilde_t, error)

    du=np.matmul(Adb_inv, Herror)
    Vin=Vin+du[0][0]  # Update the real inputs
    UTotal[i+1]=Vin # Keep track of your inputs as you go from t=0 --> t=10 seconds

    y_aug_opt=np.matmul(F,x_aug_t)+np.matmul(H_tilde,du)
    v_opt_total[i+1][0:hz] = np.transpose(y_aug_opt)

    # Compute new states in the open loop system
    states=support.open_loop_new_states(states,Vin)
    statesTotal[i+1][0:len(states)]=states

# --- Plotting ---
plt.figure(figsize=(10, 5))

# Plot Reference vs Actual speed
plt.plot(t, refSignals, 'r--', linewidth=2, label='Reference Speed')
plt.plot(t, v_opt_total[:, 0], 'b-', linewidth=2, label='Actual Speed (Controlled)')

plt.ylabel('Velocity (m/s)')
plt.xlabel('Time (s)')
plt.title('Closed-Loop Speed Control (Frictionless Motor)')
plt.legend()
plt.grid(True)

plt.tight_layout()
plt.show()

# Plot the the input Vin
plt.plot(t, UTotal[:], 'b-', linewidth=2, label='Input Voltage')

plt.ylabel('Vin (V)')
plt.xlabel('Time (s)')
plt.title('Closed-Loop Speed Control (Frictionless Motor)')
plt.legend()
plt.grid(True)

plt.tight_layout()
plt.show()








    

