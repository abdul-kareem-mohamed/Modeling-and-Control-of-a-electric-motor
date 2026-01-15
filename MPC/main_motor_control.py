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

# Generate the reference signals
t=np.arange(0,time_length+Ts,Ts) # time from 0 to 10 seconds, sample time (Ts=1 second)
reference=20/r #[rad/s]
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
w_opt_total=np.zeros((len(t),hz))

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
# The matrices (Hdb,Fdbt,Cdb,Adc) stay mostly constant during the simulation.
# Therefore, it is more efficient to generate them here before you start the simulation loop.
# However, in the end of the simulation, the horizon period (hz) will start decreasing.
# That is when the matrices need to be regenerated (done inside the simulation loop)
Hdb,Fdbt,Cdb,Adc=support.mpc_simplification(Ad,Bd,Cd,Dd,hz)

# Initiate the controller - simulation loops
k=0
for i in range(0,sim_length-1):

    # Generate the augmented current state and the reference vector
    x_aug_t=np.transpose([np.concatenate((states,[Vin]),axis=0)]) # X_aug_k_transpose

    # From the refSignals vector, only extract the reference values from your [current sample (NOW) + Ts] to [NOW+horizon period (hz)]
    # Example: t_now is 3 seconds, hz = 3 samples, so from refSignals vectors, you move the elements to vector r:
    # r=[w_3.01, w_3.02, w_3.03]
    # With each loop, it all shifts by 0.01 second because Ts=0.01 s
    k=k+outputs
    if k+outputs*hz<=len(refSignals):
        r=refSignals[k:k+outputs*hz]
    else:
        r=refSignals[k:len(refSignals)]
        hz=hz-1

    if hz<constants['hz']: # Check if hz starts decreasing
        # These matrices (Hdb,Fdbt,Cdb,Adc) were created earlier at the beginning of the loop.
        # They constant almost throughout the entire simulation. However,
        # in the end of the simulation, the horizon period (hz) starts decreasing.
        # Therefore, the matrices need to be constantly updated in the end of the simulation.
        Hdb,Fdbt,Cdb,Adc=support.mpc_simplification(Ad,Bd,Cd,Dd,hz)

    ft=np.matmul(np.concatenate((np.transpose(x_aug_t)[0][0:len(x_aug_t)],r),axis=0),Fdbt)
    du=-np.matmul(np.linalg.inv(Hdb),np.transpose([ft]))
    x_aug_opt=np.matmul(Cdb,du)+np.matmul(Adc,x_aug_t)
    w_opt=np.matmul(C_w_opt[0:hz,0:(len(states)+np.size(Vin))*hz],x_aug_opt)

    w_opt=np.transpose((w_opt))[0]
    w_opt_total[i+1][0:hz]=w_opt

    # Update the real inputs
    Vin=Vin+du[0][0]

    # Keep track of your inputs as you go from t=0 --> t=7 seconds
    UTotal[i+1]=Vin

    # Compute new states in the open loop system (interval: Ts)
    states=support.open_loop_new_states(states,Vin)
    statesTotal[i+1][0:len(states)]=states

# --- 5. Plotting (Velocity Only) ---
plt.figure(figsize=(10, 5))

# Plot Reference vs Actual
plt.plot(t, refSignals, 'r--', linewidth=2, label='Reference Speed')
plt.plot(t, statesTotal[:,1], 'b-', linewidth=2, label='Actual Speed (Controlled)')

plt.ylabel('Angular Velocity (rad/s)')
plt.xlabel('Time (s)')
plt.title('Closed-Loop Speed Control (Frictionless Motor)')
plt.legend()
plt.grid(True)

plt.tight_layout()
plt.show()
