import numpy as np
import matplotlib.pyplot as plt

def calculate_prefilter(A, B, K, C):
    """
    Calculates the static pre-filter gain v to ensure unity DC gain.
    Formula: v = -1 / (C * inv(A - B*K) * B)
    """
    # Closed loop system matrix A_cl = A - B*K
    A_cl = A - (B @ K)
    
    # DC Gain of closed loop system
    try:
        inv_A_cl = np.linalg.inv(A_cl)
        dc_gain = C @ inv_A_cl @ B
        v = -1.0 / dc_gain.item()
    except np.linalg.LinAlgError:
        print("Warning: Matrix singular, setting pre-filter to 1")
        v = 1.0
        
    return v

def run_simulation():
    # --- 1. System Definitions (DC Motor Example) ---
    # x1 = Angular Velocity, x2 = Current
    # Parameters
    km = 0.05; R = 1.0; L = 0.5; J = 0.01
    
    # Matrices A, B, C, D
    A = np.array([[0, km/J], 
                  [-km/L, -R/L]])
    
    B = np.array([[0], 
                  [1/L]])
    
    C = np.array([[1, 0]]) # Output is Angular Velocity (x1)
    D = np.array([[0]])

    # Controller Gain K (transpose)
    # Let's assume we designed this for poles at [-10, -11]
    # You can replace this with your calculated 'k_transpose'
    # K = [k1, k2]
    K = np.array([[1.8, 0.45]]) 

    # --- 2. Calculate Pre-filter ---
    # This scales the reference so steady-state error is zero
    v = calculate_prefilter(A, B, K, C)
    print(f"Calculated Pre-filter Gain (v): {v:.4f}")

    # --- 3. Simulation Setup ---
    t_end = 2.0         # Simulation duration (seconds)
    dt = 0.001          # Time step (seconds)
    time = np.arange(0, t_end, dt)
    
    # Initial State vector x = [0, 0] (Stopped)
    x = np.array([[0.0], 
                  [0.0]]) 
    
    # Reference Input (Step input to 100 rad/s)
    ref_input = 100.0 
    
    # Data logging for plotting
    y_log = []
    u_log = []
    
    # --- 4. The Control Loop ---
    for t in time:
        # A. MEASURE: Get current state (in real life, from sensors)
        # x is available directly here
        
        # B. CONTROL LAW: u = (v * r) - (K * x)
        # This is the equation from your block diagram
        # (v * ref) is the feedforward part
        # (K @ x) is the feedback part
        feedforward = v * ref_input
        feedback = K @ x
        u = feedforward - feedback
        
        # Limit voltage (optional saturation, e.g., +/- 12V)
        # u = np.clip(u, -12, 12)

        # C. ACTUATE / PHYSICS: Update system state
        # dx/dt = Ax + Bu
        # x_new = x_old + dx/dt * dt  (Forward Euler Integration)
        dx = (A @ x) + (B * u)
        x = x + (dx * dt)
        
        # D. OUTPUT: Calculate y = Cx + Du
        y = (C @ x) + (D * u)
        
        # Log data
        y_log.append(y.item())
        u_log.append(u.item())

    # --- 5. Visualization ---
    plt.figure(figsize=(10, 8))
    
    # Plot 1: Output Response
    plt.subplot(2, 1, 1)
    plt.plot(time, y_log, label='Output Speed (rad/s)', linewidth=2)
    plt.axhline(y=ref_input, color='r', linestyle='--', label='Reference (100)')
    plt.title(f'Step Response with Pre-filter (v={v:.2f})')
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

# Run it
if __name__ == "__main__":
    run_simulation()
