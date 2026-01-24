import numpy as np
import scipy.linalg as lin
import matplotlib.pyplot as plt

class LQController:
    def __init__(self):
        # 1. Define System Matrices (DC Motor)
        # Constants
        km = 0.05; R = 1.0; L = 0.5; J = 0.01; r = 0.03
        
        self.A = np.array([[0, km/J], 
                           [-km/L, -R/L]])
        
        self.B = np.array([[0], 
                           [1/L]])
        
        self.C = np.array([[r, 0]])  # Output is Speed

    
    def solve_are_hamiltonian(self, A, B, Q, R):
        """
        Solves the Algebraic Riccati Equation using the Hamiltonian Method.
        Returns the solution matrix P.
        """
        n = A.shape[0]

        # 1. Compute the coupling matrix S = B * inv(R) * B.T
        R_inv = np.linalg.inv(R)
        S = B @ R_inv @ B.T

        # 2. Construct the Hamiltonian Matrix Z
        # Structure: [[ A, -S ],
        #             [-Q, -A.T]]
        
        # Create rows first
        top_row = np.hstack((A, -S))
        bottom_row = np.hstack((-Q, -A.T))
        
        # Stack them vertically
        Z = np.vstack((top_row, bottom_row))

        # 3. Perform Eigendecomposition
        vals, vecs = np.linalg.eig(Z)

        # 4. Sort and Select Stable Eigenvectors
        # We need the eigenvectors where the real part of the eigenvalue is < 0.
        
        # Get indices of eigenvalues with negative real parts
        # Note: For a solvable system, there should be exactly n such values.
        stable_indices = [i for i, v in enumerate(vals) if v.real < 0]
        
        if len(stable_indices) != n:
            raise ValueError("Matrix Z does not have a stable splitting. Check controllability/observability.")

        # Extract the corresponding n eigenvectors (columns)
        U = vecs[:, stable_indices]

        # 5. Partition U into U1 (top n rows) and U2 (bottom n rows)
        U1 = U[:n, :]
        U2 = U[n:, :]

        # 6. Compute P = U2 * inv(U1)
        try:
            inv_U1 = np.linalg.inv(U1)
            P = U2 @ inv_U1
        except np.linalg.LinAlgError:
            raise ValueError("U1 is singular; solution cannot be computed via this method.")

        # The result may contain tiny imaginary parts (numerical noise) -> return Real part only
        return np.real(P)
        
    def design_lqr(self, Q_penalty, R_penalty):
        """
        Implements the 3 steps from your slide:
        1. Choice of Q and R (passed as arguments)
        2. Solving Matrix Riccati Equation (for P)
        3. Calculation of optimal K
        """
        print(f"--- Designing LQ Controller ---")
        
        # Step 1: Format Q and R matrices
        # Q penalizes state error (speed error, current error)
        # R penalizes actuator effort (voltage)
        Q = np.array(Q_penalty)
        R = np.matrix(str(R_penalty)) # R must be 1x1 or mxm
        
        print(f"Weight Q (State Penalty):\n{Q}")
        print(f"Weight R (Control Penalty): {R_penalty}")

        # Step 2: Solve the Continuous Algebraic Riccati Equation (CARE)
        # Equation: A.T*P + P*A - P*B*inv(R)*B.T*P + Q = 0
        P = self.solve_are_hamiltonian(self.A, self.B, Q, R)
        
        print(f"\nSolution Matrix P (from Riccati Eq):\n{P}")

        # Step 3: Calculate Optimal Gain K
        # Formula: K = inv(R) * B.T * P
        R_inv = np.linalg.inv(R)
        K = R_inv @ self.B.T @ P
        
        print(f"\nOptimal Gain Vector K: {K}")
        
        # Calculate Closed Loop Eigenvalues (Poles) to see where LQR placed them
        A_cl = self.A - (self.B @ K)
        poles = np.linalg.eigvals(A_cl)
        print(f"Resulting Closed-Loop Poles: {poles}")
        
        # Calculate Pre-filter v for reference tracking
        # v = -1 / (C * inv(A - BK) * B)
        v = -1.0 / (self.C @ np.linalg.inv(A_cl) @ self.B).item()
        print(f"Pre-filter Gain v: {v:.4f}")
        
        return K, v

    def simulate(self, K, v):
        # Time setup
        t = np.linspace(0, 2, 1000)
        dt = t[1] - t[0]
        
        x = np.array([[0.0], [0.0]]) # Initial state
        ref = 100.0 # Target Speed
        
        y_log = []
        u_log = []
        
        for _ in t:
            # Control Law: u = v*r - K*x
            u = (v * ref) - (K @ x)
            
            # System Dynamics: dx = Ax + Bu
            dx = (self.A @ x) + (self.B * u)
            
            # Integration
            x = x + dx * dt
            y = self.C @ x
            
            y_log.append(y.item())
            u_log.append(u.item())
            
        return t, y_log, u_log

# --- Main Execution ---

motor = LQController()

# --- STEP 1: Choose Q and R ---
# Tuning Strategy:
# High Q (e.g., 100) -> Aggressive performance, faster speed tracking
# High R (e.g., 10)  -> Save energy, lower voltage, slower response

# Q is 2x2 because we have 2 states [velocity, current]
# We put a high penalty (10) on velocity (index 0,0) to prioritize speed tracking.
# We put a low penalty (0.01) on current (index 1,1).
Q_choice = np.matrix('10 0;0 0.01')

R_choice = 0.1  # Low penalty on voltage = "Use as much voltage as needed"

# --- Run Design ---
K_opt, v_opt = motor.design_lqr(Q_choice, R_choice)

# --- Simulate ---
t, y, u = motor.simulate(K_opt, v_opt)

plt.figure(figsize=(10, 8))
plt.subplot(2,1,1)
plt.plot(t, y, label='Speed Output')
plt.plot(t, [100]*len(t), 'r--', label='Reference')
plt.title(f"LQR Step Response (Q[0,0]={Q_choice[0][0]}, R={R_choice})")
plt.grid(True)
plt.legend()

plt.subplot(2,1,2)
plt.plot(t, u, 'g', label='Control Voltage')
plt.title("Control Effort (Optimal)")
plt.xlabel("Time (s)")
plt.grid(True)
plt.tight_layout()
plt.show()
