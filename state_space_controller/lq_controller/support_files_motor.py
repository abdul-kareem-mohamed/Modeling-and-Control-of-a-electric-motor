import numpy as np
import matplotlib.pyplot as plt
from scipy.linalg import expm
from numpy.linalg import matrix_power

class SupportFilesMotor:
    ''' The following functions interact with the main file '''

    def __init__(self):
        '''Load the constants that do not change'''

        # constants
        motor_constant=1.25 #[N*m/A]
        R=0.5 #[ohm]
        L=0.01 #[H]
        r=0.25 #[m]
        m=80 #[kg]
        J=m*r**2 #[kg*m**2]
        Ts=0.01 # sampling time
        time_length = 10 # [s] - duration of the entire manoeuvre

        # Q penalizes state error (speed error, current error)
        # R penalizes actuator effort (voltage)
        Q_penalty = np.matrix('10 0; 0 0.01')
        R_penalty = np.matrix('0.1') # R must be 1x1 or mxm

        self.constants={'motor_constant':motor_constant, 'R':R, 'L':L, 'r':r, 'J':J,\
                        'Ts':Ts, 'time_length':time_length, 'Q_penalty':Q_penalty, 'R_penalty':R_penalty}

        return None
        
    def state_space(self):
        '''This function forms the state space matrices and transforms them in the discrete form'''

        # Get the necessary constants
        km=self.constants['motor_constant']
        R=self.constants['R']
        L=self.constants['L']
        r=self.constants['r']
        J=self.constants['J']
        Ts = self.constants['Ts']

        # Get the state space matrices for the control
        A=np.array([[0, km/J], [-km/L, -R/L]])
        B=np.array([[0], [1/L]])
        Ct=np.array([[r, 0]])
        D=0

        return A, B, Ct, D
    
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
    
    def design_lqr(self, A, B):

        # choice of matrix for penalising states and actuating effort
        Q=self.constants['Q_penalty']
        R=self.constants['R_penalty']

        print(f"Weight Q (State Penalty):\n{Q}")
        print(f"Weight R (Control Penalty): {R}")

        # Solving the matrix riccati equation
        # Equation: A.T*P + P*A - P*B*inv(R)*B.T*P + Q = 0
        P=self.solve_are_hamiltonian(A, B, Q, R)

        print(f"\nSolution Matrix P (from Riccati Eq):\n{P}")

        # calculation of optimal state-space controller k_transpose
        # Formula: K = inv(R) * B.T * P
        R_inv = np.linalg.inv(R)
        K_transpose = R_inv @ B.T @ P

        print(f"\nOptimal Gain Vector K: {K_transpose}")

        # Calculate Closed Loop Eigenvalues (Poles) to see where LQR placed them
        A_cl = A - (B @ K_transpose)
        poles = np.linalg.eigvals(A_cl)
        print(f"Resulting Closed-Loop Poles: {poles}")

        return K_transpose

    def calculate_prefilter(self, A, b, C, k_transpose):
        """
        Calculates the pre-filter gain v using the formula:
        v = 1 / ( c.T * inv(-A + b*k.T) * b )
        
        Parameters:
        A : (n, n) System matrix
        b : (n, 1) Input matrix
        k : (1, n) or (n,) Controller gain vector
        C : (1, n) Output matrix
        """
        
        # 1. Ensure 'b' is a column vector (n, 1)
        # If it's a 1D array, reshape it.
        if b.ndim == 1:
            b = b.reshape(-1, 1)

        # 2. Ensure 'k' is a row vector (1, n)
        # This is critical for the outer product (b @ k) to create an (n, n) matrix.
        if k_transpose.ndim == 1:
            k_transpose = k_transpose.reshape(1, -1)
            
        # 3. Compute the term inside the parenthesis: (-A + b * k^T)
        # Note: In standard matrix math, if k is a row vector, we write b @ k.
        # The formula writes k^T assuming k is a column vector originally.
        # The goal is to get the outer product (n x 1) * (1 x n) = (n x n).
        term_inside = -A + (b @ k_transpose)
        
        # 4. Compute the inverse
        try:
            term_inv = np.linalg.inv(term_inside)
        except np.linalg.LinAlgError:
            print("Error: Matrix is singular and cannot be inverted.")
            return None

        # 5. Compute the denominator: C * inv * b
        # Note: The image writes c^T, which represents the standard row output matrix C.
        denominator = C @ term_inv @ b
        
        # 6. Calculate v
        v = 1.0 / denominator
        
        print(f"Pre-filter Gain v: {v.item():.4f}")

        # Return as a simple scalar float
        return v.item()


# support=SupportFilesMotor()
# A, B, C, D = support.state_space()
# K_transpose=support.design_lqr(A, B)
# vf=support.calculate_prefilter(A,B,C,K_transpose)
# print(vf)
# print("A: ")
# print(Ac)
# print("\n")
# print("B: ")
# print(Bc)
# print("\n")
# print("Ct: ")
# print(Cc)
# print("\n")
# print("Dd: ")
# print(Dd)
# print("\n")
