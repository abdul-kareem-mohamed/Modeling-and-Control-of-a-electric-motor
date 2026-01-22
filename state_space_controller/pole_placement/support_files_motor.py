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

        self.constants={'motor_constant':motor_constant, 'R':R, 'L':L, 'r':r, 'J':J,\
                        'Ts':Ts, 'time_length':time_length}

        return None
    

    def discretize_zoh(self, A, B, T):
        """
        Discretize a continuous-time system (A, B) using Zero-Order Hold.
        
        Parameters:
        A, B : ndarray (Continuous-time matrices)
        T    : float (Sampling period)
        """
        n = A.shape[0]
        m = B.shape[1]
        
        # 1. Construct the augmented matrix M
        # [ A  B ]
        # [ 0  0 ]
        upper_row = np.hstack([A, B])
        lower_row = np.zeros((m, n + m))
        M = np.vstack([upper_row, lower_row])
        
        # 2. Compute the matrix exponential of (M * T)
        Phi = expm(M * T)
        
        # 3. Extract Ad and Bd from the resulting matrix
        Ad = Phi[:n, :n]
        Bd = Phi[:n, n:]
        
        return Ad, Bd
        
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
    
    def get_controllability_matrix(self, A, B):
        """ Computes the Controllability Matrix Ss = [B, AB, A^2B, ... A^(n-1)B] """
        # 1. Determine system order 'n' from matrix A (n x n)
        n = A.shape[0]
        
        # 2. Initialize the matrix with the first column: B
        Ss = B
        
        # 3. Loop to calculate subsequent columns (AB, A^2B, etc.)
        # The image formula is: (b, Ab, ..., A^(n-1)b)
        for i in range(1, n):
            # Calculate A^i * B
            AiB = np.linalg.matrix_power(A, i) @ B
            
            # Stack the new column horizontally to the right
            Ss = np.hstack((Ss, AiB))
            
        return Ss
    
    def transform_to_ccf(self, A, B, C):
        '''This function transforms the original system to controllable canonical form'''

        controllability_matrix=self.get_controllability_matrix(A, B)
        inv_controllability_matrix=np.linalg.inv(controllability_matrix)
        sst=inv_controllability_matrix[np.size(inv_controllability_matrix,0)-1, :]

        n=A.shape[0]
        inv_T=sst
        for i in range(1, n):
            Ai_ss=sst @ np.linalg.matrix_power(A, i)

            inv_T=np.vstack((inv_T, Ai_ss))
        
        T=np.linalg.inv(inv_T)
        Ac=inv_T @ A @ T
        Bc=inv_T @ B
        Cc=C @ T

        return Ac, Bc, Cc, inv_T
    


    def calculate_canonical_gain(self, A, pole1, pole2):
        # 1. Define Desired Poles (from the image text)
        
        
        # 2. Form the Desired Characteristic Polynomial
        # (s - lambda1)(s - lambda2) = s^2 - (lambda1 + lambda2)s + (lambda1 * lambda2)
        # Example: (s + 50)(s + 2) = s^2 + 52s + 100
        # We can use np.poly to get coefficients automatically: [1, 52, 100]
        desired_poly = np.poly([pole1, pole2])
        
        # Extract coefficients (alpha1, alpha0) corresponding to s^1 and s^0
        # desired_poly[0] is s^2 coefficient (1)
        # desired_poly[1] is s^1 coefficient (52)
        # desired_poly[2] is s^0 coefficient (100)
        alpha1 = desired_poly[1]  # 52
        alpha0 = desired_poly[2]  # 100
        
        # 3. Define Original System Coefficients
        # Derive system coefficients from A matrix
        
        a={}
        lastrow=A[np.size(A,0)-1][:]
        for i in range(0, np.size(lastrow, 0)):
            a["a"+str(i)]=-lastrow[i]
        
        # 4. Calculate Gains via Coefficient Comparison (The Core Task)
        # Formula: k_desired = coeff_desired - coeff_original
        k_c={}
        i=1
        for var, value in a.items(): 
            k_c["k_"+var]=desired_poly[desired_poly.size-i] - value
            i+=1
        
        # 5. Construct the Gain Vector k_c^T
        # (Usually canonical form gains are ordered from lowest power s^0 to s^n-1)
        k_C_transpose = np.array(list(k_c.values()))
        
        return k_C_transpose
    
    def open_loop_new_states(self,states,U1):
        '''This function computes the new state vector for one sample time later'''

        current_states=states
        new_states=current_states
        w=current_states[0]
        I=current_states[1]

        km=self.constants['motor_constant']
        R=self.constants['R']
        L=self.constants['L']
        r=self.constants['r']
        J=self.constants['J']
        Ts = self.constants['Ts']
        
        i_dot=(U1/L)-(km*w/L)-(R*I/L)
        w_dot=km*I/J

        w=w+w_dot*Ts
        I=I+i_dot*Ts

        new_states[0]=w
        new_states[1]=I

        return new_states
    
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
        
        # Return as a simple scalar float
        return v.item()


# support=SupportFilesMotor()
# A, B, C, D = support.state_space()
# Ac, Bc, Cc, inv_T = support.transform_to_ccf(A, B, C)
# k_C_transpose=support.calculate_canonical_gain(Ac, -50, -2)
# k_transpose=k_C_transpose @ inv_T
# vf=support.calculate_prefilter(A,B,C,k_transpose)
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
