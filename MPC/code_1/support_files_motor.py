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
        Nu=2 #control horizon

        R_weight=0.1 # weights for inputs (only 1 input in our case)

        hz=10 #horizon period

        self.constants={'motor_constant':motor_constant, 'R':R, 'L':L, 'r':r, 'J':J,\
                        'Ts':Ts, 'time_length':time_length, 'hz':hz, 'Q':Q, 'S':S, 'R_weight':R_weight,\
                            'Nu':Nu}

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

        # Discretise the system - zero-order hold method
        Ad, Bd = self.discretize_zoh(A, B, Ts)
        Cdt=Ct
        Dd=D

        return Ad, Bd, Cdt, Dd
    
    def mpc_simplification(self, Ad, Bd, Cdt, Dd, hz):
        '''This function creates the compact matrices for Model Predictive Control'''

        A_aug=np.concatenate((Ad,Bd),axis=1)
        temp1=np.zeros((np.size(Bd,1),np.size(Ad,1)))
        temp2=np.identity(np.size(Bd,1))
        temp=np.concatenate((temp1,temp2),axis=1)

        A_aug=np.concatenate((A_aug,temp),axis=0)
        B_aug=np.concatenate((Bd,np.identity(np.size(Bd,1))),axis=0)
        C_aug=np.concatenate((Cdt,np.zeros((np.size(Cdt,0),np.size(Bd,1)))),axis=1)
        D_aug=Dd

        # --- Calculate F Matrix ---
        # F = [ C_tilde * A_tilde ]
        #     [ C_tilde * A_tilde^2 ]
        #     ...
        F_list = []
        for k in range(1, hz + 1):
            F_row = C_aug @ matrix_power(A_aug, k)
            F_list.append(F_row)

        F = np.vstack(F_list)

        # --- Calculate H Matrix ---
        # H is a Lower Triangular Toeplitz Matrix
        # First column term: C_tilde * A_tilde^k * B_tilde (where power starts at 0 for diagonal)
        H = np.zeros((hz, hz))

        for i in range(hz):       # Rows (Future Time Steps)
            for j in range(hz):   # Cols (Future Inputs)
                if j <= i:
                    power = i - j
                    
                    # If power is 0, A^0 = I, so term is C_tilde * B_tilde
                    # If power > 0, term is C_tilde * A^power * B_tilde
                    if power == 0:
                        term = C_aug @ B_aug
                    else:
                        term = C_aug @ matrix_power(A_aug, power) @ B_aug
                    
                    H[i, j] = term.item() # .item() converts 1x1 array to scalar

        return H,F
    
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

