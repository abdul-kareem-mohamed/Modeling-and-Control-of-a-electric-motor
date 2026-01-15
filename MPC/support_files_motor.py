import numpy as np
import matplotlib.pyplot as plt
from scipy.linalg import expm

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
        Ts=0.001 # sampling time
        time_length = 10 # [s] - duration of the entire manoeuvre

        Q=np.matrix('1') # weights for outputs (all samples, except the last one) -  1 outputs, so 1x1 identity
        S=np.matrix('10') # weights for the final horizon period outputs
        R_weight=np.matrix('1') # weights for inputs (only 1 input in our case)

        hz=50 #horizon period

        self.constants={'motor_constant':motor_constant, 'R':R, 'L':L, 'r':r, 'J':J,\
                        'Ts':Ts, 'time_length':time_length, 'hz':hz, 'Q':Q, 'S':S, 'R_weight':R_weight}

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
        # db - double bar
        # dbt - double bar transpose
        # dc - double circumflex

        A_aug=np.concatenate((Ad,Bd),axis=1)
        temp1=np.zeros((np.size(Bd,1),np.size(Ad,1)))
        temp2=np.identity(np.size(Bd,1))
        temp=np.concatenate((temp1,temp2),axis=1)

        A_aug=np.concatenate((A_aug,temp),axis=0)
        B_aug=np.concatenate((Bd,np.identity(np.size(Bd,1))),axis=0)
        C_aug=np.concatenate((Cdt,np.zeros((np.size(Cdt,0),np.size(Bd,1)))),axis=1)
        D_aug=Dd

        Q=self.constants['Q']
        S=self.constants['S']
        R=self.constants['R_weight']

        CQC=np.matmul(np.transpose(C_aug),Q)
        CQC=np.matmul(CQC,C_aug)

        CSC=np.matmul(np.transpose(C_aug),S)
        CSC=np.matmul(CSC,C_aug)

        QC=np.matmul(Q,C_aug)
        SC=np.matmul(S,C_aug)

        Qdb=np.zeros((np.size(CQC,0)*hz,np.size(CQC,1)*hz))
        Tdb=np.zeros((np.size(QC,0)*hz,np.size(QC,1)*hz))
        Rdb=np.zeros((np.size(R,0)*hz,np.size(R,1)*hz))
        Cdb=np.zeros((np.size(B_aug,0)*hz,np.size(B_aug,1)*hz))
        Adc=np.zeros((np.size(A_aug,0)*hz,np.size(A_aug,1)))

        for i in range(0,hz):
            if i == hz-1:
                Qdb[np.size(CSC,0)*i:np.size(CSC,0)*i+CSC.shape[0],np.size(CSC,1)*i:np.size(CSC,1)*i+CSC.shape[1]]=CSC
                Tdb[np.size(SC,0)*i:np.size(SC,0)*i+SC.shape[0],np.size(SC,1)*i:np.size(SC,1)*i+SC.shape[1]]=SC
            else:
                Qdb[np.size(CQC,0)*i:np.size(CQC,0)*i+CQC.shape[0],np.size(CQC,1)*i:np.size(CQC,1)*i+CQC.shape[1]]=CQC
                Tdb[np.size(QC,0)*i:np.size(QC,0)*i+QC.shape[0],np.size(QC,1)*i:np.size(QC,1)*i+QC.shape[1]]=QC

            Rdb[np.size(R,0)*i:np.size(R,0)*i+R.shape[0],np.size(R,1)*i:np.size(R,1)*i+R.shape[1]]=R

            for j in range(0,hz):
                if j<=i:
                    Cdb[np.size(B_aug,0)*i:np.size(B_aug,0)*i+B_aug.shape[0],np.size(B_aug,1)*j:np.size(B_aug,1)*j+B_aug.shape[1]]=np.matmul(np.linalg.matrix_power(A_aug,((i+1)-(j+1))),B_aug)

            Adc[np.size(A_aug,0)*i:np.size(A_aug,0)*i+A_aug.shape[0],0:0+A_aug.shape[1]]=np.linalg.matrix_power(A_aug,i+1)

        Hdb=np.matmul(np.transpose(Cdb),Qdb)
        Hdb=np.matmul(Hdb,Cdb)+Rdb

        temp=np.matmul(np.transpose(Adc),Qdb)
        temp=np.matmul(temp,Cdb)

        temp2=np.matmul(-Tdb,Cdb)
        Fdbt=np.concatenate((temp,temp2),axis=0)

        return Hdb,Fdbt,Cdb,Adc
    
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


# support=SupportFIlesMotor()
# Ad, Bd, Cdt, Dd = support.state_space()
# support.mpc_simplification(Ad, Bd, Cdt, Dd, 3)
# print("Ad: ")
# print(Ad)
# print("\n")
# print("Bd: ")
# print(Bd)
# print("\n")
# print("Cdt: ")
# print(Cdt)
# print("\n")
# print("Dd: ")
# print(Dd)
# print("\n")
