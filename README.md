# Modeling-and-Control-of-a-electric-motor
This repository houses the speed control of a electric motor using state-space control and MPC control.  

## Model Predictive Control  

<img width="879" height="432" alt="image" src="https://github.com/user-attachments/assets/bd8b735f-f4a9-4fd9-b3d9-ad8093137612" />  
Figure 1: Velocity vs Time Plot  

<img width="889" height="425" alt="image" src="https://github.com/user-attachments/assets/5cf7ab60-ffb0-468e-b777-a0f2f8823cff" />  
Figure 2: Zoomed image of Velocity vs Time plot  

<img width="867" height="461" alt="image" src="https://github.com/user-attachments/assets/81f72f16-726d-4158-9d92-36b27d038268" />  
Figure 3: Input Voltage vs Time plot  

when the actuation penalty got increased to 0.1, the results are as follows:  

<img width="871" height="422" alt="image" src="https://github.com/user-attachments/assets/0a924db9-7646-4ba5-8668-64441537403e" />  
Figure 4: Velocity vs Time Plot after increasing the actuation penalty to 0.1  

<img width="777" height="570" alt="image" src="https://github.com/user-attachments/assets/5abf79b1-c6a9-435d-92f6-7f7509530dad" />  
Figure 5: Input Velocity vs Time plot after increasing the actuation penalty to 0.1  

When the control action parameter got penalised, the aggressive behaviour of input voltage has drastically reduced but this has introduced some oscillation in the speed control.  






