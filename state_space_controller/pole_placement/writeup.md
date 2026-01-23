The objective of this project is to design and implement a controller for regulating the speed of an electric motor. State-space controller is designed via pole placement to control the speed of the motor. 

## ⚙️ Mathematical Model  

This project models the dynamics of a DC motor driving a vehicle. The system is divided into electrical and mechanical subsystems, derived using Kirchhoff's Voltage Law and Newton's Second Law.

<img width="438" height="232" alt="image" src="https://github.com/user-attachments/assets/ac646610-8598-493b-acf3-c109bd570d51" />  

Figure 1: Electric Drive: Schematic diagram of DC motor  


<img width="481" height="200" alt="image" src="https://github.com/user-attachments/assets/18b4e9ec-e411-40ca-876c-490a7099ccb0" />  

Figure 2: Simplified longitudinal-dynamics model of a vehicle  

### Differential Equations

**1. Electrical Subsystem**
Applying Kirchhoff's voltage law to the armature circuit, where $V_{in}$ is the input voltage, $R$ is resistance, and $L$ is inductance:

$$V_{in} = V_R + V_L + V_{EMF}$$

$$V_{in} = R \cdot I + L \cdot \dot{I} + k_m \cdot \omega$$

Rearranging for the rate of change of current ($\dot{I}$):
$$\dot{I} = \frac{1}{L}V_{in} - \frac{R}{L}I - \frac{k_m}{L}\omega$$

**2. Mechanical Subsystem**
The motor torque ($T_M$) is proportional to the current ($I$) by the motor constant ($k_m$), driving the inertia ($J$):

$$J \cdot \dot{\omega} = T_M = k_m \cdot I$$

Rearranging for angular acceleration ($\dot{\omega}$):
$$\dot{\omega} = \frac{k_m}{J} \cdot I$$

**3. Output Relationship**
The linear velocity of the car ($v$) is related to the angular velocity ($\omega$) by the wheel radius ($r$):
$$v = r \cdot \omega$$

---

### State-Space Representation

To design the controller, we formulate the system in state-space notation $\dot{x} = Ax + Bu$ and $y = Cx$.

**State Vectors:**
* State $\mathbf{x} = [x_1, x_2]^T = [\omega, I]^T$
* Input $u = V_{in}$
* Output $y = v$ (Vehicle Velocity)

**Matrix Formulation:**

$$
\begin{bmatrix} \dot{x}_1 \\ \dot{x}_2 \end{bmatrix} =
\underbrace{
\begin{bmatrix}
0 & \frac{k_m}{J} \\
-\frac{k_m}{L} & -\frac{R}{L}
\end{bmatrix}}_{A}
\cdot
\begin{bmatrix} x_1 \\ x_2 \end{bmatrix}
+
\underbrace{
\begin{bmatrix}
0 \\
\frac{1}{L}
\end{bmatrix}}_{B}
\cdot u
$$

**Output Equation:**

$$
y = \underbrace{\begin{bmatrix} r & 0 \end{bmatrix}}_{C^T} \cdot \begin{bmatrix} x_1 \\ x_2 \end{bmatrix}
$$

### Nomenclature

| Symbol | Parameter | Unit |
| :--- | :--- | :--- |
| $V_{in}$ | Input Voltage | $V$ |
| $I$ | Armature Current | $A$ |
| $\omega$ | Angular Velocity | $rad/s$ |
| $R$ | Armature Resistance | $\Omega$ |
| $L$ | Armature Inductance | $H$ |
| $k_m$ | Motor Constant | $N\cdot m/A$ |
| $J$ | Moment of Inertia | $kg \cdot m^2$ |
| $r$ | Wheel Radius | $m$ |

## State-Space Control via Pole Placement  

State-space control represents a fundamental shift from classical control methods (like PID). While classical control typically relies on feeding back the system output $y(t)$ (error-based control), state-space control involves feeding back the entire state vector $x(t)$. This approach provides significantly more authority over the system's dynamics, allowing for precise placement of the closed-loop poles to achieve desired stability and performance characteristics.  

**The Control Law**  

The core mechanism of this controller is the state feedback law. The controller computes the manipulated variable $u(t)$ (the input to the plant) by summing the weighted individual states and comparing them to the reference value.  

The control equation is defined as:  

$$u(t) = w(t) - \underline{k}^T \underline{x}(t)$$  

Where:  

* $u(t)$: The control input (manipulated variable).
* $w(t)$: The reference value (setpoint). unlike simple regulators where $w(t)=0$, this formulation maintains $w(t)$ for direct tracking control.
* $\underline{k}^T$: The feedback gain vector (row vector). It contains the weights $k_1, k_2, \dots, k_n$.
* $\underline{x}(t)$: The state vector containing all system states $x_1, \dots, x_n$.

Expanded, the feedback term looks like this:  $$\underline{k}^T \underline{x}(t) = k_1 x_1(t) + k_2 x_2(t) + \dots + k_n x_n(t)$$

<img width="857" height="380" alt="image" src="https://github.com/user-attachments/assets/2b9e0663-3628-485f-be3e-509c1c9afe14" />  
Figure 3: closed-loop of the system  



