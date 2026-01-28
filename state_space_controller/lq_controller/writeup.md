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

## 📉 Control Theory: The Loss Function

Before implementing the discrete MPC, we define the control objective using a continuous-time quadratic loss function $J$. This function mathematically defines what "optimal" performance means for the motor.

** The General Cost Function**
The controller aims to minimize the accumulated error and control effort over a time interval $t_e$:

$$
J = \int_{0}^{t_e} [e^2(t) + \alpha u^2(t)] \, dt
$$

Where:
* **$e(t)$**: Control deviation (Error = Target Speed - Actual Speed).
* **$u(t)$**: Actuating effort (Voltage applied to the motor).
* **$\alpha$**: Weighting factor (Penalizes high voltage usage).
