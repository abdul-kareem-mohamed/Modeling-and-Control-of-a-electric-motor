The objective of this project is to design and implement a controller for regulating the speed of an electric motor. Model Predictive Controller is used to control the speed of the motor. 

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

### ⚡ Discrete-Time Model (Zero-Order Hold)

To implement the controller on a digital processor, the continuous-time system is discretized using the **Zero-Order Hold (ZOH)** method. This assumes the control input $u(t)$ remains constant between sampling instances $T_s$.

The system takes the discrete form:

$$x_{k+1} = A_d x_k + B_d u_k$$
$$y_k = C_d x_k$$

Where the discrete matrices are derived from the continuous system matrices ($A, B$) and sampling time ($T_s$) as follows:

**1. Discrete System Matrix ($A_d$):**
Calculated using the matrix exponential:
$$A_d = e^{A T_s} = \mathcal{L}^{-1} \{ (sI - A)^{-1} \} |_{t=T_s}$$

**2. Discrete Input Matrix ($B_d$):**
obtained by integrating the state transition over one sample period:
$$B_d = \left( \int_{0}^{T_s} e^{A\tau} d\tau \right) B$$
*(Note: If $A$ is invertible, this simplifies to $$B_d = A^{-1}(A_d - I)B$$)*

**3. Discrete Output Matrix ($C_d$):**
Since the output equation is algebraic, it remains unchanged:
$$C_d = C$$

## 🔄 Augmented State-Space Model (Velocity Form)

To improve tracking performance and allow the controller to penalize the *rate of change* of the input (rather than just the absolute value), we reformulate the system into the **Velocity Form**.

In this formulation, the manipulated variable becomes the input increment:
$$\Delta u_k = u_k - u_{k-1}$$

This requires augmenting the state vector to include the state increments ($\Delta x_k$) and the current output ($y_k$).

**1. Difference Equation**
Subtracting the state equation at time $k-1$ from time $k$:
$$x_{k+1} - x_k = A_d(x_k - x_{k-1}) + B_d(u_k - u_{k-1})$$
$$\Delta x_{k+1} = A_d \Delta x_k + B_d \Delta u_k$$

**2. Augmented State Vector**
We define a new state vector $\mathbf{x}_{aug}$ that stacks the change in state with the output:
$$
\mathbf{x}_{aug, k} = \begin{bmatrix} \Delta \mathbf{x}_k \\ y_k \end{bmatrix} = \begin{bmatrix} \Delta \omega_k \\ \Delta I_k \\ y_k \end{bmatrix}
$$

**3. Augmented Matrices**
The output equation evolves as $y_{k+1} = y_k + C_d \Delta x_{k+1}$. Combining this with the difference equation yields the augmented system matrices:

$$
\mathbf{x}_{aug, k+1} = 
\underbrace{
\begin{bmatrix} 
A_d & \mathbf{0}^T \\ 
C_d A_d & 1 
\end{bmatrix}}_{A_{aug}} 
\cdot \mathbf{x}_{aug, k} 
+ 
\underbrace{
\begin{bmatrix} 
B_d \\ 
C_d B_d 
\end{bmatrix}}_{B_{aug}} 
\cdot \Delta u_k
$$

$$
y_k = \underbrace{\begin{bmatrix} \mathbf{0} & 1 \end{bmatrix}}_{C_{aug}} \cdot \mathbf{x}_{aug, k}
$$

**Why use this formulation?**
* **Integral Action:** By including $y_k$ in the state, the controller "remembers" the current output level, ensuring offset-free tracking of the reference.
* **Slew Rate Control:** Since the input is $\Delta u_k$, we can directly penalize aggressive changes in voltage, protecting the motor from voltage spikes.

---
