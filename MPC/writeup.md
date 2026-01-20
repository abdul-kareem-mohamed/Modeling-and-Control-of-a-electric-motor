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

## 🔄 Augmented State-Space Model

To improve tracking performance and allow the controller to penalize the *rate of change* of the input (rather than just the absolute value), so that we can have a smooth transition to the desired state, we optimize the *rate of change* of the input.  

There is a fundamental mismatch between what the optimizer calculates and what the vehicle needs:  
* Optimizer Output: The MPC finds the sequence of optimal changes: $\Delta V_{i}$.
* Plant Input: The motor physics require the absolute input voltage: $V_{i}$.

  To bridge this, we use the recursive relationship:

  $$V_i = V_{i, k-1} + \Delta V_i$$
  
  This means that to find the current input voltage, the controller must "remember" what the input voltage was at the previous time step ($k-1$).

Because the "previous input voltage" is now required to predict future states, it can no longer be treated as a simple constant; it must become a state variable.  
We augment the state vector $\mathbf{x}$ by adding $\delta$ as the 3rd element:  

$$\mathbf{x}_{aug} = \begin{bmatrix} w \\ I \\ V_{i, k-1} \end{bmatrix}$$  

Thus, the augmented system matrix, and input matrix are as follows:  

$$
\underbrace{\begin{bmatrix}\vec{x}_{k+1} \\ V_{in, k}\end{bmatrix}}_{\tilde{X}_{k+1}} = \underbrace{\begin{bmatrix} A & B \\ \mathbf{0} & I \end{bmatrix}}_{\tilde{A}} \underbrace{\begin{bmatrix}\vec{x}_k \\ V_{in, k-1}\end{bmatrix}}_{\tilde{X}_k} + \underbrace{\begin{bmatrix}B \\ 1\end{bmatrix}}_{\tilde{B}} \Delta V_{in}
$$

and the output matrix is as follows:  

$$
y = \underbrace{\begin{bmatrix} r & 0 \end{bmatrix}}_{\underline{c}^T} \cdot \begin{bmatrix} x_1 \\ x_2 \end{bmatrix}
$$  

* The new system description is:

$$
\underline{\tilde{x}}(k+1) = \underline{\tilde{A}} \ \underline{\tilde{x}}(k) + \underline{\tilde{b}} \Delta u(k)
$$

$$
y(k) = \underline{\tilde{c}}^T \ \underline{\tilde{x}}(k)
$$

## 🔮 MPC Prediction Model

Using the augmented state-space model, the controller predicts the future output values $\hat{y}$ over a prediction horizon $N_p$. This prediction is performed recursively, assuming the control horizon $N_u$ equals the prediction horizon ($N_p = N_u$).

**1. Recursive Prediction Vectors**
By iterating the state equation forward, we express the vector of predicted outputs $\underline{\hat{y}}$ as a function of the current state $\underline{\tilde{x}}(k)$ and the sequence of future control increments $\Delta \underline{u}$:

$$
\underline{\hat{y}} = 
\begin{bmatrix} 
\hat{y}(k+1) \\ 
\hat{y}(k+2) \\ 
\vdots \\ 
\hat{y}(k+N_p) 
\end{bmatrix}
$$

**2. Matrix Formulation**
The recursive equations are stacked into a compact matrix form. This separates the prediction into two parts: the **Free Response** (effect of current state) and the **Forced Response** (effect of future control inputs):

$$
\underline{\hat{y}} = \underline{F} \underline{\tilde{x}}(k) + \underline{H} \Delta \underline{u}^+
$$

Where the matrices are defined as:

* **Prediction Matrix ($F$):** Propagates the current state forward.

$$
\underline{F} = 
\begin{bmatrix} 
\tilde{c}^T \tilde{A} \\ 
\tilde{c}^T \tilde{A}^2 \\ 
\vdots \\ 
\tilde{c}^T \tilde{A}^{N_p} 
\end{bmatrix}
$$
  
* **Control Matrix ($H$):** A Toeplitz-like matrix representing the system's impulse response to future inputs.
  
$$
\underline{H} = 
\begin{bmatrix} 
\tilde{c}^T \tilde{b} & 0 & \cdots & 0 \\ 
\tilde{c}^T \tilde{A} \tilde{b} & \tilde{c}^T \tilde{b} & \cdots & 0 \\ 
\vdots & \vdots & \ddots & \vdots \\ 
\tilde{c}^T \tilde{A}^{N_p-1} \tilde{b} & \tilde{c}^T \tilde{A}^{N_p-2} \tilde{b} & \cdots & \tilde{c}^T \tilde{b} 
\end{bmatrix}
$$
  
**Significance:**
This formulation allows the MPC optimization problem to be solved as a standard Quadratic Programming (QP) problem, minimizing the error between $\underline{\hat{y}}$ and the reference trajectory.  

## 🎯 Optimization & Control Law

To determine the optimal control inputs, we define a quadratic cost function $J$. This function penalizes the deviation of the predicted output $\hat{y}$ from the reference trajectory $w$, as well as the magnitude of the control updates $\Delta u$ (to ensure smooth operation).

**1. Cost Function**
The optimization problem is formulated as a sum of squared errors over the prediction horizon:

$$
J = \min_{\Delta u} \left( \sum_{i=1}^{N_p} (w_{k+i} - \hat{y}_{k+i})^2 + \lambda \sum_{i=1}^{N_u} \Delta u_{k+i}^2 \right)
$$

Where:
* $w$: Reference trajectory (target speed).
* $\hat{y}$: Predicted output from our state-space model.
* $\lambda$: Tuning parameter (weight) for control effort.

**2. Analytical Solution (Unconstrained)**
By substituting the prediction model equation ($\underline{\hat{y}} = \underline{F} \underline{\tilde{x}}(k) + \underline{H} \Delta \underline{u}$) into the cost function, the problem reduces to a standard linear least-squares problem.

Assuming no physical constraints (like voltage saturation), the optimal control sequence $\Delta \underline{u}^+$ can be calculated directly using a closed-form solution:

$$
\Delta \underline{u}^+ = \left( \underline{H}^T \underline{H} + \lambda \underline{I} \right)^{-1} \underline{H}^T (\underline{w} - \underline{F} \underline{\tilde{x}}(k))
$$

**Why is this important?**
* **Efficiency:** Because the solution is analytical (just matrix multiplication), it is computationally very fast and suitable for real-time implementation on embedded microcontrollers.
* **Role of $\lambda$:** The term $\lambda \underline{I}$ ensures the matrix is invertible and allows us to tune how "aggressive" the controller is. A higher $\lambda$ results in smoother, slower control.

The $$\Delta \underline{u}^+$$ can then be added with the previous input to calculate the new input. The new input can then be used in the plant to control the speed parameter. This loop continues for the complete time duration. 
