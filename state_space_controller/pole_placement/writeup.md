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

The state-space controller therefore acts proportionally with the factors kion the states. Is it therefore also a P-controller? It is a $PD_{n-1}$ controller. However, with a decisive difference to the standard controller: The derivatives are not obtained by differentiation of the output signal. Instead, the states are measured which already correspond to these differentiated quantities! So, there is no amplification of noise here!  

For the design of a state-space controller, i.e., the systematic determination of the values for $k_{i}$, there are two main methods:  

* *Pole placement*: There are n control parameters available (k1, k2, ..., kn), with which it is possible to shift the n poles of the characteristic polynomial to desired positions.
* *Optimization*: Minimizing a quadratic loss function leads to a globally optimal, simple, easily computable solution for the state-space controller (in contrast to the standard controller).

Our goal is now to derive an equation for determining the controller parameters. To do this, we need to find the relationship between the controller parameters and the characteristic equation. The equations of state with controller are as follows:  

**The Closed-Loop System Equation**  

We start with the standard state-space equations for the open-loop system:  

$$\underline{\dot{x}} = \underline{A}\underline{x} + \underline{b}u$$$$y = \underline{c}^T\underline{x}$$  

Substituting the control law $u = w - \underline{k}^T\underline{x}$ into the state equation yields the closed-loop dynamics:  

$$\underline{\dot{x}} = \underline{A}\underline{x} + \underline{b}(w - \underline{k}^T\underline{x})$$  

Rearranging terms to group the state vector $\underline{x}$:  

$$\underline{\dot{x}} = (\underline{A} - \underline{b}\underline{k}^T)\underline{x} + \underline{b}w$$  

This new equation represents the closed-loop system where:  

* $(\underline{A} - \underline{b}\underline{k}^T)$: This is the new effective system matrix. The state feedback has fundamentally altered the system dynamics.  

* $w$: The reference variable now acts as the external input replacing the direct manipulated variable $u$.


### Calculating Controller Parameters via Characteristic Equations  

**1. Defining the Desired Behavior**  

The fundamental goal of pole placement is to force the closed-loop system to behave according to a set of specified dynamics. This is achieved by selecting a set of $n$ desired poles, $\lambda_1, \lambda_2, \dots, \lambda_n$, where $n$ is the order of the system.  

Mathematically, specifying these poles is equivalent to defining a desired characteristic polynomial, $N(s)$:  

$$N(s) = (s - \lambda_1)(s - \lambda_2)\cdot \dots \cdot (s - \lambda_n)$$  

When expanded, this yields a polynomial of the $n$-th degree with specific target coefficients $n_i$:  

$$N(s) = s^n + n_{n-1}s^{n-1} + \dots + n_1s + n_0$$  

**2. The Design Condition**  

To implement this, we must ensure that the actual characteristic equation of our closed-loop system matches this desired polynomial.  

Recall that the closed-loop system matrix is $(\underline{A} - \underline{b}\underline{k}^T)$. The characteristic equation is found by taking the determinant of $(s\underline{I} - \text{System Matrix})$. Therefore, the fundamental condition for pole placement is:  

$$\det \left( s\underline{I} - (\underline{A} - \underline{b}\underline{k}^T) \right) = s^n + n_{n-1}s^{n-1} + \dots + n_1s + n_0$$  

**3. Coefficient Comparison Method**  

This equation provides the direct algebraic link between the unknown controller gains ($k_i$) and the desired performance ($n_i$).  

* Calculate Determinant: Analytically compute the determinant on the left-hand side. This will result in a polynomial where the coefficients are functions of the unknown gains $k$.
* Compare Coefficients: Match the coefficients of the powers of $s$ (e.g., $s^0, s^1, \dots$) from the calculated determinant with the coefficients $n_i$ from the desired polynomial.
* Solve: This yields a system of linear equations that can be solved to find the specific values for the vector $\underline{k}^T$.

**4. The Stability & Existence Condition (Controllability)**  

A critical prerequisite exists for this method to work. The state-space controller can arbitrarily move all poles to any desired location if and only if the system $(\underline{A}, \underline{b})$ is completely controllable.  
* Logic: If a system is non-controllable, the input $u$ has no physical influence on certain internal states (uncontrollable modes). Consequently, no feedback gain $k$ can alter the dynamics (eigenvalues) associated with those specific states.
* Verification: Before attempting pole placement, always verify that the Controllability Matrix has full rank: $\text{rank}(\mathcal{C}) = n$.

### Implementation  

The state-space equations are transformed to controllable canonical form and then the $k_{c_i}$ parameters are calculated. Then, the calculated $k_{c_i}$ are back transformed to $k_{i}$ for the original system. The control parameters are then used to design the control loop.   

A static prefilter is used to account the steady-state error.  

