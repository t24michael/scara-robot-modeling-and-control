# scara-robot-modeling-and-control

TBD

## TODO

- ~~Add dynamic model~~  
- ~~Add inverse dynamics control~~  
- ~~Add holist inverse dynamics control~~  
- ~~Add classical control~~  
- Add cartesian control  
- ~~Add library~~  

---

# SCARA Dynamic Modelling and Control

In this project we will go over the mathematical modelling and control strategies of a SCARA robot arm.

![SCARA](images/SCARA.png)

The SCARA robot arm usually presents four joints which consist of:

- Three revolute joints  
- One prismatic joint  

For now we simplify things and imagine the robot as just the first two revolute joints.

---

## Dynamic Modelling

Dynamic modelling is an important step in the analysis and modelling of a robotic arm as it establishes the mathematical relationship between the forces or moments applied to the motors of the robotic arm and the joint movements.

The dynamic model incorporates physical properties of the robot such as:

- Mass  
- Moments of inertia  
- Viscous forces  
- Fictitious forces such as Coriolis and centrifugal  
- Gravity  

The dynamic model of the SCARA is represented through a complex matricial equation:

$$
M(q)\ddot{q} + N(q, \dot{q}) + G(q) + J^T(q)T_R + F(\dot{q}) + \tau_d = \tau
$$

Where:

- $M(q)$ — inertia matrix  
- $N(q,\dot{q})$ — Coriolis & centrifugal matrices  
- $G(q)$ — gravity vector  
- $F(q)$ — viscous forces vector  
- $\tau_d$ — disturbance torque vector  

For easier computation of each vector and matrix we will use the Euler–Lagrange approach.

---

## Inertia Matrix

The inertia matrix for the SCARA robot is decoupled, meaning that the links don't depend on each other.

$$
M(q) = \sum_{i=1}^{n} \left( m_i J_{vi}^T J_{vi} + J_{\omega i}^T I_{Ci} J_{\omega i} \right)
$$

Where:

- $m_i$ — mass of link  
- $I_{Ci}$ — inertia tensor at COM reference frame  
- $J_{vi}, J_{\omega i}$ — Jacobian components at COM  

---

## Coriolis & Centrifugal Matrices

These are also called fictitious forces because they appear only inside a rotating reference frame.

Example intuition:

- **Coriolis:** Like throwing a ball on a spinning playground wheel  
- **Centrifugal:** Like feeling pushed outward on a curved road  

For our SCARA example:

### Quadratic Velocity Terms

$$
C(q)[\dot{q}^2] =
\begin{bmatrix}
b_{111} & b_{122} & \cdots & b_{1nn} \\
b_{211} & b_{222} & \cdots & b_{2nn} \\
\vdots & \vdots & \ddots & \vdots \\
b_{n11} & b_{n22} & \cdots & b_{nnn}
\end{bmatrix}
\begin{bmatrix}
\dot{q}_1^2 \\
\dot{q}_2^2 \\
\vdots \\
\dot{q}_n^2
\end{bmatrix}
$$

### Cross Velocity Terms

$$
B(q)[\dot{q}\dot{q}] =
\begin{bmatrix}
2b_{112} & 2b_{123} & \cdots & 2b_{1n-1n} \\
2b_{212} & 2b_{223} & \cdots & 2b_{2n-1n} \\
\vdots & \vdots & \ddots & \vdots \\
2b_{n12} & 2b_{n23} & \cdots & 2b_{nn-1n}
\end{bmatrix}
\begin{bmatrix}
\dot{q}_1 \dot{q}_2 \\
\dot{q}_1 \dot{q}_3 \\
\vdots \\
\dot{q}_{n-1} \dot{q}_n
\end{bmatrix}
$$

Christoffel coefficients:

$$
b_{ijk} = \frac{1}{2}(m_{ijk} + m_{ikj} + m_{jki})
$$

$$
m_{ijk} = \frac{\partial m_{ij}}{\partial q_k}
$$

---

## Control

Control modifies system input using feedback to minimize error between desired and measured values.

---

### PID Controller Components

- **Proportional**
$$
u(t) = K_p e(t)
$$

- **Integral**
$$
u(t) = K_i \int e(t) dt
$$

- **Derivative**
$$
u(t) = K_d \frac{de(t)}{dt}
$$

---

## Control Strategies

- Classical control  
- Inverse dynamics holistic control  
- Cartesian control  
- Adaptive control  

---

## Classical Control

Each joint motor is controlled by a PID which determines the required voltage.  
Joint motion generates disturbance torques which feed back into the motor.

---

## DC Motor Dynamic Model

### Electrical Component

$$
V = RI + L\frac{dI}{dt} + V_{emf}
$$

Where:

- $V$ — voltage  
- $R$ — armature resistance  
- $I$ — current  
- $L$ — inductance  
- $V_{emf} = k_{emf}\omega$  

---

### Mechanical Component

$$
\begin{cases}
J_a \ddot{q}_a = -M + M_{motor} - b\dot{q} \\
\dot{q} = \frac{\dot{q}_a}{i_r} \\
\tau = i_r M \eta
\end{cases}
$$

Where:

- $J_a$ — rotor inertia  
- $M$ — resistive torque  
- $M_{motor} = k_M I$  
- $b$ — viscous coefficient  
- $\eta$ — efficiency  

---

### Combined Motor Equation

After simplification:

$$
I = \frac{V - k_M \dot{q}_m}{R}
$$

Substitute into mechanical model → final form:

$$
k_M\frac{V}{Ri_r}
=
J_a\ddot{q}
+
\frac{\tau}{i_r^2\eta}
+
\dot{q}\left(b + \dot{q}\frac{k_M^2}{R}\right)
$$

---

### Generalized For n Joints

$$
\mathrm{diag}\left(k_M\frac{1}{Ri_r}\right)V
=
\mathrm{diag}(J_a)\ddot{q}
+
\mathrm{diag}\left(\frac{1}{i_r^2\eta}\right)\tau
+
\mathrm{diag}\left(b + \dot{q}\frac{k_M^2}{R}\right)\dot{q}
$$

---

## Simulink Implementation

![Classical Control](images/classical_control.png)

---

## System Blocks Description

### Trajectory Block

Contains:

- $q_{desired}$  
- $\dot{q}_{desired}$  
- $\ddot{q}_{desired}$  

---

### Motor Blocks

Inputs:

- $\tau$ — resistive torque  
- $u$ — control signal  
- velocity — motor speed  

![Motor](images/motor.png)

---

## Motor + Gearbox Specifications

### Combination

| Parameter | Value |
|---|---|
| Maximum Voltage | 24 V |
| No-load Speed | 82.70 rpm |
| Maximum Speed | 68 rpm |
| Maximum Torque | 7.5 Nm |
| Maximum Current | 4.26 A |
| Efficiency | 57.59% |

---

### Motor

| Parameter | Value |
|---|---|
| Torque Coefficient $k_M$ | 27.3 mNm/A |
| Speed Constant $k_e$ | 350 rpm/V |
| Rotor Inertia $J_c$ | 72.8 g·cm² |
| Resistance $R$ | 0.331 Ω |
| Inductance $L$ | 0.103 mH |
| Motor Mass | 326.9 g |
| Length | 71.9 mm |
| Efficiency | 88.6% |

---

### Gearbox

| Parameter | Value |
|---|---|
| Ratio $i_r$ | 103:1 |
| Efficiency | 65% |

---

## SCARA Inverse Dynamics Block

Represents robot structure torque load.

Inputs:

- $q_{1,2}$ — commanded position  
- $\dot{q}_{1,2}$ — commanded speed  
- $\ddot{q}_{1,2}$ — commanded acceleration  

![Inverse Dynamics](images/SCARA_inverse_dynamics.png)
