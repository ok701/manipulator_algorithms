# Manipulator Control Algorithms
MATLAB implementations demonstrating core concepts of manipulator control, from kinematics to task-space CoM regulation.

## 1. `forward_kinamatics.m`
This script calculates the **forward kinematics** and **gravity torques** for compensation of a **6-DOF spatial manipulator**.

### Forward Kinematics
$$
T_0^6(q) = \prod_{i=1}^{6} T_{i-1}^{i}(a_i, \alpha_i, d_i, \theta_i)
$$

Computed using the **standard Denavit–Hartenberg (DH)** convention.  
The end-effector pose is obtained from $T_0^6$, and Z–Y–X Euler angles are derived from its rotation matrix.

### Gravity Torques
$$
P(q) = \sum_i m_i \, g^T p_{c_i}(q), \qquad G(q) = \frac{\partial P}{\partial q}
$$

- $P(q)$: total potential energy  
- $G(q)$: gravity torque vector (used for static gravity compensation)  
- Each CoM height approximated as the midpoint along its local $z$-axis

### Jacobians
- **Geometric Jacobian:** $J_g = [J_v; J_w]$  
  based on joint axes $z_i$ and positions $o_i$
- **Analytic Jacobian:** $J_a = \mathrm{diag}(I, B^{-1}) J_g$,  
  where $B$ is the Z–Y–X Euler transformation matrix.

## 2. `inverse_dynamics.m`
This script simulates position control of a **3-DOF planar manipulator** using inverse kinematics with open-loop inverse dynamics.  
The three joint torques are computed from the end-effector position while keeping the last link’s global orientation $\theta_d$ fixed.

### Pipeline 
$$
(x, y, \theta_d)\xrightarrow{\text{IK}} q(t)\xrightarrow{\text{diff}} \dot q(t),\ \ddot q(t)\xrightarrow{\text{ID}} \tau(t)
$$


$$
s(t) = \frac{1 - \cos(\pi t / T)}{2}, \quad q(t) = q_0 + \Delta q \, s(t)
$$

- Smooth cosine-based interpolation ensuring continuous velocity and acceleration.

### Inverse Kinematics
$$
\begin{aligned}
D &= \frac{(x - a_3\cos\theta_d)^2 + (y - a_3\sin\theta_d)^2 - a_1^2 - a_2^2}{2a_1a_2}, \quad D \in [-1,1] \\
\theta_2 &= \text{atan2}(-\sqrt{1 - D^2}, D) \\
\theta_1 &= \text{atan2}(y - a_3\sin\theta_d,\; x - a_3\cos\theta_d)
            - \text{atan2}(a_2\sin\theta_2,\; a_1 + a_2\cos\theta_2) \\
\theta_3 &= \theta_d - (\theta_1 + \theta_2)
\end{aligned}
$$

### Inverse Dynamics
$$
\tau = M(q)\ddot{q} + C(q, \dot{q}) + G(q)
$$

## 3. `com_control.m`
This is an extended version of **2-DOF planar manipulator** control,  
implementing task-space PD control based on center of mass (CoM) regulation.

### Center of Mass (CoM)
$$
\begin{aligned}
p_{c1}(q) &= 
\begin{bmatrix}
d_1\cos q_1\\
d_1\sin q_1
\end{bmatrix}, \qquad
p_{c2}(q) =
\begin{bmatrix}
L_1\cos q_1 + d_2\cos(q_1+q_2)\\
L_1\sin q_1 + d_2\sin(q_1+q_2) 
\end{bmatrix},\\
p_{\mathrm{CoM}}(q) &= 
\frac{m_1\,p_{c1}(q) + m_2\,p_{c2}(q)}{M}, \qquad M = m_1 + m_2.
\end{aligned}
$$

### Geometric CoM Jacobian
The **first joint** affects both links’ centers of mass:

$$
\frac{\partial p_{\mathrm{CoM}}}{\partial q_1}
= \frac{1}{M}\sum_i m_i\, z_0 \times (p_{c,i} - p_0)
= z_0 \times (p_{\mathrm{CoM}} - p_0)
$$

The **second joint** only moves the second link’s CoM:

$$
\frac{\partial p_{\mathrm{CoM}}}{\partial q_2}
= \frac{m_2}{M}\, z_1 \times (p_{c,2} - p_1)
$$

### Task-Space PD Control
$$
\begin{aligned}
e &= p_{\mathrm{ref}} - p_{\mathrm{act}}, \qquad
F = K_p e - K_d v_{\mathrm{act}},\\
\tau &= J_{\mathrm{CoM}}(q)^{\top} F + G(q)
\end{aligned}
$$

- For stability, the damping term $-K_d v_{\mathrm{act}}$ is used when $v_{\mathrm{ref}} = 0$.
