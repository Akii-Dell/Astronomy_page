### Answers to Questions Based on "Robotics All Slides" PDF

### Question 2

#### a) Define the D-H parameters for the \( j \)-th and \( (j-1) \)-th links in a general serial link manipulator. Give a labeled diagram to support your answer.

- **D-H Parameters:**
  - \( a_{j-1} \): Link length, the distance between \( Z_{j-1} \) and \( Z_j \) along \( X_{j-1} \).
  - \( \alpha_{j-1} \): Link twist, the angle between \( Z_{j-1} \) and \( Z_j \) along \( X_{j-1} \).
  - \( d_j \): Link offset, the distance between \( X_{j-1} \) and \( X_j \) along \( Z_j \).
  - \( \theta_j \): Joint angle, the angle between \( X_{j-1} \) and \( X_j \) along \( Z_j \).

#### b) Derive the forward kinematics of the manipulator in Figure Q2.

- **Forward Kinematics:**
  - For a 2-link planar manipulator, the transformation matrix from the base to the end-effector is:
\[ T = T_1 \cdot T_2 \]
  - Where:
\[ T_1 = \begin{bmatrix}
\cos\theta_1 & -\sin\theta_1 & 0 & a_1\cos\theta_1 \\
\sin\theta_1 & \cos\theta_1 & 0 & a_1\sin\theta_1 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1
\end{bmatrix} \]
\[ T_2 = \begin{bmatrix}
\cos\theta_2 & -\sin\theta_2 & 0 & a_2\cos\theta_2 \\
\sin\theta_2 & \cos\theta_2 & 0 & a_2\sin\theta_2 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1
\end{bmatrix} \]

#### c) Frame {B} turns \( \frac{\pi}{3} \) radians towards the positive rotational direction according to the right-hand rule. Give the rotational matrix {B} with respect to {O}.

- **Rotation Matrix:**
\[ R = \begin{bmatrix}
\frac{1}{2} & -\frac{\sqrt{3}}{2} & 0 \\
\frac{\sqrt{3}}{2} & \frac{1}{2} & 0 \\
0 & 0 & 1
\end{bmatrix} \]

#### d) What should be the logarithm of the rotation matrix in (c)?

- **Logarithm of the Rotation Matrix:**
\[ \log(R) = \frac{\pi}{3} \cdot \begin{bmatrix}
0 & -1 & 0 \\
1 & 0 & 0 \\
0 & 0 & 0
\end{bmatrix} = \begin{bmatrix}
0 & -\frac{\pi}{3} & 0 \\
\frac{\pi}{3} & 0 & 0 \\
0 & 0 & 0
\end{bmatrix} \]

#### e) Draw a labeled diagram for applying the bicycle model for the Xavier robot. Clearly define the symbols that you use to label the diagram.

- **Bicycle Model Diagram:**
  - Symbols:
    - \( L \): Wheelbase
    - \( \delta \): Steering angle
    - \( \alpha \): Heading angle
    - \( v \): Velocity

#### f) Derive the forward kinematics for the Xavier bot based on your bicycle model in (e).

- **Forward Kinematics Equations:**
\[ \dot{x} = v \cos(\alpha) \]
\[ \dot{y} = v \sin(\alpha) \]
\[ \dot{\alpha} = \frac{v}{L} \tan(\delta) \]

#### g) Derive the forward dynamics of the robot. Consider translational and rotational dynamics, and account for thrusts and torques in your derivations. Use diagrams to illustrate the dynamic model.

- **Forward Dynamics:**
  - Translational Dynamics:
\[ m \ddot{\vec{r}} = \sum \vec{F} \]
  - Rotational Dynamics:
\[ I \ddot{\vec{\omega}} = \sum \vec{\tau} \]

#### h) How do you control the yaw both practically and mathematically?

- **Practically:** By varying the speeds of the diagonal pairs of rotors.
- **Mathematically:** Using the yaw torque equation:
\[ \tau_{yaw} = k (\omega_1^2 + \omega_3^2 - \omega_2^2 - \omega_4^2) \]

#### i) How to accelerate the air frame towards the world x-direction?

- **Acceleration in x-direction:** By tilting the quadrotor to generate a horizontal thrust component in the x-direction.

#### j) How do you use feed-forward control for controlling the altitude?

- **Feed-Forward Control:** Use a feed-forward term to anticipate changes in altitude based on the desired altitude trajectory and current state.

### Question 3

#### k) Find the Jacobian between the spatial velocity vector and joint velocities.

- **Jacobian Matrix:**
\[ \nu = J \dot{\theta} \]
  - For a two-link planar manipulator, the Jacobian \( J \) is:
\[ J = \begin{bmatrix}
-a_1 \sin(\theta_1) - a_2 \sin(\theta_1 + \theta_2) & -a_2 \sin(\theta_1 + \theta_2) \\
a_1 \cos(\theta_1) + a_2 \cos(\theta_1 + \theta_2) & a_2 \cos(\theta_1 + \theta_2) \\
1 & 1
\end{bmatrix} \]

#### l) Derive the inverse dynamics of the manipulator using Lagrangian Formulation.

- **Lagrangian \( \mathcal{L} \):**
\[ \mathcal{L} = K - U \]
  - Apply the Euler-Lagrange equation:
\[ \tau = \frac{d}{dt} \left( \frac{\partial \mathcal{L}}{\partial \dot{q}} \right) - \frac{\partial \mathcal{L}}{\partial q} \]

#### m) Explain the following visual servo configurations with proper illustrations for the manipulator in Figure Q2.

i. **End point closed loop:**
- **Description:** The camera is mounted at the end effector, providing feedback for closed-loop control.
- **Diagram:** Illustrate the manipulator with a camera at the end effector, connected to a feedback control system.

ii. **End point open loop:**
- **Description:** The camera is mounted in the workspace, observing the manipulator from a fixed position.
- **Diagram:** Illustrate the workspace with a fixed camera monitoring the manipulator.

#### n) Assume that a camera is mounted at the end effector of the manipulator in Figure Q2. Explain the difference between the following visual servo techniques with suitable process diagrams.

i. **Position based visual servo:**
- **Description:** Uses the position of features in the image to control the manipulator.
- **Diagram:** Show the end effector with the camera identifying a target position and adjusting its movement based on position error.

ii. **Image based visual servo:**
- **Description:** Uses the image features directly to control the manipulator.
- **Diagram:** Illustrate the end effector with the camera tracking specific features and adjusting its movement to maintain those features at desired positions in the image.
