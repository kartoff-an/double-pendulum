# Double Pendulum Animation
A simple simulation and animation of a damped double pendulum, modeled with [Rayleigh dissipation function](https://en.wikipedia.org/wiki/Rayleigh_dissipation_function), using the Runge-Kutta 4th Order (RK4) numerical method implemented in C++ and visualized using SFML 3.0.2. This project demonstrates the chaotic motion of a damped double pendulum system.

<img width="500" alt="double-pendulum" src="https://github.com/user-attachments/assets/fd539837-045f-4a6b-bae6-6c5c3fa528a7" />

Image Source: [The Double Pendulum](https://www.engineered-mind.com/engineering/double-pendulum-1/)

The double pendulum with the Rayleigh dissipation
$R = \frac{1}{2}c_1\dot\theta_1^2 + \frac{1}{2}c_2(\dot\theta_2 - \dot\theta_1)^2$
can be written as

$$M(\theta)\ddot\theta + N(\theta, \dot\theta) + G(\theta) + D\dot\theta = 0$$

where the terms are

- Mass matrix

$$
M(\theta) = 
\begin{pmatrix}
(m_1 + m_2)L_1^2 & m_2 L_1 L_2 \cos\Delta \\ 
m_2L_1L_2\cos\Delta & m_2L_2^2
\end{pmatrix}
$$

- Nonlinear Coriolis / centrifugal vector

$$N(\theta, \dot\theta) = \binom{m_2L_1L_2\sin\Delta\dot\theta_2^2}{-m_2L_1L_2\sin\Delta\dot\theta_1}$$

- Gravity vector

$$G(\theta) = \binom{(m_1 + m_2) gL_1\sin\theta_1}{m_2gL_2\sin\theta_2}$$

- Linear damping matrix

$$
\begin{pmatrix}
c_1 + c_2 & -c_2\\
-c_2 & c_2
\end{pmatrix},
\quad \text{so that} \quad \frac{\partial R}{\partial\dot\theta}=D\dot\theta
$$

# Requirements
- **C++17** or higher.
- **SFML 3.0.2** installed.

## Installation

### 1. Install SFML 3.0.2

Download SFML 3.0.2 from the [official website](https://www.sfml-dev.org/download.php) for your platform and follow the installation instructions.

### 2. Clone this repository
```bash
git clone https://github.com/kartoff-an/double-pendulum
cd double-pendulum
```

### 3. Configure `tasks.json` (in VS Code)

It is already included in the repository for you. Modify if necessary.

### 4. Build and run

## Controls / Parameters
- Adjust initial angles, masses, rod lengths, or the viscous damping coefficients in `main.cpp`.
