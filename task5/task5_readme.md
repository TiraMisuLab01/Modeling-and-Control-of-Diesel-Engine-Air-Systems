# Task 5: Servo Control Design with State Estimation for Disturbance Rejection

## 1. Introduction

Task 5 requires the design of a multivariable controller to maintain the output of the diesel engine air system near a specific operating setpoint $y_{sp} = [0.4, 0.8]^T$ and ensure zero steady-state error in the presence of external step disturbances. Unlike previous tasks where the target state was the zero, this task requires the system to track a non-zero reference signal. Furthermore, design constraints require that only two system outputs are measurable, precluding the direct use of full-state feedback. Therefore, the control strategy must combine a servo mechanism for tracking and disturbance suppression with a state observer for estimating the unobservable internal state.

> 任务5的需要设计一个多变量控制器，使柴油机空气系统的输出能够维持在特定的工作设定点 $y_{sp} = [0.4, 0.8]^T$ 附近，并在存在外部阶跃扰动的情况下确保零稳态误差。本任务与前几个任务中目标状态为原点的调节问题不同，本任务要求的是系统跟踪非零参考信号。此外，设计约束要求只有两个系统输出是可测量的，这排除了直接使用全状态反馈的可能性。因此，控制策略必须将用于跟踪和扰动抑制的伺服机制与用于估计不可测内部状态的状态观测器相结合

## 2. Task Solution and Methodology

To satisfy the requirements of asymptotic tracking and disturbance rejection, this project employs a **multivariable integral control** approach, introducing integral action into the control loop. The tracking error is defined as $e(t) = y_{sp} - y(t)$, and a new integral state vector $v(t)$ is introduced such that $\dot{v}(t) = e(t)$. By augmenting these integral states to the original system dynamics, the tracking problem is transformed into a control problem for the augmented system. The augmented state-space model is formulated as in Equation ().

$$
\begin{bmatrix} \dot{x} \\ \dot{v} \end{bmatrix} = 
\begin{bmatrix} A & 0 \\ -C & 0 \end{bmatrix} 
\begin{bmatrix} x \\ v \end{bmatrix} + 
\begin{bmatrix} B \\ 0 \end{bmatrix} u + 
\begin{bmatrix} 0 \\ I \end{bmatrix} y_{sp}
$$

> 为了满足渐近跟踪和扰动抑制的要求，本项目使用**多变量积分控制**方法，在控制回路中引入了积分作用，定义跟踪误差为 $e(t) = y_{sp} - y(t)$，并引入一个新的积分状态向量 $v(t)$ 使得 $\dot{v}(t) = e(t)$。通过将这些积分状态增广到原始系统动力学中，可以将跟踪问题转化为了增广系统的调节问题。增广状态空间模型构方程如式（）。

Following the establishment of the augmented system, the subsequent step involves verifying its controllability. The augmented system is controllable if and only if the original object is controllable and the matrix formed by $A, B,C$ satisfies the rank condition. Verification using the MATLAB functions `ctrb` and `rank` confirms that the system satisfies this condition. Then, employing the LQR method, the optimal feedback gain $K_{aug} = [K_x \quad K_v]$ is designed. The weighting matrix $Q_{aug}$ is tuned to heavily penalise the integral state $v$, thereby forcing the elimination of steady-state error, while $R_{aug}$ is adjusted to maintain reasonable control effort. The control law for the augmented system is formulated as in Equation ().

$$
u(t) = -K_x x(t) - K_v \int_0^t (y_{sp} - y(\tau)) d\tau
$$

> 建立增广系统后，下一步涉及检查其可控性。当且仅当原始对象是可控的，并且由 A、B、C 构成的矩阵满足秩条件时，增广系统才是可控的。在使用 MATLAB 函数 `ctrb` 和 `rank` 验证后发现系统满足该条件。然后，使用**线性二次型调节器 (LQR)** 方法设计最优反馈增益 $K_{aug} = [K_x \quad K_v]$，权重矩阵 $Q_{aug}$ 被调整为重罚积分状态 $v$，从而强制消除稳态误差，而 $R_{aug}$ 被调整以保持合理的控制力度。增广系统的控制律公式如式（）。

However, as the state vector $x(t)$ cannot be directly measured, it requires estimation. Following the principle of separation and observer design theory, this project reuses the **dimension-reduction observer** designed in Task 3. This observer estimates only $n-m=2$ unmeasured states. The observer's dynamic equations are given by Equation ().

$$
\dot{\xi}(t) = D\xi(t) + Eu(t) + Gy(t)
$$

To ensure rapid convergence of estimation errors without disrupting the control loop, the poles of the observer matrix $D$ are placed at four times the distance from the dominant poles of the closed-loop controller system. The full state vector $\hat{x}$ is reconstructed by combining the measured output $y$ and the estimated internal state $\xi$, as per Equation.

$$
\hat{x}(t) = \begin{bmatrix} C \\ T \end{bmatrix}^{-1} \begin{bmatrix} y(t) \\ \xi(t) \end{bmatrix}
$$

Furthermore, the observer gain matrix $L$ is computed using the LQR method (via duality) to ensure the estimation error dynamics are stable and faster than the closed-loop system dynamics. Finally, the resulting control law substitutes the estimated state $\hat{x}$ for the true state $x$, as shown in Equation ().

$$
u(t) = -K_x \hat{x}(t) - K_v v(t)
$$

> 然而，由于状态向量 $x(t)$ 不可直接测量，需要对其进行估计，遵循分离原理和观测器设计理论，本项目复用了在任务3中设计的**降维观测器**。该观测器仅估计 $n-m=2$ 个未测量的状态。观测器动力学方程如式（）给出。
> 
> 为了确保估计误差的快速收敛且不干扰控制回路，观测器矩阵 $D$ 的极点被放置在闭环控制器系统主导极点的4倍处。全状态向量 $\hat{x}$ 通过结合测量输出 $y$ 和估计的内部状态 $\xi$ 进行重构，如式。
> 
> 此外，观测器增益矩阵 $L$ 是使用 LQR 方法（通过对偶性）计算的，以确保估计误差动力学是稳定的，并且比闭环系统动力学更快 。最终，可实现的控制律用估计状态 $\hat{x}$ 替换了真实状态 $x$，如式（）。

The simulation results, as shown in the Figure2-12, demonstrate the effectiveness of this composite control strategy. Simulation results indicate that outputs $y_1$ and $y_2$ precisely track setpoints $0.4$ and $0.8$. When a step disturbance $w = [0.3, 0.2]^T$ is introduced at $t=10s$, the system exhibits transient deviation but rapidly recovers to the setpoint, confirming the effectiveness of the integral action. The state estimation error norm initially converged to zero. Due to the observer's ignorance of $w$'s existence, although a small offset appeared after the disturbance, closed-loop tracking remained robust. It should be noted that in the presence of unknown disturbances, while the observer converged perfectly without disturbance, steady-state estimation errors emerged after disturbance application. This arises because the standard Luenberger observer model assumes knowledge of all inputs to the object. However, the external disturbance $w$ remains unknown to the observer. Despite this estimation bias, the integral action within the outer control loop effectively compensates for both the disturbance and estimation error, ensuring the actual output still converges to the desired setpoint. This highlights the robustness of the integral servo control architecture against process disturbances and estimation uncertainties.

> 仿真结果如图所示,证明了这种复合控制策略的有效性。仿真结果表明，输出 $y_1$ 和 $y_2$ 精确地跟踪了设定点 $0.4$ 和 $0.8$。当 $t=10s$ 引入阶跃扰动 $w = [0.3, 0.2]^T$ 时，系统表现出瞬态偏差，但迅速恢复到设定点，证实了积分作用的有效性。状态估计误差范数最初收敛于零，尽管在扰动后显示出小的偏移（由于观测器不知道 $w$ 的存在），但闭环跟踪仍然保持鲁棒。需要注意的是未知扰动存在时，虽然观测器在没有扰动时完美收敛，但在施加扰动后会出现稳态估计误差。这是因为标准的 Luenberger 观测器模型假设已知对象的所有输入 ，然而，外部扰动 $w$ 对观测器来说是未知的。尽管存在这种估计偏差，外层控制回路中的积分作用有效地补偿了扰动和估计误差，确保实际输出仍然收敛到所需的设定点。这突显了积分伺服控制架构对过程扰动和估计不确定性的鲁棒性。![task5_servo control results](E:\桌面文件存放\学习资料\NUS\NUS\NUS%20Courses\ME5401%20Linear%20System\Linear-System-Mini-Project\task5\task5_servo%20control%20results.png)

## 3. Discussion and Conclusion

In Task 5, this project successfully applied servo control methods to MIMO systems with incomplete state information. By augmenting the state space with an error integrator, to ensure the system satisfies the internal model principle, which is crucial for suppressing constant disturbances with zero steady-state error and tracking step references. Simulations confirmed that integral control is the key to achieving steady-state accuracy. The use of the observer enabled the implementation of a state feedback law utilising only available outputs, demonstrating the practicality of the separation principle.

> 在任务5中，本项目成功地将伺服控制方法应用于具有不完整状态信息的 MIMO 系统。通过用误差积分器增广状态空间，我们确保了系统满足内部模型原理，这对于以零稳态误差抑制恒定扰动和跟踪阶跃参考至关重要 。仿真证实了积分控制是实现稳态精度的“法宝”。观测器的使用使我们能够仅利用可用的输出实现状态反馈律，证明了分离原理的实用性。


