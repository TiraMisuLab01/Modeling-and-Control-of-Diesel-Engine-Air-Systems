# Task 5: Servo Control Design with State Estimation for Disturbance Rejection

## 1. Introduction

The primary objective of Task 5 is to design a multivariable controller capable of maintaining the diesel engine air system outputs around a specific operating set point, $y_{sp} = [0.4, 0.8]^T$, while ensuring zero steady-state error in the presence of external step disturbances. Unlike the regulation problems in previous tasks where the target state was the origin, this task requires the system to track non-zero reference signals. Furthermore, the design constraints specify that only the two system outputs are measurable, which precludes the direct use of full state feedback. Consequently, the control strategy must integrate a servo mechanism for tracking and disturbance rejection with a state observer for estimating the unmeasurable internal states. This problem represents a synthesis of the servo control theory and state estimation techniques.

> 任务5的主要目标是设计一个多变量控制器，使柴油机空气系统的输出能够维持在特定的工作设定点 $y_{sp} = [0.4, 0.8]^T$ 附近，并在存在外部阶跃扰动的情况下确保零稳态误差。与前几个任务中目标状态为原点的调节问题不同，本任务要求系统跟踪非零参考信号。此外，设计约束指定只有两个系统输出是可测量的，这排除了直接使用全状态反馈的可能性。因此，控制策略必须将用于跟踪和扰动抑制的伺服机制与用于估计不可测内部状态的状态观测器相结合。这个问题代表了伺服控制理论和状态估计技术的综合应用。

## 2. Task Solution and Methodology

[cite_start]To address the requirements of asymptotic tracking and disturbance rejection, we adopt the **Multivariable Integral Control** approach outlined in the course materials[cite: 4504, 4527]. The standard state feedback law $u = -Kx$ is insufficient because it cannot guarantee zero steady-state error against constant disturbances or non-zero references due to the lack of internal models. Therefore, we introduce integral action into the control loop. We define the tracking error as $e(t) = y_{sp} - y(t)$ and introduce a new integral state vector $v(t)$ such that $\dot{v}(t) = e(t)$. By augmenting the original system dynamics with these integral states, we transform the tracking problem into a regulation problem for an augmented system. The augmented state space model is constructed as follows:

$$
\begin{bmatrix} \dot{x} \\ \dot{v} \end{bmatrix} = 
\begin{bmatrix} A & 0 \\ -C & 0 \end{bmatrix} 
\begin{bmatrix} x \\ v \end{bmatrix} + 
\begin{bmatrix} B \\ 0 \end{bmatrix} u + 
\begin{bmatrix} 0 \\ I \end{bmatrix} y_{sp}
$$

> [cite_start]为了满足渐近跟踪和扰动抑制的要求，我们采用了课程材料中概述的**多变量积分控制**方法 [cite: 4504, 4527]。标准的状态反馈律 $u = -Kx$ 是不足够的，因为由于缺乏内部模型，它无法保证对抗恒定扰动或非零参考信号时的零稳态误差。因此，我们在控制回路中引入了积分作用。我们定义跟踪误差为 $e(t) = y_{sp} - y(t)$，并引入一个新的积分状态向量 $v(t)$ 使得 $\dot{v}(t) = e(t)$。通过将这些积分状态增广到原始系统动力学中，我们将跟踪问题转化为了增广系统的调节问题。增广状态空间模型构建如上所示。

With the augmented system established, the next step involves checking its controllability. [cite_start]As stated in the lecture notes, the augmented system is controllable if and only if the original plant is controllable and the matrix formed by A, B, C satisfies the rank condition[cite: 4542]. Upon verification using the MATLAB function `ctrb` and `rank`, the condition is met. We then proceed to design the optimal feedback gain $K_{aug} = [K_x \quad K_v]$ using the **Linear Quadratic Regulator (LQR)** method. The weighting matrix $Q_{aug}$ is tuned to heavily penalize the integral states $v$, thereby enforcing the elimination of steady-state errors, while $R_{aug}$ is adjusted to maintain reasonable control efforts. The control law for the augmented system is formulated as:

$$
u(t) = -K_x x(t) - K_v \int_0^t (y_{sp} - y(\tau)) d\tau
$$

> [cite_start]建立增广系统后，下一步涉及检查其可控性。如讲义所述，当且仅当原始对象是可控的，并且由 A、B、C 构成的矩阵满足秩条件时，增广系统才是可控的 [cite: 4542]。在使用 MATLAB 函数 `ctrb` 和 `rank` 验证后，满足该条件。然后，我们使用**线性二次型调节器 (LQR)** 方法设计最优反馈增益 $K_{aug} = [K_x \quad K_v]$。权重矩阵 $Q_{aug}$ 被调整为重罚积分状态 $v$，从而强制消除稳态误差，而 $R_{aug}$ 被调整以保持合理的控制力度。增广系统的控制律公式如上所示。

However, since the state vector $x(t)$ is not directly measurable, we must estimate it. [cite_start]Following the separation principle and the observer design theory in Chapter 11[cite: 531], we construct a full-order state observer. The observer dynamics are given by:

$$
\dot{\hat{x}}(t) = A\hat{x}(t) + Bu(t) + L(y(t) - C\hat{x}(t))
$$

[cite_start]The observer gain matrix $L$ is calculated using the LQR method (by duality) to ensure that the estimation error dynamics are stable and faster than the closed-loop system dynamics[cite: 643]. Finally, the implementable control law replaces the true state $x$ with the estimated state $\hat{x}$:

$$
u(t) = -K_x \hat{x}(t) - K_v v(t)
$$

> [cite_start]然而，由于状态向量 $x(t)$ 不可直接测量，我们要对其进行估计。遵循分离原理和第11章中的观测器设计理论 [cite: 531][cite_start]，我们构建了一个全维状态观测器。观测器动力学方程如上给出。观测器增益矩阵 $L$ 是使用 LQR 方法（通过对偶性）计算的，以确保估计误差动力学是稳定的，并且比闭环系统动力学更快 [cite: 643]。最终，可实现的控制律用估计状态 $\hat{x}$ 替换了真实状态 $x$。

The design was implemented in MATLAB. The calculated gains are:
$K_x = \begin{bmatrix} -24.1981 & 25.1242 & 2.5621 & -17.9027 \\ 1.9207 & -65.3159 & 8.1972 & 19.7586 \end{bmatrix}$,
$K_v = \begin{bmatrix} -85.4065 & 52.0167 \\ -52.0167 & -85.4065 \end{bmatrix}$.
The simulation results show that the outputs $y_1$ and $y_2$ precisely track the setpoints $0.4$ and $0.8$. When a step disturbance $w = [0.3, 0.2]^T$ is introduced at $t=10s$, the system exhibits a transient deviation but rapidly recovers to the setpoints, confirming the effectiveness of the integral action. The state estimation error norm converges to zero initially, and although it shows a small offset after the disturbance (due to the observer being unaware of $w$), the closed-loop tracking remains robust.

> 该设计已在 MATLAB 中实现。计算得到的增益如上所示。<img title="" src="file:///E:/桌面文件存放/学习资料/NUS/NUS/NUS%20Courses/ME5401%20Linear%20System/Linear-System-Mini-Project/task5/task5_servo%20control%20results.png" alt="task5_servo control results" style="zoom:50%;">仿真结果表明，输出 $y_1$ 和 $y_2$ 精确地跟踪了设定点 $0.4$ 和 $0.8$。当 $t=10s$ 引入阶跃扰动 $w = [0.3, 0.2]^T$ 时，系统表现出瞬态偏差，但迅速恢复到设定点，证实了积分作用的有效性。状态估计误差范数最初收敛于零，尽管在扰动后显示出小的偏移（由于观测器不知道 $w$ 的存在），但闭环跟踪仍然保持鲁棒。

## 3. Discussion and Conclusion

In this task, we successfully applied the servo control methodology to a MIMO system with incomplete state information. [cite_start]By augmenting the state space with error integrators, we ensured that the system satisfies the internal model principle, which is crucial for rejecting constant disturbances and tracking step references with zero steady-state error[cite: 4210]. The simulation confirms that the integral control is the "magic weapon" for steady-state accuracy. The use of an observer allowed us to implement the state feedback law using only the available outputs, demonstrating the practicality of the separation principle.

> [cite_start]在本任务中，我们成功地将伺服控制方法应用于具有不完整状态信息的 MIMO 系统。通过用误差积分器增广状态空间，我们确保了系统满足内部模型原理，这对于以零稳态误差抑制恒定扰动和跟踪阶跃参考至关重要 [cite: 4210]。仿真证实了积分控制是实现稳态精度的“法宝”。观测器的使用使我们能够仅利用可用的输出实现状态反馈律，证明了分离原理的实用性。

One notable observation from the results is the behavior of the estimation error in the presence of unknown disturbances. While the observer converges perfectly when there is no disturbance, a steady-state estimation error appears after the disturbance is applied. [cite_start]This occurs because the standard Luenberger observer model assumes knowledge of all inputs to the plant[cite: 631]; however, the external disturbance $w$ is unknown to the observer. Despite this estimation bias, the integral action in the outer control loop effectively compensates for both the disturbance and the estimation error, ensuring that the actual outputs still converge to the desired set points. This highlights the robustness of the integral servo control architecture against both process disturbances and estimation uncertainties.

> [cite_start]结果中一个值得注意的观察是未知扰动存在时的估计误差行为。虽然观测器在没有扰动时完美收敛，但在施加扰动后会出现稳态估计误差。这是因为标准的 Luenberger 观测器模型假设已知对象的所有输入 [cite: 631]；然而，外部扰动 $w$ 对观测器来说是未知的。尽管存在这种估计偏差，外层控制回路中的积分作用有效地补偿了扰动和估计误差，确保实际输出仍然收敛到所需的设定点。这突显了积分伺服控制架构对过程扰动和估计不确定性的鲁棒性。
