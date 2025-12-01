# Task 6: State Regulation and Optimization Report

## Introduction

Task 6's primary objective extends beyond the study of control outputs to encompass the direct regulation of the system's four internal state variables. Specifically, the state vector $x$ must be maintained at a specific steady-state setpoint $x_{sp} = [0, 0.5, -0.4, 0.3]^T$. Consequently, this task requires rigorous feasibility analysis to establish whether the target state is physically attainable. If it is confirmed that precise attainment of the target is impossible due to the system's physical structure, the problem transforms into an optimisation task: finding the optimal reachable steady state $x_s^*$ that minimises the weighted quadratic cost function $J(x_s)$, thereby balancing errors across different states.

> 任务6的主要目标不仅仅是研究控制输出，而且要研究系统四个内部状态变量的直接调节。具体而言，需要将状态向量 $x$ 维持在特定的稳态设定点 $x_{sp} = [0, 0.5, -0.4, 0.3]^T$。因此，该任务需要进行严格的可行性分析，以确定目标状态在物理上是否可达。如果确认由于系统的物理结构导致无法精确达到目标后，该问题将转化为一个优化问题，寻找最优的可达稳态 $x_s^*$，使其最小化加权二次代价函数 $J(x_s)$来权衡不同状态之间的误差。

## Tasks Solvement

First, a fundamental analysis of the system's equilibrium point is conducted. For a linear time-invariant system described by $\dot{x} = Ax + Bu$, steady state implies that the rate of change of the state vector is zero. Therefore, to maintain the system at the target setpoint $x_{sp}$, a constant control input $u_{ss}$ must exist that satisfies the equilibrium Equation（）

$0 = Ax_{sp} + Bu_{ss}$.

This can be rearranged into the linear algebraic form $Bu_{ss} = -Ax_{sp}$. Using MATLAB, this project analysed the rank of the control input matrix $B$ and the augmented matrix $[B, -A x_{sp}]$. Results indicate that while $B$ has rank 2, the augmented matrix possesses rank 3. According to the Rouché–Capelli theorem, this discrepancy indicates that the target vector $-A x_{sp}$ does not lie within the column space of $B$, rendering precise adjustment to $x_{sp}$ theoretically infeasible.

> 首先对系统的平衡点进行基础分析。对于由 $\dot{x} = Ax + Bu$ 描述的线性时不变系统，稳态意味着状态向量的变化率为零。因此，为了将系统维持在目标设定点 $x_{sp}$，必须存在满足平衡方程 $0 = A x_{sp} + B u_{ss}$ 的恒定控制输入 $u_{ss}$。这可以重排为线性代数形式 $B u_{ss} = -A x_{sp}$。利用 MATLAB，本项目分析了控制输入矩阵 $B$ 和增广矩阵 $[B, -A x_{sp}]$ 的秩。结果表明，虽然 $B$ 的秩为 2，但增广矩阵的秩为 3。根据鲁谢-卡佩利定理（Rouché–Capelli theorem），这一差异表明了目标向量 $-A x_{sp}$ 不在 $B$ 的列空间内，使得精确调节到 $x_{sp}$ 在理论上不可行。

As the original objective proved unfeasible, this project formulated a constrained optimisation problem to identify the optimal achievable steady state. The cost function is defined as:

$J(x_s) = \frac{1}{2}(x_s - x_{sp})^T W (x_s - x_{sp})$,

Where $W$ is the diagonal weight matrix derived from the student ID parameters, yielding $W = \text{diag}(9, 5, 1, 2)$. In order to address this issue, this project utilised the relationship $x_s = -A^{-1}B u_s$ to derive the mapping between steady-state inputs and states, setting $M = -A^{-1}B$. Substituting this into the cost function transforms the problem into an unconstrained minimisation problem concerning the input $u_s$. The analytical solution for the optimal steady-state input is derived by setting the gradient of the objective function to zero:

 $u_s^* = (M^T W M)^{-1} M^T W x_{sp}$.

Subsequently, using MATLAB matrix operations, the optimal steady-state $x_s^*$ was computed, with a minimum cost of approximately $0.218$. The numerical results indicate that states with higher weights like $x_1$ with weight 9 converge very close to the target, while states with lower weights like $x_3$ with weight 1 exhibit significant deviation, thereby validating the optimisation logic.

> 由于原始目标不可行，本项目制定了一个约束优化问题来寻找最佳可达稳态。定义代价函数为
> 
>  $J(x_s) = \frac{1}{2}(x_s - x_{sp})^T W (x_s - x_{sp})$
> 
> 其中 $W$ 是根据学号参数 ($a=8, b=4, c=0, d=1$) 导出的对角权重矩阵，结果为 $W = \text{diag}(9, 5, 1, 2)$。为了解决这个问题，本项目利用关系式 $x_s = -A^{-1}B u_s$ 推导了稳态输入与状态之间的映射，令 $M = -A^{-1}B$。将其代入代价函数，问题转化为关于输入 $u_s$ 的无约束最小化问题。通过令目标函数的梯度为零来推导出了最优稳态输入的解析解：
> 
>  $u_s^* = (M^T W M)^{-1} M^T W x_{sp}$
> 
> 利用 MATLAB 矩阵运算，计算出最优稳态 $x_s^*$，最小代价约为 $0.218$。数值结果表明，权重较高的状态（如权重为 9 的 $x_1$）收敛得非常接近目标，而权重较低的状态（如权重为 1 的 $x_3$）表现出较大的偏差，验证了优化逻辑。

Finally, this project designed a controller to implement the control strategy and ensure asymptotic convergence to the computed optimal steady-state $x_s^*$. This control structure integrates a feedforward component for maintaining equilibrium and a feedback component for stability. The control law is defined as:

 $u(t) = u_s^* - K(x(t) - x_s^*)$,

Where $u_s^*$ provides the energy required to maintain the optimal steady state, and the feedback gain $K$ is employed to stabilise the error dynamics. The gain matrix $K$ is computed using the LQR method to ensure robust transient performance.

The simulation results are shown in Figures 2-13, 2-14. Starting from the initial condition $x_0$, the state track converges smoothly and stabilises precisely at $x_s^*$, whilst the control input stabilises at $u_s^*$.

> 最后，本项目设计了一个控制器来实施控制策略，并确保渐近收敛到计算出的最优稳态 $x_s^*$。该控制结构集成了用于维持平衡的前馈分量和用于稳定的反馈分量，控制律定义为：
> 
>  $u(t) = u_s^* - K(x(t) - x_s^*)$
> 
> 其中 $u_s^*$ 提供维持最优稳态所需的能量，反馈增益 $K$ 用于稳定误差动态。增益矩阵 $K$ 使用 LQR 方法计算，以确保鲁棒的瞬态性能。
> 
> 仿真结果如图2-13，2-14所示，从初始条件 $x_0$ 开始，状态轨迹平滑收敛并精确稳定在 $x_s^*$，控制输入稳定在 $u_s^*$。~

## Discussion and Conclusion

Task 6 comprehensively applied the entire linear system theory, revealing discrepancies between theoretical feasibility analysis and practical control implementation. The unavailability of meeting the original setpoint also demonstrated that actuators typically possess finite degrees of freedom relative to the system's complexity. The optimisation process within this project also successfully demonstrated the strategic use of weighting matrices to prioritise critical performance variables amidst unavoidable trade-offs: achieving high precision for state $x_1$ while permitting greater error tolerance for $x_3$.

Furthermore, the calculated optimal point successfully stabilised the system, validating the effectiveness of combining steady-state feedforward control with optimal state feedback.

> 任务6综合运用整个线性系统理论，发现了理论可行性分析与实际控制实施之间的差距，无法满足原始设定点的情况也表明了与系统的复杂性（状态数量）相比，执行器通常具有有限的自由度。本项目的优化过程也成功展示了在不可避免的权衡中，如何策略性地使用权重矩阵来优先考虑关键的性能变量：状态 $x_1$ 实现高精度与 $x_3$ 接受较大误差。此外，在计算出的最优点成功让系统稳定，也验证了结合稳态前馈控制与最优状态反馈的有效性。

In summary, these 6 tasks systematically explore pole placement, LQR, observer design, decoupling, servo control, and state regulation. Each task builds upon its predecessor, ultimately converging into an optimisation problem that demands a holistic understanding of system dynamics, controllability, and steady-state analysis.

> 总的来说，这六个任务系统地探索了极点配置、LQR、观测器设计、解耦、伺服控制和状态调节，并且几乎每个任务都建立在之前的任务基础之上，最终汇聚成这个需要对系统动力学、可控性和稳态分析有整体理解的优化问题。
