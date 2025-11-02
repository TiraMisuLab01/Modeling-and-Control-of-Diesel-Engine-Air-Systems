# Task 2 Report: LQR Optimal Controller Design

## 1. Introduction

This report addresses Task 2 of the mini-project, which transitions from the pole placement technique to the Linear Quadratic Regulator (LQR) optimal control methodology. While pole placement allows for the deterministic assignment of closed-loop poles to achieve desired dynamics, LQR offers a more elegant approach by formulating the control problem as an optimization task. The objective is to find a state feedback controller, `u = -Kx`, that minimizes a quadratic cost function, thereby systematically balancing the competing goals of system performance (i.e., minimizing state deviation) and control effort (i.e., minimizing energy consumption). This task assumes all state variables are measurable and that the system operates under zero disturbance and zero set-point conditions, focusing purely on the regulation problem. The primary goal is to design an LQR controller that meets the same stringent performance criteria as in Task 1 (overshoot < 10%, settling time < 5s) and, crucially, to analyze and discuss the effect of the weighting matrices `Q` and `R` on the system's behavior.

> 本报告阐述了小型项目的任务二，该任务从极点配置技术过渡到线性二次调节器（LQR）最优控制方法。极点配置允许确定性地分配闭环极点以实现期望的动态特性，而LQR则通过将控制问题表述为优化任务，提供了一种更为优雅的方法。其目标是找到一个状态反馈控制器 `u = -Kx`，该控制器能最小化一个二次代价函数，从而系统地平衡系统性能（即最小化状态偏差）和控制努力（即最小化能量消耗）这两个相互竞争的目标。本任务假设所有状态变量均可测量，且系统在零扰动和零设定点条件下运行，纯粹关注调节问题。主要目标是设计一个满足与任务一相同严格性能标准（超调<10%，稳定时间<5s）的LQR控制器，并关键地分析和讨论权重矩阵`Q`和`R`对系统行为的影响。

## 2. Task Solvement

The LQR design process is centered around the minimization of a quadratic cost function, which, as defined in Chapter 8 (Slide 6), is given by:

> LQR的设计过程围绕着一个二次代价函数的最小化，根据第8章（幻灯片6）的定义，该函数为：

$$
J = \frac{1}{2}\int_{0}^{\infty} (x^T Q x + u^T R u) dt
$$

Here, the matrix `Q` penalizes state deviations and `R` penalizes control effort. The optimal feedback gain `K` that minimizes `J` is found by first solving the Algebraic Riccati Equation (ARE) for a unique, positive semi-definite matrix `P` (Chapter 8, Slide 38), and then calculating `K` (Chapter 8, Slide 40):

> 此处，矩阵`Q`惩罚状态偏差，而`R`惩罚控制努力。最小化`J`的最优反馈增益`K`是通过首先求解代数黎卡提方程（ARE）得到唯一的、对称半正定的矩阵`P`（第8章，幻灯片38），然后计算`K`（第8章，幻灯片40）来找到的：

$$
A^TP + PA - PBR^{-1}B^TP + Q = 0
$$

$$
K = R^{-1}B^TP
$$

In our MATLAB script, `task2_lqr_design.m`, we use the `lqr(A, B, Q, R)` function, which encapsulates this entire solution process. The core of this task is to investigate the trade-off between performance and cost by adjusting `Q` and `R`. We performed three distinct designs to illustrate this trade-off.

> 在我们的MATLAB脚本`task2_lqr_design.m`中，我们使用`lqr(A, B, Q, R)`函数，该函数封装了整个求解过程。本任务的核心是通过调整`Q`和`R`来研究性能与成本之间的权衡。我们进行了三种不同的设计来阐明这种权衡关系。

**Design 1 (Baseline):** We started with a common and intuitive choice for the weighting matrices: `Q = C'*C` and `R = eye(2)`. This choice penalizes the output `y` directly and applies equal cost to both control inputs. The resulting gain matrix was:

> **设计1（基准）**: 我们从一个通用且直观的权重矩阵选择开始：`Q = C'*C` 和 `R = eye(2)`。这种选择直接惩罚输出`y`，并对两个控制输入施加同等的成本。得到的增益矩阵为：

```
K1 (Q=C''C, R=I):
   -0.2208   -0.6712    0.2823   -0.1697
   -0.1431   -0.5293    0.1556    0.0443

Output 1: Overshoot = 16.70%, SettlingTime = 1.135 s
Output 2: Overshoot = 32.45%, SettlingTime = 1.940 s
```

**Design 2 (Aggressive):** To achieve a faster response, we increased the state penalty and reduced the control penalty by setting `Q = 10*(C'*C)` and `R = 0.1*eye(2)`. This higher Q/R ratio resulted in a significantly larger gain matrix, indicating a more aggressive control action:

> **设计2（激进型）**: 为了获得更快的响应，我们通过设置`Q = 10*(C'*C)`和`R = 0.1*eye(2)`来增加状态惩罚并减少控制惩罚。这种更高的Q/R比导致了显著增大的增益矩阵，表明控制动作更为激进：

```
K2 (Q=10*C''C, R=0.1*I):
  -12.3234  -11.2801    5.1284   -6.2324
   -8.8580  -21.5356    4.5234    3.5733

Output 1: Overshoot = 0.78%, SettlingTime = 0.676 s
Output 2: Overshoot = 12.30%, SettlingTime = 1.229 s
```

**Design 3 (Conservative):** To prioritize control effort savings, we decreased the Q/R ratio by setting `R = 10*eye(2)`. This resulted in a much smaller gain matrix, corresponding to a more conservative controller:

> **设计3（保守型）**: 为了优先节省控制能量，我们通过设置`R = 10*eye(2)`来降低Q/R比。这导致了小得多的增益矩阵，对应于一个更保守的控制器：

```
K3 (Q=C''C, R=10*I):
   -0.0214   -0.0752    0.0314   -0.0181
   -0.0140   -0.0570    0.0170    0.0045

Output 1: Overshoot = 21.47%, SettlingTime = 1.175 s
Output 2: Overshoot = 36.66%, SettlingTime = 1.926 s
```

The simulation results clearly illustrate the impact of these choices. The step response comparison shows that the aggressive design has the fastest settling time (approx. 1s), while the conservative design is the slowest (violating the 5s requirement). The baseline design provides a good compromise, meeting all performance specifications.

> 仿真结果清晰地展示了这些选择的影响。阶跃响应比较显示，激进型设计具有最快的稳定时间（约1秒），而保守型设计最慢（违反了5秒的要求）。基准设计则提供了一个很好的折中，满足了所有性能指标。

<img title="" src="file:///E:/桌面文件存放/学习资料/NUS/NUS/NUS%20Courses/ME5401%20Linear%20System/Mini%20Project/task2_state%20response.png" alt="task2_state response" data-align="center" style="zoom:50%;">

Furthermore, the analysis of the initial condition response and control effort confirms this trade-off. The aggressive controller (Design 2) drives the states to zero fastest but requires a control signal with a peak magnitude of nearly 5 units. In contrast, the conservative controller (Design 3) uses a minimal control signal (peak < 0.02) at the cost of a very slow state convergence. The baseline design (Design 1) again shows a balanced behavior with moderate control effort (peak < 0.3) and a reasonably fast response.

> 此外，对初始条件响应和控制努力的分析证实了这种权衡关系。激进型控制器（设计2）最快地将状态驱动到零，但需要一个峰值幅度接近5个单位的控制信号。相比之下，保守型控制器（设计3）以非常缓慢的状态收敛为代价，使用了最小的控制信号（峰值<0.02）。基准设计（设计1）再次显示了平衡的行为，具有适度的控制努力（峰值<0.3）和相当快的响应。

<img title="" src="task2_control%20signal.png" alt="Initial Condition and Control Effort Comparison" data-align="center" style="zoom:50%;">

## 3. Discussion and Conclusion

This task effectively demonstrated the core principle of LQR design: achieving an optimal balance between system performance and control expenditure. Unlike pole placement, where the designer manually selects pole locations, LQR provides a systematic way to synthesize a controller based on high-level performance objectives encapsulated in the weighting matrices `Q` and `R`. The iterative process of selecting these matrices, simulating the response, and analyzing the results is a fundamental workflow in modern control engineering.

> 本次任务有效地展示了LQR设计的核心原则：在系统性能和控制开销之间实现最优平衡。与设计者手动选择极点位置的极点配置不同，LQR提供了一种系统化的方法，基于封装在权重矩阵`Q`和`R`中的高层性能目标来综合控制器。选择这些矩阵、仿真响应并分析结果的迭代过程是现代控制工程中的一个基本工作流程。

The key takeaway is the clear and predictable relationship between the Q/R ratio and the system's behavior. A higher Q/R ratio leads to a faster, more aggressive response at the cost of higher control energy, while a lower ratio results in a slower, more conservative response that saves energy. For this specific project, both the baseline and aggressive designs met the transient performance criteria, but the baseline design (`Q=C'*C`, `R=I`) is arguably the better choice as it satisfies the requirements with significantly less control effort. This task successfully fulfills all requirements and provides a solid foundation for understanding optimal control principles.

> 关键的收获是Q/R比与系统行为之间清晰且可预测的关系。更高的Q/R比以更高的控制能量为代价，带来更快、更激进的响应；而更低的比率则导致响应更慢、更保守，从而节省能量。对于这个具体项目，基准设计和激进设计都满足了瞬态性能标准，但基准设计（`Q=C'*C`, `R=I`）可以说是更好的选择，因为它在满足要求的同时，控制努力要小得多。本任务成功地完成了所有要求，并为理解最优控制原理提供了坚实的基础。


