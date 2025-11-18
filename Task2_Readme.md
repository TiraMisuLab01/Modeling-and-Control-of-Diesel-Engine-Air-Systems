# Task 2 Report: LQR Optimal Controller Design

## 1. Introduction

This report addresses Task 2 of the mini-project, which transitions from the pole placement technique to the Linear Quadratic Regulator (LQR) optimal control methodology. While pole placement allows for the deterministic assignment of closed-loop poles to achieve desired dynamics, LQR offers a more elegant approach by formulating the control problem as an optimization task. The objective is to find a state feedback controller, `u = -Kx`, that minimizes a quadratic cost function, thereby systematically balancing the competing goals of system performance (i.e., minimizing state deviation) and control effort (i.e., minimizing energy consumption). This task assumes all state variables are measurable and that the system operates under zero disturbance and zero set-point conditions, focusing purely on the regulation problem. The primary goal is to design an LQR controller that meets the same stringent performance criteria as in Task 1 (overshoot < 10%, settling time < 20s) and, crucially, to analyze and discuss the effect of the weighting matrices `Q` and `R` on the system's behavior.

> 任务二要求：
> 
> 任务二核心在于从极点配置方法过渡到线性二次调节器（LQR）最优控制理论的应用。相较于通过直接指定闭环极点来塑造系统动态的极点配置法，LQR将控制问题转化为一个优化问题，从而提供了一种更为系统化的设计范式。其核心目标是求解一个状态反馈控制器 `u = -Kx`，该控制器能够最小化一个二次型代价函数，用以在系统性能（最小化状态误差）与控制成本（最小化能量消耗）这两个相互制约的目标之间取得最佳平衡。本任务设定在一个理想化的调节问题（Regulation Problem）场景下，即假设所有状态变量均可测量，且系统在零扰动和零设定点下运行。设计的最终目标是获得一个满足性能指标（超调<10%，稳定时间<20s）的LQR控制器，并深入分析权重矩阵`Q`和`R`对系统行为的影响。

## 2. Task Solvement

The LQR design process is centered around the minimization of a quadratic cost function, which, as defined in Chapter 8 (Slide 6), is given by:

> LQR的设计过程围绕着一个二次型代价函数的最小化展开。根据课程讲义第8章（幻灯片6）的定义，该代价函数`J`的形式如下：

$$
J = \frac{1}{2}\int_{0}^{\infty} (x^T Q x + u^T R u) dt
$$

Here, the matrix `Q` penalizes state deviations and `R` penalizes control effort. The optimal feedback gain `K` that minimizes `J` is found by first solving the Algebraic Riccati Equation (ARE) for a unique, positive semi-definite matrix `P` (Chapter 8, Slide 38), and then calculating `K` (Chapter 8, Slide 40):

> 在此函数中，矩阵`Q`用于惩罚状态误差，而矩阵`R`则用于惩罚控制输入的大小。能够最小化`J`的最优反馈增益`K`，其求解过程依赖于代数黎卡提方程（ARE）。首先，需要解出ARE唯一的对称半正定解矩阵`P`（参考第8章，幻灯片38），进而通过该解计算出最优增益`K`（参考第8章，幻灯片40）：

$$
A^TP + PA - PBR^{-1}B^TP + Q = 0
$$

$$
K = R^{-1}B^TP
$$

In our MATLAB script, `task2_lqr_design_EN_1.m`, we use the `lqr(A, B, Q, R)` function, which encapsulates this entire solution process. The core of this task is to investigate the trade-off between performance and cost by adjusting `Q` and `R`. We performed three distinct designs to illustrate this trade-off.

> 在本次设计的MATLAB脚本`task2_lqr_design_EN_1.m`中，我们利用`lqr(A, B, Q, R)`函数来完成上述求解过程。本任务的核心在于通过调整`Q`和`R`来研究性能与成本之间的权衡关系。为此，我们进行了三组不同的设计实验。

**Design 1 (Baseline):** We started with a common and intuitive choice for the weighting matrices: `Q = C'*C` and `R = eye(2)`. This choice penalizes the output `y` directly and applies equal cost to both control inputs. The resulting gain matrix was:

> **设计一 (基准方案):** 我们首先采用了一组在工程中常用且直观的权重矩阵：`Q = C'*C` 与 `R = eye(2)`。该选择的物理意义在于直接对系统的输出`y`进行惩罚，并对两个控制输入施加了同等的权重。计算得到此方案的反馈增益矩阵为：

```
K1 (Q=C'C, R=I):
   -0.2208   -0.6712    0.2823   -0.1697
   -0.1431   -0.5293    0.1556    0.0443
```

**Design 2 (Aggressive):** To achieve a faster response, we increased the state penalty and reduced the control penalty by setting `Q = 100*(C'*C)` and `R = 0.5*eye(2)`. This higher Q/R ratio resulted in a significantly larger gain matrix, indicating a more aggressive control action:

> **设计二 (激进方案):** 为获得更快的系统响应，我们提高了状态惩罚并降低了控制惩罚，具体设置为 `Q = 100*(C'*C)` 和 `R = 0.5*eye(2)`。这种更高的Q/R比值使得控制器增益显著增大，预示着一个更快的控制策略。

```
K2 (Q=100*C'C, R=0.5*I):
  -19.2982  -14.2529    6.7233   -9.3526
  -14.5523  -34.4957    6.6388    5.7409
```

The simulation results clearly illustrate the impact of these choices. The step response comparison shows that the aggressive design has the fastest settling time (approx. 1.1s), while the baseline design is slower but still meets the performance criteria. The quantitative metrics confirm these visual observations.

> 仿真结果清晰地揭示了不同权重选择对系统性能的影响。阶跃响应比较显示，激进型设计（设计二）具有最快的稳定时间（约1.1秒），而基准设计（设计一）虽然响应较慢，但其性能指标同样满足设计要求。具体的性能数据进一步验证了这一观察。



Furthermore, the analysis of the initial condition response and control effort confirms this trade-off. The aggressive controller (Design 2) drives the states to zero fastest but requires a control signal with a peak magnitude of nearly 5 units. In contrast, the baseline design (Design 1) again shows a balanced behavior with moderate control effort (peak < 0.3) and a reasonably fast response.

> 此外，对初始条件响应和相应控制输入的分析进一步证实了这种性能与成本的权衡关系。激进型控制器（设计二）能最快地将系统状态调节至零点，但其代价是需要一个峰值接近5个单位的控制信号。相比之下，基准控制器（设计一）则展现了更为均衡的特性，它以一个适度的控制信号（峰值小于0.3）实现了足够快的响应速度。

![Initial Condition and Control Effort Comparison](task2_control%20signal.png)

## 3. Discussion and Conclusion

This task effectively demonstrated the core principle of LQR design: achieving an optimal balance between system performance and control expenditure. Unlike pole placement, where the designer manually selects pole locations, LQR provides a systematic way to synthesize a controller based on high-level performance objectives encapsulated in the weighting matrices `Q` and `R`. The iterative process of selecting these matrices, simulating the response, and analyzing the results is a fundamental workflow in modern control engineering.

> 本次任务有效地验证了LQR设计的核心思想：在系统性能与控制成本之间寻求最优的平衡点。与需要设计者手动指定极点位置的极点配置法不同，LQR基于封装在高层性能目标中的权重矩阵`Q`和`R`，提供了一种系统化的控制器综合方法。通过选择权重矩阵、仿真系统响应并分析结果的迭代过程，是现代控制工程中的一个基本设计流程。

The key takeaway is the clear and predictable relationship between the Q/R ratio and the system's behavior. A higher Q/R ratio leads to a faster, more aggressive response at the cost of higher control energy, while a lower ratio results in a slower, more conservative response that saves energy. For this specific project, both the baseline and aggressive designs met the transient performance criteria of `ts < 20s`. However, considering the trade-off, the baseline design (`Q=C'*C`, `R=I`) is arguably the better choice as it satisfies the requirements with significantly less control effort. This task successfully fulfills all requirements and provides a solid foundation for understanding optimal control principles.

> 本次设计的关键收获在于深刻理解了Q/R比值与系统行为之间清晰且可预测的关系。提高Q/R比值会带来更快的动态响应，但其代价是更高的控制能量消耗；反之，降低该比值则会使系统响应趋于平缓，从而节省控制成本。针对本项目的具体要求，基准方案与激进方案均满足了`ts < 20s`的稳定时间指标。然而，综合考量性能与成本的平衡，基准设计（`Q=C'*C`, `R=I`）无疑是更优的选择，因为它在满足性能指标的前提下，显著降低了控制器的能量消耗。本任务成功地完成了所有要求，并为理解最优控制原理提供了坚实的基础。
