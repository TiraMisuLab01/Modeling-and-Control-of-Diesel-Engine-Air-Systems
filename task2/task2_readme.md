# Task 2 Report: LQR Optimal Controller Design

## 1. Introduction

This report addresses Task 2 of the mini-project, which transitions from the pole placement technique to the Linear Quadratic Regulator (LQR) optimal control methodology. While pole placement allows for the deterministic assignment of closed-loop poles to achieve desired dynamics, LQR offers a more elegant approach by formulating the control problem as an optimization task. The objective is to find a state feedback controller, `u = -Kx`, that minimizes a quadratic cost function, thereby systematically balancing the competing goals of system performance (i.e., minimizing state deviation) and control effort (i.e., minimizing energy consumption). This task assumes all state variables are measurable and that the system operates under zero disturbance and zero set-point conditions, focusing purely on the regulation problem. The primary goal is to design an LQR controller that meets the same stringent performance criteria as in Task 1 (overshoot < 10%, settling time < 20s) and, crucially, to analyze and discuss the effect of the weighting matrices `Q` and `R` on the system's behavior.

> 任务二要求：
> 
> 任务二放弃使用极点配置方法，转而使用线性二次调节器（LQR）最优控制。相较于通过直接指定闭环极点来塑造系统动态的极点配置法，LQR将控制问题转化为一个优化问题，从而更为系统化的进行设计。其目标是求解一个状态反馈控制器 `u = -Kx`，该控制器能够最小化一个二次型代价函数 J，从而在系统性能（最小化状态误差）与控制成本（最小化能量消耗）这两个相互制约的目标之间取得平衡。因此，本任务需要设计一个满足性能指标（的LQR控制器，并深入分析权重矩阵`Q`和`R`对系统行为的影响。

## 2. Task Solvement

The core of LQR design lies in minimising the quadratic cost function. The quadratic cost function `J` is defined as follows:

> LQR的设计的核心是二次型代价函数的最小化。二次型代价函数`J`的函数如下所示：

$$
J = \frac{1}{2}\int_{0}^{\infty} (x^T Q x + u^T R u) dt
$$

    In this function, matrix `Q` is used to penalise the state error, while matrix `R` penalises the scale of the control input. This project employs the Algebraic Riccati Equation (ARE) to determine the optimal feedback gain `K` that minimises `J`. 
    Firstly, the unique symmetric positive semidefinite solution matrix `P` to the ARE is solved by Eq. (). Subsequently, the optimal gain `K` is computed by Eq. ().

> 在此函数中，矩阵`Q`用于惩罚状态误差，而矩阵`R`则用于惩罚控制输入的大小。本项目使用代数黎卡提方程（ARE）来求解能够最小化`J`的最优反馈增益`K`。首先，通过式（）解出ARE唯一的对称半正定解矩阵`P`，接着通过式（）计算出最优增益`K`

$$
A^TP + PA - PBR^{-1}B^TP + Q = 0
$$

$$
K = R^{-1}B^TP
$$

In our MATLAB script, `task2_lqr_design_EN_1.m`, we use the `lqr(A, B, Q, R)` function, which encapsulates this entire solution process. The core of this task is to investigate the trade-off between performance and cost by adjusting `Q` and `R`. We performed three distinct designs to illustrate this trade-off.In this project, the MATLAB function `lqr(A, B, Q, R)` is used to calculate the above-mentioned solution. Additionally, to investigate the trade-off between performance and computational cost, two sets of `Q` and `R` matrices have been set up for experimentation.

> 在本项目中，matlab的`lqr(A, B, Q, R)`函数被用来完成上述求解过程。同时，为了研究性能与成本之间的权衡关系，本项目设置了两组`Q`和`R`矩阵来进行实验。

Design 1 (Baseline): This project initially used a set of weight matrices which commonly used in engineering and intuitive: `Q = C^T*C` and `R = I` to minimise the output energy $y^Ty$, applying equal weighting to both control inputs. Subsequently, the feedback gain matrix K_1 obtained from this approach was calculated as:

> **设计一 (基准方案):** 本项目首先采用了一组在工程中常用且直观的权重矩阵：`Q = C^T*C` 与 `R = eye(2)`来最小化输出的能量 $y^Ty$，并且对两个控制输入施加了同等的权重。随后，计算此方案得到的反馈增益矩阵K_1为：

```
K1 (Q=C'C, R=I):
   -0.2208   -0.6712    0.2823   -0.1697
   -0.1431   -0.5293    0.1556    0.0443
```

Design 2 (Aggressive): the design increased the state penalty and reduced the control penalty to achieve faster system response, specifically setting `Q = 100*(C^T*C)` and `R = 0.5I`. Consequently, this approach exhibits a higher Q/R ratio, shorter settling time, and larger control signal amplitude. Subsequently, the feedback gain matrix K₁ derived from this design is calculated as:

> **设计二 (激进方案):** 为了获得更快的系统响应，我们提高了状态惩罚并降低了控制惩罚，具体来说，设置 `Q = 100*(C^T*C)` 和 `R = 0.5I`。因此，此方案具有更高的Q/R比值，更短的调节时间，更大的控制信号幅值。随后，计算此方案得到的反馈增益矩阵K_1为：

```
K2 (Q=100*C'C, R=0.5*I):
  -19.2982  -14.2529    6.7233   -9.3526
  -14.5523  -34.4957    6.6388    5.7409
```

Eventually, simulations were conducted using MATLAB to examine the impact of different weight selections on system performance during step responses. The performance indicators for two designs are presented in Table x-x. Figure x-x demonstrates that Design 1 exhibits not only a slower response but also higher overshoot which is over 10% of the setpoint. Therefore, Design 1 fails to meet requirements. Due to the controller imposes overly stringent constraints on control energy, it cannot exert sufficient control force to suppress overshoot. In contrast, Design 2 exhibits a faster settling time, rapidly bringing the system state to zero, and its performance indicators meet the requirements. However, this comes at the cost of requiring a control signal with a peak magnitude of approximately 6 units.

> 最后，使用matlab 进行仿真，检查阶跃响应下不同权重选择对系统性能的影响，两个设计的性能指标如表 x-x所示。图 x-x显示，设计一不仅响应较慢，而且超调量M_P超过了10%。因此设计一无法满足要求，在此设计下控制器对控制能量的限制过于严格，导致无法施加足够的控制力来抑制超调。相比之下，设计二2 具有更快的稳定时间，能最快地将系统状态调节至零点，并且性能指标满足要求。但其代价是需要一个峰值约为6个单位的控制信号。

![task2_initial condition response and control effort comparison](E:\桌面文件存放\学习资料\NUS\NUS\NUS%20Courses\ME5401%20Linear%20System\Linear-System-Mini-Project\task2\task2_initial%20condition%20response%20and%20control%20effort%20comparison.png)

## 3. Discussion and Conclusion

Task 2 demonstrated a profound understanding of how to find the optimal balance between system performance and control costs by adjusting the Q and R settings. Increasing the Q/R ratio will result in faster dynamic response, but at the expense of higher control energy consumption; conversely, reducing this ratio will cause the system response to become more gradual, thereby saving control costs.

> 任务2通过调整Q与R的设置，深刻的理解了如何在系统性能与控制成本之间寻求最优的平衡点。提高Q/R比值会带来更快的动态响应，但其代价是更高的控制能量消耗；反之，降低该比值则会使系统响应趋于平缓，从而节省控制成本。针对本项目的具体要求，只有设计2满足了所有的性能指标。
