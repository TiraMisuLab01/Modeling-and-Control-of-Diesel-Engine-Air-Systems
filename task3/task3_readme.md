Task 3 Report: Reduced-Order Observer Design for Diesel Engine Control
======================================================================

## 1. Introduction

In the context of the multivariable control design for the diesel engine air system, the preceding tasks operated under the assumption that the full state vector is accessible for feedback. However, in practical engineering scenarios, measuring all state variables is often unfeasible due to the high cost of sensors or the physical inaccessibility of internal states. Furthermore, as the state variables in this project are derived from system identification, they lack explicit physical meanings, making direct measurement impossible. The project specification for Task 3 addresses this reality by restricting available information to the two system outputs and the control inputs. Consequently, a state observer is required to reconstruct the full state vector to implement the Linear Quadratic Regulator (LQR) designed in Task 2.

> 之前的任务1-2是在假设全状态向量可用于反馈的情况下进行的。然而，在实际工程场景中，由于传感器成本高昂或内部状态物理上不可接近，测量所有状态变量往往是不可行的。此外，由于本项目中的状态变量源于系统辨识，它们缺乏明确的物理意义，使得直接测量变得不太可能。任务 3 通过将可用信息限制为两个系统输出和控制输入来模拟这一现实问题。因此，需要一个状态观测器来重构全状态向量，以实现任务 2 中设计的线性二次调节器 (LQR)。

## 2. Tasks Solvement

### 2.1 Theoretical Framework of Reduced-Order Observer

123

> 虽然全阶观测器可以用来模拟整个系统的动态来估计所有四个状态变量，但它的设计是冗余的。因为两个测量输出已经提供了关于状态线性组合的精确信息。鉴于该系统有四个状态 ($n=4$) 和两个独立输出 ($m=2$)，理论上只需要估计剩余的 $n-m=2$ 个状态就足够了，且计算效率更高。因此，本报告重点关注降维观测器的设计与实现。主要目标是综合一个能够确保估计误差渐近收敛的降维观测器，并通过分析不同的观测器极点位置，研究收敛速度与控制信号完整性之间的权衡。

The design methodology for the reduced-order observer strictly follows the theoretical framework presented in Chapter 11 of the lecture notes. The core concept involves transforming the original state vector $x \in \mathbb{R}^n$ into a new coordinate system that consists of the measurable outputs $y \in \mathbb{R}^m$ and a reduced state vector $\xi \in \mathbb{R}^{n-m}$. This linear transformation is defined by a nonsingular matrix composed of the output matrix $C$ and a transformation matrix $T$ to be designed:

$\begin{bmatrix} y \\ \xi \end{bmatrix} = \begin{bmatrix} C \\ T \end{bmatrix} x$

where $\xi$ represents the internal states of the observer. The dynamics of the reduced-order observer are governed by the following differential equation:

$\dot{\xi} = D\xi + Eu + Gy$

where $D$, $E$, and $G$ are the observer matrices of appropriate dimensions.

> 降维观测器的设计需要将原始状态向量 $x \in \mathbb{R}^n$ 变换到一个新的坐标系，该坐标系由可测输出 $y \in \mathbb{R}^m$ 和降维状态向量 $\xi \in \mathbb{R}^{n-m}$ 组成。该线性变换由一个包含输出矩阵 $C$ 和待设计变换矩阵 $T$ 的非奇异矩阵，其中 $\xi$ 代表观测器的内部状态。降维观测器的动态由以下微分方程描述，其中 $D$、$E$ 和 $G$ 是相应维度的观测器矩阵。

To ensure that the estimated state $\xi$ correctly tracks the transformation $Tx$ and that the estimation error $e(t) = \xi(t) - Tx(t)$ decays to zero asymptotically, the observer matrices must satisfy the specific algebraic constraints derived from the error dynamics equation $\dot{e} = De + (DT - TA + GC)x + (E - TB)u$. These constraints are:

1. Matrix $D$ must be stable, meaning all its eigenvalues must have negative real parts.

2. The matrix $T$ must satisfy the Sylvester equation: $TA - DT = GC$.

3. The input matrix $E$ must be matched such that $E = TB$.

> 为了确保估计状态 $\xi$ 正确跟踪变换 $Tx$，并且估计误差 $e(t) = \xi(t) - Tx(t)$ 渐近衰减为零，观测器矩阵必须满足由误差动态方程导出的特定代数约束。这些约束包括：
> 
> 1. 矩阵 $D$ 必须是稳定的，即其所有特征值的实部必须为负。
> 2. 矩阵 $T$ 必须满足 Sylvester 方程：$TA - DT = GC$。
> 3. 输入矩阵 $E$ 必须匹配，使得 $E = TB$。

Upon successfully estimating $\xi(t)$, the full state estimate $\hat{x}(t)$ is reconstructed by inverting the transformation matrix. This reconstruction combines the noise-free measurement $y(t)$ directly with the dynamic estimate $\xi(t)$, providing a distinct advantage over full-order observers where all states are filtered estimates. The reconstruction equation is given by:

$\hat{x} = \begin{bmatrix} C \\ T \end{bmatrix}^{-1} \begin{bmatrix} y \\ \xi \end{bmatrix} = M \begin{bmatrix} y \\ \xi \end{bmatrix}$

The final control law is then implemented as $u = -K\hat{x}$, utilizing the LQR gain $K$ derived in the previous task.

> 成功估计 $\xi(t)$ 后，通过对变换矩阵求逆来重构全状态估计 $\hat{x}(t)$。重构方程如Eq.()所示。最终的控制律为 $u = -K\hat{x}$



The design was implemented in MATLAB using the specific parameters derived from the matriculation number ($a=8, b=4, c=0, d=1$). Based on Task 2 results, the dominant closed-loop pole of the LQR controlled system was identified at $\lambda_{dom} \approx -2.2368$. To comprehensively investigate the impact of observer dynamics on system performance, three distinct design cases were established for the observer matrix $D = \text{diag}(\lambda_{obs1}, \lambda_{obs2})$ based on multipliers of $\lambda_{dom}$:

* **Case 1 (Slow Observer)**: The poles are placed at $1 \times \lambda_{dom}$. This represents a scenario where the observer dynamics are as sluggish as the controller dynamics.

* **Case 2 (Recommended Observer)**: The poles are placed at $4 \times \lambda_{dom}$. This aligns with the standard engineering rule of thumb (3 to 5 times faster) to ensure estimation convergence precedes control action.

* **Case 3 (Fast Observer)**: The poles are placed at $15 \times \lambda_{dom}$. This represents an aggressive design intended to achieve near-instantaneous estimation.

> 具体来说，设计是在 MATLAB 中实现的，使用了从学号导出的特定参数 ($a=8, b=4, c=0, d=1$)。基于任务 2 的结果，LQR 控制系统的主导闭环极点被确定为 $\lambda_{dom} \approx -2.2368$。为了全面研究观测器动态对系统性能的影响，基于 $\lambda_{dom}$ 的倍数，为观测器矩阵 $D$ 建立了三种不同的设计方案：Case 1（慢速观测器）：极点位于 1 倍主导极点处，代表观测器动态与控制器动态一样迟缓的情况。Case 2（推荐观测器）：极点位于 4 倍主导极点处，符合标准工程经验法则（快 3 到 5 倍），以确保估计收敛先于控制动作。Case 3（快速观测器）：极点位于 15 倍主导极点处，代表旨在实现近乎瞬时估计的激进设计。

For each case, the transformation matrix $T$ was obtained by solving the Sylvester equation $A^TT^T - T^TD^T = C^TG^T$ using MATLAB's `sylvester` function, with matrix $G$ initially set to ones. The invertibility of the reconstruction matrix was verified for each iteration. The simulation results are presented below to illustrate the performance differences.

> 对于每种情况，通过使用 MATLAB 的 `sylvester` 函数求解 Sylvester 方程 $A^TT^T - T^TD^T = C^TG^T$ 来获得变换矩阵 $T$，矩阵 $G$ 最初设为全 1 矩阵。每次迭代都验证了重构矩阵的可逆性。仿真结果如下所示，以说明性能差异。

Figure 1: State Estimation Error Norm Analysis

The figure below illustrates the norm of the state estimation error $\|x(t) - \hat{x}(t)\|$ over time. The inset plot provides a magnified view of the initial transient response (0 to 0.6s), clearly showing the "peaking phenomenon" in the fast observer case.

> **图 1：状态估计误差范数分析**。下图展示了状态估计误差范数 $\|x(t) - \hat{x}(t)\|$ 随时间的变化。插图提供了初始瞬态响应（0 到 0.6秒）的放大视图，清晰地显示了快速观测器情况下的“峰值现象”。<img title="" src="file:///E:/桌面文件存放/学习资料/NUS/NUS/NUS%20Courses/ME5401%20Linear%20System/Linear-System-Mini-Project/task3/task3_state%20estimation%20error%20norm.png" alt="task3_state estimation error norm" style="zoom:33%;" data-align="center">

Figure 2: State Tracking Performance

This figure decomposes the tracking performance for a representative state variable ($x_2$) across the three cases. The solid black lines represent the actual state trajectories, while the dashed colored lines represent the estimates.

> **图 2：状态跟踪性能**。该图分解了三种情况下代表性状态变量 ($x_2$) 的跟踪性能。实心黑线代表实际状态轨迹，而虚线彩色线代表估计值。<img title="" src="file:///E:/桌面文件存放/学习资料/NUS/NUS/NUS%20Courses/ME5401%20Linear%20System/Linear-System-Mini-Project/task3/task3_state%20tracking%20performance%20conparison.png" alt="task3_state tracking performance conparison" data-align="center" style="zoom:33%;">

Figure 3: Control Effort and Peaking Phenomenon

This figure plots the control input $u_1(t)$, specifically focusing on the initial phase to highlight the impact of estimation errors on the control signal magnitude.

> **图 3：控制量与峰值现象**。该图绘制了控制输入 $u_1(t)$，特别关注初始阶段，以突显估计误差对控制信号幅度的影响。<img title="" src="file:///E:/桌面文件存放/学习资料/NUS/NUS/NUS%20Courses/ME5401%20Linear%20System/Linear-System-Mini-Project/task3/task3_control%20input.png" alt="task3_control input" data-align="center" style="zoom:33%;">

3. Discussion and Conclusion

----------------------------

The comparative analysis of the simulation results reveals critical insights into the trade-offs required in observer design.

The **Slow Observer (Case 1, Red Line)** demonstrates the consequences of insufficient estimation bandwidth. As seen in Figure 1, the estimation error persists for nearly 2 seconds, which is significant relative to the system's time constants. Figure 2 visually confirms this lag, where the estimated state (red dashed line) fails to track the rapid changes of the true state initially. This estimation lag effectively introduces a delay in the feedback loop, which degrades the transient response of the closed-loop system, although the control effort (Figure 3) remains smooth.

> 对仿真结果的比较分析揭示了观测器设计中所需的权衡的关键见解。**慢速观测器（Case 1，红线）** 展示了估计带宽不足的后果。如图 1 所示，估计误差持续了近 2 秒，这相对于系统的时间常数来说是显著的。图 2 直观地证实了这种滞后，估计状态（红色虚线）最初无法跟踪真实状态的快速变化。这种估计滞后有效地在反馈回路中引入了延迟，从而降低了闭环系统的瞬态响应，尽管控制量（图 3）保持平滑。

Conversely, the **Fast Observer (Case 3, Blue Line)** highlights the dangers of overly aggressive pole placement. While Figure 1 shows that the error converges to zero extremely quickly (within 0.2 seconds), it exhibits a massive initial spike, reaching a magnitude of nearly 9. This is a classic manifestation of the **Peaking Phenomenon** inherent in high-gain observers. The detrimental effect of this peaking is evident in Figure 3, where the control signal $u_1$ experiences a severe, sharp fluctuation at $t=0$. In a physical diesel engine, such a spike would likely saturate the VGT or EGR actuators and could potentially cause mechanical damage. Furthermore, theoretically, such a high-gain observer would dangerously amplify any sensor noise, rendering the control system unstable in practice.

> 相反，**快速观测器（Case 3，蓝线）** 突显了过度激进的极点配置的危险。虽然图 1 显示误差极快地收敛到零（在 0.2 秒内），但它表现出一个巨大的初始尖峰，幅度接近 9。这是高增益观测器固有的 **峰值现象 (Peaking Phenomenon)** 的典型表现。这种峰值的有害影响在图 3 中显而易见，控制信号 $u_1$ 在 $t=0$ 处经历了剧烈、尖锐的波动。在物理柴油发动机中，这种尖峰可能会使 VGT 或 EGR 执行器饱和，并可能导致机械损坏。此外，理论上，如此高增益的观测器会危险地放大任何传感器噪声，使控制系统在实践中不稳定。

The **Recommended Observer (Case 2, Green Line)** validates the engineering rule of thumb. By placing the poles at 4 times the dominant LQR poles, the design achieves an optimal compromise. The error convergence is sufficiently fast (approx. 0.5s) to provide accurate feedback for control, as shown by the tight tracking in Figure 2. Crucially, it avoids the severe peaking seen in the fast case, resulting in a control signal (Figure 3) that is responsive yet physically realizable.

> **推荐观测器（Case 2，绿线）** 验证了工程经验法则。通过将极点放置在 LQR 主导极点的 4 倍处，该设计实现了最佳折衷。误差收敛足够快（约 0.5 秒），能够为控制提供准确的反馈，如图 2 中的紧密跟踪所示。至关重要的是，它避免了快速情况下的严重峰值，产生了一个既响应迅速又在物理上可实现的控制信号（图 3）。

In conclusion, this task successfully demonstrated the implementation of a reduced-order observer, effectively reducing the observer dynamics from order 4 to 2 by leveraging known outputs. The investigation confirms that observer poles should be placed approximately 3 to 5 times faster than the closed-loop system dynamics to balance estimation speed against the risks of peaking phenomena and noise amplification.

> 总之，本任务成功演示了降维观测器的实现，通过利用已知输出有效地将观测器动态从 4 阶降低到
