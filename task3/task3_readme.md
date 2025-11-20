Task 3 Report: Reduced-Order Observer Design for Diesel Engine Control
======================================================================

## 1. Introduction

In the context of the multivariable control design for the diesel engine air system, the preceding tasks operated under the assumption that the full state vector is accessible for feedback. However, in practical engineering scenarios, measuring all state variables is often unfeasible due to the high cost of sensors or the physical inaccessibility of internal states. Furthermore, as the state variables in this project are derived from system identification, they lack explicit physical meanings, making direct measurement impossible. The project specification for Task 3 addresses this reality by restricting available information to the two system outputs and the control inputs. Consequently, a state observer is required to reconstruct the full state vector to implement the Linear Quadratic Regulator (LQR) designed in Task 2.

> 之前的任务1-2是在假设全状态向量可用于反馈的情况下进行的。然而，在实际工程场景中，由于传感器成本高昂或内部状态物理上不可接近，测量所有状态变量往往是不可行的。此外，由于本项目中的状态变量源于系统辨识，它们缺乏明确的物理意义，使得直接测量变得不太可能。任务 3 通过将可用信息限制为两个系统输出和控制输入来模拟这一现实问题。因此，需要一个状态观测器来重构全状态向量，以实现任务 2 中设计的线性二次调节器 (LQR)。

## 2. Tasks Solvement

### 2.1 Theoretical Framework of Reduced-Order Observer

Although a full-order observer can be used to simulate the entire system dynamics for estimating all four state variables, its design is redundant. Because the two measurement outputs already provide precise information about linear combinations of the states. Given that the system has four states (n=4) and two independent outputs (m=2), it is theoretically sufficient to estimate only the remaining n-m=2 states, which is computationally more efficient. Therefore, this project focuses on the design and implementation of a dimension-reduction observer. The primary objective is to synthesise a dimension-reduction observer that guarantees asymptotic convergence of estimation error, whilst investigating the trade-off between convergence rate and control signal integrity through analysis of different observer pole locations.

> 虽然全阶观测器可以用来模拟整个系统的动态来估计所有四个状态变量，但它的设计是冗余的。因为两个测量输出已经提供了关于状态线性组合的精确信息。鉴于该系统有四个状态 ($n=4$) 和两个独立输出 ($m=2$)，理论上只需要估计剩余的 $n-m=2$ 个状态就足够了，且计算效率更高。因此，本报告重点关注降维观测器的设计与实现。主要目标是综合一个能够确保估计误差渐近收敛的降维观测器，并通过分析不同的观测器极点位置，研究收敛速度与控制信号完整性之间的权衡。

~~The design methodology for the reduced-order observer strictly follows the theoretical framework presented in Chapter 11 of the lecture notes. The core concept involves transforming the original state vector $x \in \mathbb{R}^n$ into a new coordinate system that consists of the measurable outputs $y \in \mathbb{R}^m$ and a reduced state vector $\xi \in \mathbb{R}^{n-m}$. This linear transformation is defined by a nonsingular matrix composed of the output matrix $C$ and a transformation matrix $T$ to be designed:~~

~~$\begin{bmatrix} y \\ \xi \end{bmatrix} = \begin{bmatrix} C \\ T \end{bmatrix} x$~~

~~where $\xi$ represents the internal states of the observer. The dynamics of the reduced-order observer are governed by the following differential equation:~~

~~$\dot{\xi} = D\xi + Eu + Gy$~~

~~where $D$, $E$, and $G$ are the observer matrices of appropriate dimensions.~~

> ~~降维观测器的设计需要将原始状态向量 $x \in \mathbb{R}^n$ 变换到一个新的坐标系，该坐标系由可测输出 $y \in \mathbb{R}^m$ 和降维状态向量 $\xi \in \mathbb{R}^{n-m}$ 组成。该线性变换由一个包含输出矩阵 $C$ 和待设计变换矩阵 $T$ 的非奇异矩阵，其中 $\xi$ 代表观测器的内部状态。降维观测器的动态由以下微分方程描述，其中 $D$、$E$ 和 $G$ 是相应维度的观测器矩阵。~~

~~To ensure that the estimated state $\xi$ correctly tracks the transformation $Tx$ and that the estimation error $e(t) = \xi(t) - Tx(t)$ decays to zero asymptotically, the observer matrices must satisfy the specific algebraic constraints derived from the error dynamics equation $\dot{e} = De + (DT - TA + GC)x + (E - TB)u$. These constraints are:~~

1. ~~Matrix $D$ must be stable, meaning all its eigenvalues must have negative real parts.~~

2. ~~The matrix $T$ must satisfy the Sylvester equation: $TA - DT = GC$.~~

3. ~~The input matrix $E$ must be matched such that $E = TB$.~~

> ~~为了确保估计状态 $\xi$ 正确跟踪变换 $Tx$，并且估计误差 $e(t) = \xi(t) - Tx(t)$ 渐近衰减为零，观测器矩阵必须满足由误差动态方程导出的特定代数约束。这些约束包括：~~
> 
> 1. ~~矩阵 $D$ 必须是稳定的，即其所有特征值的实部必须为负。~~
> 2. ~~矩阵 $T$ 必须满足 Sylvester 方程：$TA - DT = GC$。~~
> 3. ~~输入矩阵 $E$ 必须匹配，使得 $E = TB$。~~

~~Upon successfully estimating $\xi(t)$, the full state estimate $\hat{x}(t)$ is reconstructed by inverting the transformation matrix. This reconstruction combines the noise-free measurement $y(t)$ directly with the dynamic estimate $\xi(t)$, providing a distinct advantage over full-order observers where all states are filtered estimates. The reconstruction equation is given by:~~

~~$\hat{x} = \begin{bmatrix} C \\ T \end{bmatrix}^{-1} \begin{bmatrix} y \\ \xi \end{bmatrix} = M \begin{bmatrix} y \\ \xi \end{bmatrix}$~~

~~The final control law is then implemented as $u = -K\hat{x}$, utilizing the LQR gain $K$ derived in the previous task.~~

> ~~成功估计 $\xi(t)$ 后，通过对变换矩阵求逆来重构全状态估计 $\hat{x}(t)$。重构方程如Eq.()所示。最终的控制律为 $u ~~= -K\hat{x}$



Specifically, based on the results from Task 2, the dominant closed-loop pole of the LQR control system was determined to be $\lambda_{dom} \approx -2.2368$. In order to comprehensively investigate the impact of observer dynamics on system performance, three distinct design cases for the observer matrix $D$ were established based on multiples of $\lambda_{dom}$:

Case 1 (Slow Observer): The pole is positioned at 1 times the dominant pole, representing a scenario where the observer dynamics are as slow as the controller dynamics.

Case 2 (Recommended Observer): The pole is positioned at 4 times the dominant pole, following standard engineering experience rules (3 to 5 times faster) to ensure estimation convergence precedes control actions.

Case 3 (Fast Observer): The pole is positioned at 15 times the dominant pole, representing an aggressive design aimed at achieving near-instantaneous estimation.

> 具体来说，基于任务 2 的结果，LQR 控制系统的主导闭环极点被确定为 $\lambda_{dom} \approx -2.2368$。为了全面研究观测器动态对系统性能的影响，基于 $\lambda_{dom}$ 的倍数，为观测器矩阵 $D$ 建立了三种不同的设计方案：
> 
> Case 1（慢速观测器）：极点位于 1 倍主导极点处，代表观测器动态与控制器动态一样迟缓的情况。
> 
> Case 2（推荐观测器）：极点位于 4 倍主导极点处，符合标准工程经验法则（快 3 到 5 倍），以确保估计收敛先于控制动作。
> 
> Case 3（快速观测器）：极点位于 15 倍主导极点处，代表旨在实现近乎瞬时估计的激进设计。

For each case, the Sylvester equation $TA - DT = GC$ is solved using MATLAB's `sylvester` function to obtain the transformation matrix $T$, with matrix $G$ initially set as a full-ones matrix. The invertibility of the reconstructed matrix is verified at each iteration. The simulation results are shown in Figures 2-8, 2-9, and 2-10.

> 对于每种情况，使用 MATLAB 的 `sylvester` 函数求解 Sylvester 方程 $TA - DT = GC$ 来获得变换矩阵 $T$，矩阵 $G$ 最初设为全 1 矩阵。每次迭代都验证了重构矩阵的可逆性。仿真结果如图2-8,2-9,2-10所示。

Figure 1: State Estimation Error Norm Analysis

Figure 2: State Tracking Performance

Figure 3: Control Effort and Peaking Phenomenon

Figure 2-8 depicts the temporal evolution of the estimation error norm $\|x(t) - \hat{x}(t)\|$. It can be observed that the **slow observer (red line)** exhibits a gradual decay rate, requiring approximately 2 seconds to stabilise. In contrast, the fast observer (blue line) converges within 0.2 seconds but exhibits a large initial spike with an amplitude approaching 9. The recommended observer (green line) displays a balanced convergence curve without excessive initial overshoot.

> 图 2-8 是估计误差范数 $\|x(t) - \hat{x}(t)\|$ 随时间的变化，可以观察到，**慢速观测器（红线）** 表现出缓慢的衰减率，大约需要 2 秒才能稳定。相比之下，**快速观测器（蓝线）** 在 0.2 秒内收敛，但表现出较大的初始尖峰，幅度接近 9。**推荐观测器（绿线）** 显示了平衡的收敛曲线，没有过度的初始超调。
> 
> <img title="" src="file:///E:/桌面文件存放/学习资料/NUS/NUS/NUS%20Courses/ME5401%20Linear%20System/Linear-System-Mini-Project/task3/task3_state%20estimation%20error%20norm.png" alt="task3_state estimation error norm" style="zoom:33%;" data-align="center">

Figure 2-9 tracks the performance of representative state variables ($x_2$) under three cases. The solid black line represents the actual state trajectory, while the dashed coloured lines denote the estimated values. Results indicate that for the **slow observer**, the estimated state (red dashed line) slightly behind the actual state during the transient phase. The **recommended observer** and **fast observer** provide tighter tracking, with the fast observer's estimates becoming virtually indistinguishable from the true state after the initial moment.

> ** 图 2-9 追踪了三种情况下代表性状态变量 ($x_2$) 的跟踪性能，实心黑线代表实际状态轨迹，而虚线彩色线代表估计值。结果表明，对于**慢速观测器**，估计状态（红色虚线）在瞬态阶段略微滞后于实际状态。**推荐观测器**和**快速观测器**提供了更紧密的跟踪，快速观测器的估计值在初始时刻后几乎与真实状态无法区分。<img title="" src="file:///E:/桌面文件存放/学习资料/NUS/NUS/NUS%20Courses/ME5401%20Linear%20System/Linear-System-Mini-Project/task3/task3_state%20tracking%20performance%20conparison.png" alt="task3_state tracking performance conparison" data-align="center" style="zoom:33%;">



Figure 2-10 plots the control input $u_1(t)$, with particular attention to the initial phase, to observe the effect of estimation error on the amplitude of the control signal. A peak phenomenon is observed in the fast observer (blue line), where the control signal undergoes a sharp, abrupt fluctuation at $t=0$. In contrast, the slow observer exhibits a smoother control signal but corrects most slowly. The recommended observer maintains a reasonable control amplitude, closely resembling that of ideal state feedback.

> **图 2-10 绘制了控制输入 $u_1(t)$，特别关注初始阶段，来观察估计误差对控制信号幅度的影响。观察到快速观测器（蓝线）** 的**峰值现象**，其中控制信号在 $t=0$ 处经历了剧烈、尖锐的波动，而**慢速观测器**的控制信号更为平滑的，但校正最慢，相比之下**推荐观测器**保持了与理想状态反馈相似且合理的控制幅度。<img title="" src="file:///E:/桌面文件存放/学习资料/NUS/NUS/NUS%20Courses/ME5401%20Linear%20System/Linear-System-Mini-Project/task3/task3_control%20input.png" alt="task3_control input" data-align="center" style="zoom:33%;">

Overall, comparative analysis of simulation results reveals fundamental trade-offs inherent in observer design, particularly between estimation speed and the occurrence of “spiking phenomena”. **Slow Observer (Case 1)** demonstrates the consequences of insufficient estimation bandwidth. Estimation lag introduces delay into the feedback loop. Whilst the control output remains smooth, this delay degrades the closed-loop transient response, rendering the system sluggish in correcting initial errors. Therefore, performance suffers if the observer dynamics are not sufficiently faster than the object dynamics.

Conversely, **Fast Observer (Case 3)** demonstrates the practical dangers of high-gain observers. Whilst the error converges extremely rapidly, the high gain amplifies the initial estimation error, leading to large spikes in the state estimate (Figure 2-8). This spike propagates to the controller, causing severe peaks in the control signal $u_1$ observed in Figure 2-10. In physical diesel engine systems, such pulsed control signals could saturate actuators (e.g., VGT or EGR valves) or induce mechanical stress. Furthermore, theoretically, such high gains would drastically amplify sensor noise, rendering the control system unstable in noisy environments.

**Recommended Observer (Case 2)** By placing poles at four times the LQR dominant pole, error convergence is sufficiently rapid to ensure accurate controller tracking without inducing hazardous spikes. Control signals remain within physically achievable bounds, with closed-loop performance closely approximating that of full state feedback.

> 总的来说，仿真结果的比较分析显示了观测器设计中所需的基本权衡，特别是估计速度与“峰值现象”之间的权衡。**慢速观测器 (Case 1)** 展示了估计带宽不足的后果。估计滞后在反馈回路中引入了延迟。虽然控制量是平滑的，但这种延迟降低了闭环瞬态响应，使得系统在纠正初始误差方面变得迟缓。因此，如果观测器动态不比对象动态足够快，性能就会受到影响。
> 
> 相反，**快速观测器 (Case 3)** 展现了高增益观测器的实际危险。虽然误差极快地收敛，但高增益放大了初始估计误差，导致状态估计出现较大的尖峰（图 1）。这个尖峰传播到控制器，导致在图 3 中观察到的控制信号 $u_1$ 的严重峰值。在物理柴油发动机系统中，这种脉冲控制信号可能会使执行器（如 VGT 或 EGR 阀）饱和或造成机械应力。此外，理论上，如此高的增益会急剧放大传感器噪声，使控制系统在噪声环境中不稳定。
> 
> **推荐观测器 (Case 2)** 通过将极点放置在 LQR 主导极点的 4 倍处，误差收敛足够快，确保了控制器的准确跟踪，并且也没引起危险的峰值。控制信号保持在物理上可实现的范围内，闭环性能非常接近全状态反馈的性能。

3. Discussion and Conclusion

-Task 3 successfully implemented a dimension-reducing observer, effectively utilising system output to reduce the estimated order from fourth-order to second-order. Research confirmed that observer poles should be positioned approximately 3~5 times faster than the closed-loop system dynamics. This configuration balances the requirement for rapid estimation with the risks of peak phenomena and noise sensitivity, providing a robust solution for diesel engine control problems.

> 任务3成功实现了降维观测器，有效地利用系统输出将估计阶数从 4 阶降低到 2 阶。研究证实，观测器极点应放置在比闭环系统动态快约 3 到 5 倍的位置。这种配置平衡了快速估计的需求与峰值现象和噪声敏感性的风险，为柴油发动机控制问题提供了一个鲁棒的解决方案。
