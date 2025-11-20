# Task 4: Decoupling Controller Design

## 1. Introduction

The primary objective of this task is to design a multivariable controller that decouples the interaction between the inputs and outputs of the diesel engine air path system. In a coupled Multiple-Input Multiple-Output (MIMO) system, a change in a single input variable typically affects multiple output variables, which complicates the control strategy. [cite_start]Decoupling control aims to transform this interactive system into a set of independent Single-Input Single-Output (SISO) subsystems, where each reference input controls only its corresponding output[cite: 1029]. Specifically for this project, the goal is to design a state feedback control law $u = -Kx + Fr$ such that the closed-loop transfer function matrix is diagonal. A critical constraint for this design is that the resulting closed-loop system must be stable. Furthermore, the internal stability of the decoupled system must be verified by analyzing the system's response to non-zero initial conditions with zero external input.

> [cite_start]本任务的主要目标是设计一个多变量控制器，以解耦柴油发动机空气路径系统的输入和输出之间的相互作用。在耦合的多输入多输出（MIMO）系统中，单个输入变量的变化通常会影响多个输出变量，这使得控制策略变得复杂。解耦控制旨在将这种交互式系统转化为一组独立的单输入单输出（SISO）子系统，其中每个参考输入仅控制其对应的输出 [cite: 1029]。具体到本项目，目标是设计状态反馈控制律 $u = -Kx + Fr$，使得闭环传递函数矩阵为对角阵。该设计的一个关键约束是所得闭环系统必须是稳定的。此外，必须通过分析系统在零外部输入下对非零初始条件的响应来验证解耦系统的内部稳定性。

## 2. Task Solvement

The design process follows the state feedback decoupling theory outlined in Linear Systems theory. The first step is to determine the relative degree, denoted as $\sigma_i$, for each output channel $y_i$. [cite_start]The relative degree represents the difference between the degree of the denominator and the numerator of the transfer function for that row, or in state-space terms, the smallest integer $\sigma_i$ such that $c_i^T A^{\sigma_i-1} B \neq 0$[cite: 1330]. Based on the provided system matrices $A$, $B$, and $C$, the relative degrees were calculated using MATLAB. The calculation revealed that for both outputs, the product $c_i B$ was non-zero, resulting in $\sigma_1 = 1$ and $\sigma_2 = 1$. [cite_start]Consequently, the decoupling matrix $B^*$ is constructed as follows[cite: 1453]:

$$
B^* = \begin{bmatrix} c_1^T A^{\sigma_1-1} B \\ c_2^T A^{\sigma_2-1} B \end{bmatrix} = \begin{bmatrix} c_1^T B \\ c_2^T B \end{bmatrix}
$$

> [cite_start]设计过程遵循线性系统理论中概述的状态反馈解耦理论。第一步是确定每个输出通道 $y_i$ 的相对阶，记为 $\sigma_i$。相对阶表示该行传递函数的分母与分子阶数之差，或者用状态空间术语来说，是使得 $c_i^T A^{\sigma_i-1} B \neq 0$ 的最小整数 $\sigma_i$ [cite: 1330][cite_start]。基于提供的系统矩阵 $A$、$B$ 和 $C$，使用 MATLAB 计算了相对阶。计算显示对于两个输出，乘积 $c_i B$ 均非零，因此 $\sigma_1 = 1$ 且 $\sigma_2 = 1$。随之，解耦矩阵 $B^*$ 构造如下 [cite: 1453]：
> $
> B^* = \begin{bmatrix} c_1^T A^{\sigma_1-1} B \\ c_2^T A^{\sigma_2-1} B \end{bmatrix} = \begin{bmatrix} c_1^T B \\ c_2^T B \end{bmatrix}
> $

[cite_start]Upon computation, the determinant of $B^*$ was found to be non-zero ($\det(B^*) \approx -0.037$), satisfying the necessary and sufficient condition for the existence of a decoupling control law[cite: 1454]. Since the project requires closed-loop stability, the simple integrator decoupling method (Theorem 1) is insufficient as it places poles at the origin. [cite_start]Instead, the "Decoupling with Pole Placement" method (Theorem 2) is employed[cite: 1546]. This method allows the specification of stable closed-loop poles for the decoupled subsystems. We define stable polynomials $\phi_{f_i}(s)$ for each channel. To meet the settling time requirement ($t_s < 20s$), the poles were chosen at $\lambda_1 = -2$ and $\lambda_2 = -3$, leading to $\phi_{f_1}(s) = s + 2$ and $\phi_{f_2}(s) = s + 3$.

> [cite_start]经计算，$B^*$ 的行列式非零（$\det(B^*) \approx -0.037$），满足存在解耦控制律的充要条件 [cite: 1454][cite_start]。由于项目要求闭环稳定性，简单的积分器解耦方法（定理 1）是不够的，因为它将极点配置在原点。因此，采用“带极点配置的解耦”方法（定理 2）[cite: 1546]。该方法允许为解耦子系统指定稳定的闭环极点。我们为每个通道定义了稳定多项式 $\phi_{f_i}(s)$。为了满足调节时间要求（$t_s < 20s$），极点选择为 $\lambda_1 = -2$ 和 $\lambda_2 = -3$，即 $\phi_{f_1}(s) = s + 2$ 和 $\phi_{f_2}(s) = s + 3$。

[cite_start]The feedforward gain matrix $F$ and the state feedback gain matrix $K$ are calculated using the following equations derived from Theorem 2[cite: 1549]:

$$
F = (B^*)^{-1}, \quad K = (B^*)^{-1} \begin{bmatrix} c_1^T \phi_{f_1}(A) \\ c_2^T \phi_{f_2}(A) \end{bmatrix}
$$

The intermediate matrix calculations yielded the following controller gains:
$
F = \begin{bmatrix} -0.7328 & 0.2161 \\ -0.2333 & 36.9157 \end{bmatrix}, \quad K = \begin{bmatrix} -20.2961 & 4.7472 & -17.0316 & 6.9603 \\ 306.1532 & -475.5242 & 300.8394 & 150.1761 \end{bmatrix}
$

> [cite_start]前馈增益矩阵 $F$ 和状态反馈增益矩阵 $K$ 使用定理 2 导出的以下公式计算 [cite: 1549]：
> $
> F = (B^*)^{-1}, \quad K = (B^*)^{-1} \begin{bmatrix} c_1^T \phi_{f_1}(A) \\ c_2^T \phi_{f_2}(A) \end{bmatrix}
> $
> 中间矩阵计算得出以下控制器增益：
> $
> F = \begin{bmatrix} -0.7328 & 0.2161 \\ -0.2333 & 36.9157 \end{bmatrix}, \quad K = \begin{bmatrix} -20.2961 & 4.7472 & -17.0316 & 6.9603 \\ 306.1532 & -475.5242 & 300.8394 & 150.1761 \end{bmatrix}
> $

Simulation was conducted to verify the design. The step response of the closed-loop system demonstrates perfect decoupling, where the diagonal channels exhibit the desired first-order response characteristics, and the off-diagonal channels remain at zero initially. However, a critical observation was made in the zero-input response simulation with the initial condition $x_0 = [0.5, -0.1, 0.3, -0.8]^T$. The state trajectories did not converge to the origin; instead, they exhibited significant divergence after approximately 2 seconds. This prompted an eigenvalue analysis of the closed-loop matrix $A_{cl} = A - BK$. The eigenvalues were found to be $\{-2.00, -3.00, -459.62, 14.56\}$.

> 进行了仿真以验证设计。闭环系统的阶跃响应展示了完美的解耦，其中对角线通道表现出预期的特定一阶响应特性，而非对角线通道最初保持为零。<img title="" src="file:///E:/桌面文件存放/学习资料/NUS/NUS/NUS%20Courses/ME5401%20Linear%20System/Linear-System-Mini-Project/task4/task4_decoupling%20step%20response.png" alt="task4_decoupling step response" style="zoom:33%;">
> 然而，在初始条件为 $x_0 = [0.5, -0.1, 0.3, -0.8]^T$ 的零输入响应仿真中观察到了一个关键现象。状态轨迹并没有收敛到原点；相反，它们在大约 2 秒后表现出显著的发散。这促使我们对闭环矩阵 $A_{cl} = A - BK$ 进行特征值分析。发现特征值为 $\{-2.00, -3.00, -459.62, 14.56\}$。<img src="file:///E:/桌面文件存放/学习资料/NUS/NUS/NUS%20Courses/ME5401%20Linear%20System/Linear-System-Mini-Project/task4/task4_intrnal%20stability.png" title="" alt="task4_intrnal stability" style="zoom:33%;">

## 3. Discussion and Conclusion

The results present a dichotomy: the input-output transfer function matrix is diagonal and theoretically stable (with poles at -2 and -3), yet the state-space realization is internally unstable. The presence of the eigenvalue $\lambda = 14.56$ in the right-half plane confirms the instability. This phenomenon is a classic example of unstable pole-zero cancellation occurring during the decoupling process. While the feedback law successfully cancels the dynamics coupling the inputs to the "wrong" outputs, it renders an unstable mode unobservable from the output. Consequently, while the output $y(t)$ may appear well-behaved for a short duration, the internal state variables $x(t)$ grow unbounded, eventually causing numerical overflow in simulation or physical failure in a real system.

> 结果呈现出一种二分性：输入-输出传递函数矩阵是对角阵且理论上稳定的（极点在 -2 和 -3），但状态空间实现却是内部不稳定的。右半平面特征值 $\lambda = 14.56$ 的存在证实了这种不稳定性。这种现象是解耦过程中发生不稳定零极点对消的典型例子。虽然反馈律成功抵消了将输入耦合到“错误”输出的动力学特性，但它使得一个不稳定模态对输出变得不可观测。因此，虽然输出 $y(t)$ 在短时间内可能表现良好，但内部状态变量 $x(t)$ 会无限增长，最终导致仿真中的数值溢出或真实系统中的物理故障。

In conclusion, the decoupling controller design for Task 4 was mathematically successful in diagonalizing the system's transfer function matrix, satisfying the structural decoupling requirement. However, the answer to the question "Is the decoupled system internally stable?" is a definitive **NO**. The decoupling process induced an unstable hidden mode. This highlights a fundamental limitation of state feedback decoupling: it cannot stabilize fixed modes that coincide with the transmission zeros of the plant if those zeros are unstable. For this specific diesel engine system, a pure decoupling strategy is dangerous due to the loss of internal stability.

> 总之，Task 4 的解耦控制器设计在数学上成功地将系统的传递函数矩阵对角化，满足了结构解耦的要求。然而，对于“解耦后的系统是否内部稳定？”这一问题的回答是明确的**“否”**。解耦过程诱发了一个不稳定的隐藏模态。这凸显了状态反馈解耦的一个基本局限性：如果受控对象的传输零点是不稳定的，且解耦过程导致固定模态与这些零点重合，则无法通过解耦来稳定系统。对于这个特定的柴油发动机系统，由于内部稳定性的丧失，纯解耦策略是危险的。
