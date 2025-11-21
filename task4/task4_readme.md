# Task 4: Decoupling Controller Design

## 1. Introduction

The primary objective of Task 4 is to design a multivariable controller to decouple the inputs and outputs of a diesel engine system. Within coupled MIMO systems, variations in a single input variable typically affect multiple output variables, complicating control strategies. Thus, decoupling control aims to transform such interactive systems into a set of independent SISO subsystems, where each reference input controls only its corresponding output. Specifically, this task requires designing a state feedback control law $u = -Kx + Fr$ such that the closed-loop transfer function matrix is diagonalised and the closed-loop system remains stable. Furthermore, the internal stability of the decoupled system must be verified by analysing its response to non-zero initial conditions under zero external inputs.

> 任务4的主要目标是设计一个多变量控制器，以解耦柴油发动机系统的输入和输出。在耦合的 MIMO系统中，单个输入变量的变化通常会影响多个输出变量，这使得控制策略变得复杂。解耦控制旨在将这种交互式系统转化为一组独立的 SISO 子系统，其中每个参考输入仅控制其对应的输出。具体来说，本任务目标是设计状态反馈控制律 $u = -Kx + Fr$，使得闭环传递函数矩阵为对角阵，并且保证闭环系统是稳定的。此外，还需要分析系统在零外部输入下对非零初始条件的响应来验证解耦系统的内部稳定性。

## 2. Task Solvement

The first step is to determine the relative order for each output channel $y_i$, denoted as $\sigma_i$. The relative order represents the difference between the denominator and numerator orders of the transfer function for that row, and is also the smallest integer $\sigma_i$ such that $c_i^T A^{\sigma_i-1} B \neq 0$. Next, based on the system matrices $A$, $B$, and $C$ computed in the previous tasks, the relative orders are calculated. The computation reveals that for both outputs, the product $c_i B$ is non-zero. Therefore, $\sigma_1 = 1$ and $\sigma_2 = 1$. Consequently, the decoupling matrix $B^*$ is constructed as shown in Equation ().

$$
B^* = \begin{bmatrix} c_1^T A^{\sigma_1-1} B \\ c_2^T A^{\sigma_2-1} B \end{bmatrix} = \begin{bmatrix} c_1^T B \\ c_2^T B \end{bmatrix}
$$

> 第一步是确定每个输出通道 $y_i$ 的相对阶，记为 $\sigma_i$。相对阶表示该行传递函数的分母与分子阶数之差，也是使得 $c_i^T A^{\sigma_i-1} B \neq 0$ 的最小整数 $\sigma_i$ 。接着基于前几问计算的系统矩阵 $A$、$B$ 和 $C$，来计算出相对阶。计算显示对于两个输出，乘积 $c_i B$ 均非零，因此 $\sigma_1 = 1$ 且 $\sigma_2 = 1$。随之，解耦矩阵 $B^*$ 构造如式（）所示：
> $
> B^* = \begin{bmatrix} c_1^T A^{\sigma_1-1} B \\ c_2^T A^{\sigma_2-1} B \end{bmatrix} = \begin{bmatrix} c_1^T B \\ c_2^T B \end{bmatrix}
> $

Calculations indicate that the determinant of $B^*$ is non-zero (approximately -0.037), satisfying the conditions for the existence of decoupling control laws. As the project requires closed-loop stability, simple integrator decoupling methods are not enough. Therefore, the ‘decoupling with pole placement’ method is adopted, which permits the specification of stable closed-loop poles for the decoupled subsystem. The project defined a stabilising polynomial $\phi_{f_i}(s)$ for each channel to meet the time-constraint requirement ($t_s < 20s$), the poles were selected as $\lambda_1 = -2$ and $\lambda_2 = -3$, yielding $\phi_{f_1}(s) = s + 2$ and $\phi_{f_2}(s) = s + 3$.

> 经计算，$B^*$ 的行列式非零（$\det(B^*) \approx -0.037$），满足解耦控制律存在的条件。由于项目要求闭环稳定性，简单的积分器解耦方法是不够的，因此，采用“带极点配置的解耦”方法，该方法允许为解耦子系统指定稳定的闭环极点。我们为每个通道定义了稳定多项式 $\phi_{f_i}(s)$。为了满足调节时间要求（$t_s < 20s$），极点选择为 $\lambda_1 = -2$ 和 $\lambda_2 = -3$，即 $\phi_{f_1}(s) = s + 2$ 和 $\phi_{f_2}(s) = s + 3$。

The feedforward gain matrix $F$ and the state feedback gain matrix $K$ are calculated using the following formula derived from Theorem 2:

$$
F = (B^*)^{-1}, \quad K = (B^*)^{-1} \begin{bmatrix} c_1^T \phi_{f_1}(A) \\ c_2^T \phi_{f_2}(A) \end{bmatrix}
$$

The intermediate matrix calculation yields the following controller gains:
$
F = \begin{bmatrix} -0.7328 & 0.2161 \\ -0.2333 & 36.9157 \end{bmatrix}, \quad K = \begin{bmatrix} -20.2961 & 4.7472 & -17.0316 & 6.9603 \\ 306.1532 & -475.5242 & 300.8394 & 150.1761 \end{bmatrix}
$

> 前馈增益矩阵 $F$ 和状态反馈增益矩阵 $K$ 使用定理 2 导出的以下公式计算：
> $
> F = (B^*)^{-1}, \quad K = (B^*)^{-1} \begin{bmatrix} c_1^T \phi_{f_1}(A) \\ c_2^T \phi_{f_2}(A) \end{bmatrix}
> $
> 中间矩阵计算得出以下控制器增益：
> $
> F = \begin{bmatrix} -0.7328 & 0.2161 \\ -0.2333 & 36.9157 \end{bmatrix}, \quad K = \begin{bmatrix} -20.2961 & 4.7472 & -17.0316 & 6.9603 \\ 306.1532 & -475.5242 & 300.8394 & 150.1761 \end{bmatrix}
> $

Subsequent simulations were conducted to validate the design. However, a critical phenomenon was observed during the zero-input-response simulation with initial conditions $x_0 = [0.5, -0.1, 0.3, -0.8]^T$. The state trajectories did not converge to the origin; instead, they exhibited significant divergence after approximately 2 seconds. This prompted us to conduct an eigenvalue analysis of the closed-loop matrix $A_{cl} = A - BK$. The eigenvalues were found to be $\{-2.00, -3.00, -459.62, 14.56\}$.

The results revealed a dichotomy: while the input-output transfer function matrix is diagonal and theoretically stable, the state-space implementation exhibits internal instability. The presence of the right-half-plane eigenvalue $\lambda = 14.56$ confirms this instability. This phenomenon arises from the cancellation of unstable zero-pole pairs during decoupling. Whilst the feedback law successfully counteracts the dynamics coupling the input to the ‘erroneous’ output, it renders an unstable mode unobservable in the output. Consequently, whilst the output $y(t)$ may exhibit favourable behaviour in the short term, the internal state variable $x(t)$ will grow without bound. This ultimately leads to numerical overflow in simulations or physical failure in real systems.

The step response of the closed-loop system demonstrates a good degree of decoupling, as shown in Figure 2-12. The diagonal channels exhibit the expected specific first-order response characteristics, whilst the non-diagonal channels initially remain at zero but show a slight rise after 2 seconds. This project attributes the cause to the finite numerical precision inherent in MATLAB simulations on computers. Consequently, both the inverse operation of the decoupling matrix $B^*$ and the calculation of controller parameters $K, F$ introduce minute **numerical errors**. Furthermore, the closed-loop system matrix $A_{cl} = A - BK$ possesses a **positive real eigenvalue** $\lambda \approx 14.56$. At $t=2$, $e^{29.12} \approx 4.4 \times 10^{12}$. The error at this point is approximately $10^{-15} \times 10^{12} = 10^{-3}$.

> 闭环系统的阶跃响应展示了较好的解耦，如图2-12所示。其中对角线通道表现出预期的特定一阶响应特性，而非对角线通道最初保持为零，但在2s后出现一定的抬升。本项目认为原因是MATLAB 在计算机中进行仿真时，数字的精度是有限的因此，解耦矩阵 $B^*$ 的逆运算和控制器参数 $K, F$ 的计算都存在极其微小的**数值误差**，并且闭环系统矩阵 $A_{cl} = A - BK$ 有一个**正实部特征值** $\lambda \approx 14.56$，当 $t=2$ 时，$e^{29.12} \approx 4.4 \times 10^{12}$。此时误差约为 $10^{-15} \times 10^{12} = 10^{-3}$。
> 
> <img title="" src="file:///E:/桌面文件存放/学习资料/NUS/NUS/NUS%20Courses/ME5401%20Linear%20System/Linear-System-Mini-Project/task4/task4_decoupling%20step%20response.png" alt="task4_decoupling step response" style="zoom:33%;">
> 随后进行仿真以验证设计。然而，在初始条件为 $x_0 = [0.5, -0.1, 0.3, -0.8]^T$ 的零输入响应仿真中观察到了一个关键现象。状态轨迹并没有收敛到原点；相反，它们在大约 2 秒后表现出显著的发散。这促使我们对闭环矩阵 $A_{cl} = A - BK$ 进行特征值分析。发现特征值为 $\{-2.00, -3.00, -459.62, 14.56\}$。结果呈现出一种二分性：输入-输出传递函数矩阵是对角阵且理论上稳定的，但状态空间实现却是内部不稳定的。右半平面特征值 $\lambda = 14.56$ 的存在证实了这种不稳定性。这种现象是解耦过程中发生不稳定零极点对消，虽然反馈律成功抵消了将输入耦合到“错误”输出的动力学特性，但它使得一个不稳定模态对输出变得不可观测。因此，虽然输出 $y(t)$ 在短时间内可能表现良好，但内部状态变量 $x(t)$ 会无限增长，最终导致仿真中的数值溢出或真实系统中的物理故障。<img src="file:///E:/桌面文件存放/学习资料/NUS/NUS/NUS%20Courses/ME5401%20Linear%20System/Linear-System-Mini-Project/task4/task4_intrnal%20stability.png" title="" alt="task4_intrnal stability" style="zoom:33%;">

## 3. Discussion and Conclusion

The decoupling controller design for Task 4 mathematically succeeded in diagonalising the system's transfer function matrix, thereby satisfying the structural decoupling requirement. However, the decoupled system proved internally unstable. The decoupling process induces an unstable hidden mode, highlighting a fundamental limitation of state feedback decoupling: if the transfer zeros of the controlled object are unstable and the decoupling process causes fixed modes to coincide with these zeros, the system cannot be stabilised through decoupling. For this particular diesel engine system, the pure decoupling strategy is hazardous due to the loss of internal stability.

> Task 4 的解耦控制器设计在数学上成功地将系统的传递函数矩阵对角化，满足了结构解耦的要求。然而，对于“解耦后的系统内部并不稳定。解耦过程诱发了一个不稳定的隐藏模态，这凸显了状态反馈解耦的一个基本局限性：如果受控对象的传输零点是不稳定的，且解耦过程导致固定模态与这些零点重合，则无法通过解耦来稳定系统。对于这个特定的柴油发动机系统，由于内部稳定性的丧失，纯解耦策略是危险的。
