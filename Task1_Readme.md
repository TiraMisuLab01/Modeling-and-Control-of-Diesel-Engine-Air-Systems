# Task 1 Report: Pole Placement Controller Design for a Diesel Engine Air System

## 1. Introduction

This report details the design of a state feedback controller for a diesel engine air system, as outlined in Task 1 of the ME5401/EE5101 Linear Systems mini-project. The primary objective of modern engine control is to meet increasingly stringent government emission regulations for pollutants such as oxides of nitrogen (NOx) and particulate matter (PM), while maintaining high performance and fuel efficiency. This is achieved by precisely managing the engine's air-to-fuel ratio (AFR) and the rate of exhaust gas recirculation (EGR). The control of this system is challenging due to the significant cross-coupling between the actuators, namely the Variable Geometry Turbocharger (VGT) and the EGR valve. Adjusting one actuator invariably affects the operating point and dynamic response of the other, making it a classic Multiple-Input Multiple-Output (MIMO) control problem.

> 本报告详细阐述了为柴油机空气系统设计状态反馈控制器的过程，该任务是ME5401/EE5101线性系统小型项目中的任务一。现代发动机控制的首要目标是在满足日益严格的政府对氮氧化物（NOx）和颗粒物（PM）等污染物排放法规的同时，保持高性能和燃油效率。这需要通过精确管理发动机的空燃比（AFR）和废气再循环率（EGR）来实现。该系统的控制之所以具有挑战性，是因为执行器——即可变几何涡轮（VGT）和EGR阀——之间存在显著的交叉耦合。调节一个执行器总是会影响另一个执行器的操作点和动态响应，这使其成为一个典型的多输入多输出（MIMO）控制问题。

The project provides a fourth-order, two-input, two-output linear state-space model, obtained through system identification, which describes the dynamics of the plant around a specific operating point. The model is given by the standard state-space equations:

> 该项目提供了一个通过系统辨识获得的四阶、双输入、双输出的线性状态空间模型，该模型描述了设备在特定工作点附近的动态特性。该模型由标准的状态空间方程给出：

$$
\dot{x} = Ax + Bu
$$

$$
y = Cx
$$

Task 1 specifically addresses the regulation problem. The goal is to design a state feedback controller of the form `u = -Kx` that stabilizes the system and ensures a desirable transient response from a non-zero initial state. The performance specifications require that for a step input, the closed-loop system exhibits an overshoot of less than 10% and a 2% settling time of less than 5 seconds. This report will document the entire design process, from system analysis and pole selection to controller synthesis and performance verification, strictly following the principles of pole placement for MIMO systems as detailed in Chapter 7 of the course notes.

> 任务一专门解决调节问题。其目标是设计一个形式为 `u = -Kx` 的状态反馈控制器，该控制器能够稳定系统，并确保系统从一个非零初始状态恢复时具有理想的瞬态响应。性能指标要求对于阶跃输入，闭环系统的超调量小于10%，且2%稳定时间小于5秒。本报告将记录从系统分析、极点选择到控制器综合和性能验证的整个设计过程，并严格遵循课程讲义第7章中详述的MIMO系统极点配置原理。

## 2. Task Solvement

The design process for the state feedback controller is systematically executed in four main steps: system definition and analysis, selection of desired closed-loop dynamics, controller synthesis, and simulation-based verification. This entire workflow is implemented in the MATLAB script `task1_pole_placement_design.m`.

> 状态反馈控制器的设计过程系统地分为四个主要步骤：系统定义与分析、期望闭环动态选择、控制器综合以及基于仿真的验证。整个工作流程在MATLAB脚本`task1_pole_placement_design.m`中实现。

First, the system-specific state-space matrices `A`, `B`, and `C` were determined by substituting the user-specific parameters (`a=8, b=4, c=0, d=1`) into the formulas provided in the project description. This was accomplished in the initial section of the MATLAB script. The resulting numerical matrices, which form the basis for all subsequent calculations, are shown below:

> 首先，通过将用户特定的参数（`a=8, b=4, c=0, d=1`）代入项目说明书中提供的公式，确定了系统特定的状态空间矩阵`A`、`B`和`C`。这一步是在MATLAB脚本的初始部分完成的。这些作为所有后续计算基础的数值矩阵如下所示：

```
Matrix A:
   -8.0487   -0.0399   -5.0500    3.5846
   -4.5740    3.0012   -4.3662   -1.5183
    3.7698   16.1212  -17.0853    4.4936
   -9.8978    8.3742   -4.4331   -4.2878

Matrix B:
    0.4564    0.0319
    0.0199   -0.0200
    4.4939    1.7983
   -1.4269   -0.2730

Matrix C:
   -3.2988   -2.1861    0.0370   -0.0109
    0.2282   -2.1506   -0.0104    0.0163
```

The fundamental prerequisite for arbitrary pole placement is system controllability. As established in Chapter 7, a system is controllable if and only if its controllability matrix has full rank. For this 4th-order MIMO system, the controllability matrix is defined as $W_c = [B, AB, A^2B, A^3B]$. The rank of this matrix was computed using the MATLAB code `Wc = ctrb(A, B); rank_Wc = rank(Wc);`. The result, `rank_Wc = 4`, confirms that the system is fully controllable, thus guaranteeing that we can place the closed-loop poles at any desired location in the s-plane.

> 可控性是任意极点配置的基本前提。根据第7章的理论，一个系统可控的充要条件是其可控性矩阵满秩。对于这个四阶MIMO系统，可控性矩阵定义为 $W_c = [B, AB, A^2B, A^3B]$。我们使用MATLAB代码 `Wc = ctrb(A, B); rank_Wc = rank(Wc);` 计算了该矩阵的秩。结果显示 `rank_Wc = 4`，确认了系统是完全可控的，从而保证了我们可以将闭环极点配置在s平面上的任何期望位置。

Next, the desired closed-loop pole locations were determined based on the transient performance specifications. Using the second-order system approximation formulas from Chapter 7, the requirement for a settling time $t_s < 5$ seconds implies that the real part of the dominant poles must satisfy $|Re(\lambda)| = \zeta\omega_n > 4/t_s = 0.8$. The requirement for an overshoot $M_p < 10%$ implies that the damping ratio must satisfy $\zeta > 0.591$. To achieve a fast response with acceptable overshoot, a pair of dominant complex conjugate poles was chosen at $\lambda_{1,2} = -1.5 \pm 1.5j$. These poles have a real part of -1.5 (satisfying the settling time requirement) and a damping ratio of $\zeta = 0.707$ (satisfying the overshoot requirement). To ensure their dominance, two additional non-dominant poles were placed further into the left-half plane at $\lambda_3 = -5$ and $\lambda_4 = -6$, which are approximately 3-4 times faster than the dominant poles. The final set of desired poles is `P_desired = [-1.5 + 1.5j, -1.5 - 1.5j, -5.0, -6.0]`.

> 接下来，我们根据瞬态性能指标确定了期望的闭环极点位置。利用第7章中的二阶系统近似公式，稳定时间 $t_s < 5$ 秒的要求意味着主导极点的实部必须满足 $|Re(\lambda)| = \zeta\omega_n > 4/t_s = 0.8$。超调量 $M_p < 10%$ 的要求则意味着阻尼比必须满足 $\zeta > 0.591$。为了在可接受的超调下实现快速响应，我们选择了一对主导复共轭极点位于 $\lambda_{1,2} = -1.5 \pm 1.5j$。这对极点的实部为-1.5（满足稳定时间要求），阻尼比为 $\zeta = 0.707$（满足超调要求）。为了确保它们的主导地位，另外两个非主导极点被放置在S左半平面的更深处，位于 $\lambda_3 = -5$ 和 $\lambda_4 = -6$，它们比主导极点快大约3-4倍。因此，最终确定的期望极点集为 `P_desired = [-1.5 + 1.5j, -1.5 - 1.5j, -5.0, -6.0]`。

For the controller synthesis, the Unity Rank Method from Chapter 7 was employed. This method simplifies the MIMO design problem by converting the system into an equivalent Single-Input Single-Output (SISO) system. This is achieved by defining the control input vector as $u = qv$, where `q` is a weighting vector and `v` is a new scalar input. The state equation becomes $\dot{x} = Ax + (Bq)v$. A weighting vector of `q = [1; 1]` was chosen to ensure that both physical actuators (VGT and EGR) are utilized by the controller. The controllability of the resulting SISO pair `(A, Bq)` was verified and confirmed. Subsequently, the SISO feedback gain vector $k_{siso}$ was calculated using the `acker` function in MATLAB, which implements the Ackermann's formula, to place the poles of `(A - Bq * k_siso)` at the desired locations. The final MIMO feedback gain matrix `K` was then constructed using the relation $K = qk_{siso}$. The resulting gain matrix is:

> 在控制器综合阶段，我们采用了第7章介绍的单位秩方法。该方法通过将MIMO设计问题转化为一个等效的单输入单输出（SISO）系统来简化设计。这是通过定义控制输入向量为 $u = qv$ 实现的，其中`q`是一个权重向量，`v`是一个新的标量输入，状态方程因此变为 $\dot{x} = Ax + (Bq)v$。我们选择权重向量为 `q = [1; 1]` 以确保两个物理执行器（VGT和EGR）都被控制器所用。所得到的SISO系统`(A, Bq)`的可控性得到了检验和确认。随后，我们使用MATLAB中的`acker`函数（该函数实现了Ackermann公式）计算了SISO反馈增益向量 $k_{siso}$，以将`(A - Bq * k_{siso})`的极点配置到期望位置。最终的MIMO反馈增益矩阵`K`通过关系 $K = qk_{siso}$ 构建。得到的增益矩阵如下：

```
计算出的反馈增益矩阵 K:
    1.0283    0.7312   -1.6269    1.5801
    1.0283    0.7312   -1.6269    1.5801
```

Finally, the performance of the designed controller was verified through simulation. The closed-loop system, represented by `sys_cl = ss(A-B*K, B, C, 0)`, was subjected to two tests as required. First, the response from a non-zero initial state $x_0 = [0.5; -0.1; 0.3; -0.8]$ was simulated. The results, shown in the figure below, demonstrate that all four state variables converge to zero in approximately 4 seconds, indicating excellent regulation performance. The corresponding control signals `u1` and `u2` exhibit an initial peak of about 4.5 units before rapidly decaying to zero, which is a reasonable magnitude for a simulation context.

> 最后，我们通过仿真验证了所设计控制器的性能。闭环系统由`sys_cl = ss(A-B*K, B, C, 0)`表示，并按要求进行了两项测试。首先，仿真了系统在非零初始状态 $x_0 = [0.5; -0.1; 0.3; -0.8]$ 下的响应。如下图所示，结果表明所有四个状态变量在大约4秒内收敛到零，显示了出色的调节性能。相应的控制信号`u1`和`u2`在初始阶段表现出约4.5个单位的峰值，然后迅速衰减为零，这在仿真环境中是一个合理的幅值。

![task1_state variable response and control signal_EN](E:\桌面文件存放\学习资料\NUS\NUS\NUS%20Courses\ME5401%20Linear%20System\Mini%20Project\task1_state%20variable%20response%20and%20control%20signal_EN.png)

Second, the step response of the closed-loop system was evaluated by applying a unit step to each input channel separately. The output responses, shown in the figure below, confirm that the performance specifications are met. The settling time for all outputs is well under 5 seconds, and the overshoot is minimal, far below the 10% limit. This validates the effectiveness of the pole placement design.

> 其次，我们通过分别对每个输入通道施加单位阶跃来评估闭环系统的阶跃响应。如下图所示的输出响应证实了性能指标已得到满足。所有输出的稳定时间都远小于5秒，且超调量极小，远低于10%的限制。这验证了极点配置设计的有效性。

![task1_two different type of step response output_EN](E:\桌面文件存放\学习资料\NUS\NUS\NUS%20Courses\ME5401%20Linear%20System\Mini%20Project\task1_two%20different%20type%20of%20step%20response%20output_EN.png)

## 3. Discussion and Conclusion

This task successfully demonstrated the application of the pole placement method to a practical MIMO control problem. Through a systematic design process, a state feedback controller was synthesized that not only stabilized the system but also precisely shaped its dynamic response to meet stringent performance criteria. The key takeaway is the direct relationship between the location of closed-loop poles and the system's transient behavior, and how state feedback provides a powerful tool to manipulate these poles.

> 本次任务成功地展示了极点配置方法在实际MIMO控制问题中的应用。通过一个系统化的设计流程，我们合成了一个状态反馈控制器，它不仅稳定了系统，还精确地塑造了其动态响应以满足严格的性能标准。关键的收获是理解了闭环极点位置与系统瞬态行为之间的直接关系，以及状态反馈作为一种强大的工具来操控这些极点的能力。

The use of the Unity Rank method proved to be an effective strategy for handling the non-uniqueness of the MIMO controller design. By selecting a weighting vector `q`, we were able to simplify the problem and still achieve the desired performance. It was also observed that the choice of `q` directly influences the structure of the final gain matrix `K` and thus the distribution of control effort between the actuators. The entire process, from theoretical analysis to simulation, confirms that the designed controller fully satisfies all requirements of Task 1. This exercise also highlights the importance of the initial assumption that all state variables are measurable, setting a clear motivation for the subsequent tasks involving state estimation.

> 单位秩方法的使用被证明是处理MIMO控制器设计非唯一性问题的有效策略。通过选择一个权重向量`q`，我们能够简化问题并仍然达到预期的性能。我们还观察到，`q`的选择直接影响最终增益矩阵`K`的结构，从而影响控制力在不同执行器之间的分配。从理论分析到仿真的整个过程证实了所设计的控制器完全满足任务一的所有要求。本次练习也凸显了“所有状态变量均可测量”这一初始假设的重要性，为后续涉及状态估计的任务提供了明确的动机。
