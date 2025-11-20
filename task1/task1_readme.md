# Task 1 Report: Pole Placement Controller Design for a Diesel Engine Air System

## 1. Introduction

~~This report details the design of a state feedback controller for a diesel engine air system, as outlined in Task 1 of the ME5401/EE5101 Linear Systems mini-project. The primary objective of modern engine control is to meet increasingly stringent government emission regulations for pollutants such as oxides of nitrogen (NOx) and particulate matter (PM), while maintaining high performance and fuel efficiency. This is achieved by precisely managing the engine's air-to-fuel ratio (AFR) and the rate of exhaust gas recirculation (EGR). The control of this system is challenging due to the significant cross-coupling between the actuators, namely the Variable Geometry Turbocharger (VGT) and the EGR valve. Adjusting one actuator invariably affects the operating point and dynamic response of the other, making it a classic Multiple-Input Multiple-Output (MIMO) control problem.~~

> ~~本报告详细阐述了为柴油机空气系统设计状态反馈控制器的过程，该任务是ME5401/EE5101线性系统小型项目中的任务一。现代发动机控制的首要目标是在满足日益严格的政府对氮氧化物（NOx）和颗粒物（PM）等污染物排放法规的同时，保持高性能和燃油效率。这需要通过精确管理发动机的空燃比（AFR）和废气再循环率（EGR）来实现。该系统的控制之所以具有挑战性，是因为执行器——即可变几何涡轮（VGT）和EGR阀——之间存在显著的交叉耦合。调节一个执行器总是会影响另一个执行器的操作点和动态响应，这使其成为一个典型的多输入多输出（MIMO）控制问题。~~

The project provides a fourth-order, two-input, two-output linear state-space model, obtained through system identification, which describes the dynamics of the plant around a specific operating point. The model is given by the standard state-space equations:

> 该项目提供了一个通过系统辨识获得的四阶线性状态空间模型。MIMO模型包含两个输入：u1: VGT（可变几何涡轮）的叶片位置；u2: EGR（废气再循环）的阀门位置。同时输出也有两项：y1: 缸内空燃比 (AFR)；y2: 进气歧管EGR百分比。该模型描述了设备在特定工作点附近的动态特性，状态空间方程如下：

$$
\dot{x} = Ax + Bu
$$

$$
y = Cx
$$

Task 1 specifically addresses the regulation problem. The goal is to design a state feedback controller of the form `u = -Kx` that stabilizes the system and ensures a desirable transient response from a non-zero initial state. The performance specifications require that for a step input, the closed-loop system exhibits an overshoot of less than 10% and a 2% settling time of less than 5 seconds. This report will document the entire design process, from system analysis and pole selection to controller synthesis and performance verification, strictly following the principles of pole placement for MIMO systems as detailed in Chapter 7 of the course notes.

> 任务一专门解决调节问题。其目标是设计一个形式为 `u = -Kx` 的状态反馈控制器，该控制器能够使系统稳定可控，并确保系统从一个非零初始状态恢复时具有理想的瞬态响应。具体来说，对于阶跃输入，闭环系统的超调量 M_p 小于10%，且2%稳定时间 t_s 小于20秒。

## 2. Task Solvement

The design process for the state feedback controller is systematically executed in four main steps: system definition and analysis, selection of desired closed-loop dynamics, controller synthesis, and simulation-based verification. This entire workflow is implemented in the MATLAB script `task1_pole_placement_design.m`.

> 状态反馈控制器的设计过程系统地分为四个主要步骤：系统矩阵的确定与可控性检验、确定期望的闭环极点位置、计算反馈增益矩阵 K以及仿真与性能分析。整个工作流程在MATLAB脚本`task1_pole_placement_design.m`中实现。

First, the system-specific state-space matrices `A`, `B`, and `C` were determined by substituting the user-specific parameters (`a=8, b=4, c=0, d=1`) into the formulas provided in the project description. This was accomplished in the initial section of the MATLAB script. The resulting numerical matrices, which form the basis for all subsequent calculations, are shown below:

> 首先，将学号对应的参数参数`a=8, b=4, c=0, d=1`代入项目给出的参数辨识公式，确定系统特定的状态空间矩阵`A`、`B`和`C`。得到的数值矩阵结果如图所示：

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

> 可控性是任意极点配置的基本前提。根据 chapter 7 Pole Placement 的理论，一个系统可控的充要条件是其可控性矩阵满秩。
> 
> 对于这个四阶MIMO系统，可控性矩阵定义为 $W_c = [B, AB, A^2B, A^3B]$。因此本项目使用MATLAB代码 `Wc = ctrb(A, B); rank_Wc = rank(Wc);` 计算该可控矩阵的秩。结果显示 `rank_Wc = 4`，因此确认了系统是完全可控的，从而保证了项目可以将闭环极点配置在s左半平面上的任何期望位置。

Next, the desired closed-loop pole locations were determined based on the transient performance specifications. Using the second-order system approximation formulas from Chapter 7, the requirement for a settling time $t_s < 5$ seconds implies that the real part of the dominant poles must satisfy $|Re(\lambda)| = \zeta\omega_n > 4/t_s = 0.8$. The requirement for an overshoot $M_p < 10%$ implies that the damping ratio must satisfy $\zeta > 0.591$. To achieve a fast response with acceptable overshoot, a pair of dominant complex conjugate poles was chosen at $\lambda_{1,2} = -1.5 \pm 1.5j$. These poles have a real part of -1.5 (satisfying the settling time requirement) and a damping ratio of $\zeta = 0.707$ (satisfying the overshoot requirement). To ensure their dominance, two additional non-dominant poles were placed further into the left-half plane at $\lambda_3 = -5$ and $\lambda_4 = -6$, which are approximately 3-4 times faster than the dominant poles. The final set of desired poles is `P_desired = [-1.5 + 1.5j, -1.5 - 1.5j, -5.0, -6.0]`.

> 接下来，项目根据瞬态性能指标确定了期望的闭环极点位置。利用第7章中的二阶系统近似公式，稳定时间 $t_s = 4/ \zeta ·\omega_n < 20$ 秒的要求意味着主导极点的实部必须满足 $|Re(\lambda)| = \zeta\omega_n > 4/t_s = 0.2$。超调量 $M_p = ... < 10%$ 的要求则意味着阻尼比必须满足 $\zeta > 0.591$。为了在可接受的超调下实现快速响应，我们选择了一对主导复共轭极点位于 $\lambda_{1,2} = -0.4 \pm 0.4j$。这对极点的实部为-0.4（满足稳定时间要求），阻尼比为 $\zeta = 0.707$（满足超调要求）。为了确保它们是主导极点，另外两个非主导极点被放置在S左半平面的更深处，位于 $\lambda_3 = -1.2$ 和 $\lambda_4 = -2.0$，它们比主导极点快大约3-5倍。因此，最终确定的期望极点集为 `P_desired = [-0.4 + 0.4j, -0.4 - 0.4j, -1.2, -2.0]`。

For the controller synthesis, the Unity Rank Method from Chapter 7 was employed. This method simplifies the MIMO design problem by converting the system into an equivalent Single-Input Single-Output (SISO) system. This is achieved by defining the control input vector as $u = qv$, where `q` is a weighting vector and `v` is a new scalar input. The state equation becomes $\dot{x} = Ax + (Bq)v$. A weighting vector of `q = [1; 1]` was chosen to ensure that both physical actuators (VGT and EGR) are utilized by the controller. The controllability of the resulting SISO pair `(A, Bq)` was verified and confirmed. Subsequently, the SISO feedback gain vector $k_{siso}$ was calculated using the `acker` function in MATLAB, which implements the Ackermann's formula, to place the poles of `(A - Bq * k_siso)` at the desired locations. The final MIMO feedback gain matrix `K` was then constructed using the relation $K = qk_{siso}$. The resulting gain matrix is:

> 在设计反馈增益矩阵 K阶段，本项目采用单位秩方法（Unity Rank K Method）。该方法通过将MIMO设计问题转化为一个等效的单输入单输出（SISO）系统来简化设计。具体来说，通过定义输入向量为
> 
>  $u = qv$ 
> 
> 其中`q`是一个权重向量，`v`是一个新的标量输入，状态方程因此变为
> 
>  $\dot{x} = Ax + (Bq)v$
> 
> 我们选择权重向量为<mark> `q = [1; 1]` ？？？最后一次课（chap 10?11?summary?）表示不要选择[1;1]?</mark>以确保两个物理执行器（VGT和EGR）都被控制器所用。所得到的SISO系统`(A, Bq)`的可控性进行了了检验和确认。
> 
> ![补图](file:///C:/Users/16612/Pictures/Typedown/3b638d1c-61cd-4dec-a580-5dc6e4ee4eb6.png)
> 
> 随后，我们使用MATLAB中的`acker`函数来实现Ackermann公式，计算了SISO反馈增益向量 $k_{siso}$，将`(A - Bq * k_siso)`的极点配置到期望位置。最终的MIMO反馈增益矩阵`K`通过关系 $K = qk_{siso}$ 构建。得到的增益矩阵如下：

```
Calculated Feedback Gain Matrix K:
   -0.6222    2.3625   -3.1444    1.3715
   -0.6222    2.3625   -3.1444    1.371501
```

Finally, the performance of the designed controller was verified through simulation. The closed-loop system, represented by `sys_cl = ss(A-B*K, B, C, 0)`, was subjected to two tests as required. First, the response from a non-zero initial state $x_0 = [0.5; -0.1; 0.3; -0.8]$ was simulated. The results, shown in the figure below, demonstrate that all four state variables converge to zero in approximately 4 seconds, indicating excellent regulation performance. The corresponding control signals `u1` and `u2` exhibit an initial peak of about 4.5 units before rapidly decaying to zero, which is a reasonable magnitude for a simulation context.

> 最后，系统输入阶跃响应进行仿真与性能分析来验证所设计控制器的性能。闭环系统由`sys_cl = ss(A-B*K, B, C, 0)`表示，并按要求进行了两项测试：
> 
> 首先，仿真系统在非零初始状态 $x_0 = [0.5; -0.1; 0.3; -0.8]$ 下的响应。如下图所示，结果表明所有四个状态变量在大约4秒内收敛到零，显示了出色的调节性能。相应的控制信号`u1`和`u2`在初始阶段表现出约4.5个单位的峰值，然后迅速衰减为零，性能符合项目要求。

![85ff2c6d-59c1-4766-9af6-65d2f2fe3099](file:///C:/Users/16612/Pictures/Typedown/85ff2c6d-59c1-4766-9af6-65d2f2fe3099.png)

Second, the step response of the closed-loop system was evaluated by applying a unit step to each input channel separately. The output responses, shown in the figure below, confirm that the performance specifications are met. The settling time for all outputs is well under 5 seconds, and the overshoot is minimal, far below the 10% limit. This validates the effectiveness of the pole placement design.

> 其次，分别对每个输入通道施加单位阶跃来评估闭环系统的阶跃响应。输出响应也表明性能指标已符合要求，所有输出的稳定时间都远小于5秒，且超调量极小，远低于10%的限制，如下图所示。因此验证了极点配置设计的有效性。

![bd7adb88-bfab-4887-8da4-2bf410061533](file:///C:/Users/16612/Pictures/Typedown/bd7adb88-bfab-4887-8da4-2bf410061533.png)

## 3. Discussion and Conclusion

This task successfully demonstrated the application of the pole placement method to a practical MIMO control problem. Through a systematic design process, a state feedback controller was synthesized that not only stabilized the system but also precisely shaped its dynamic response to meet stringent performance criteria. The key takeaway is the direct relationship between the location of closed-loop poles and the system's transient behavior, and how state feedback provides a powerful tool to manipulate these poles.

> 任务1成功地在实际MIMO控制问题中应用极点配置方法，通过系统矩阵的确定与可控性检验、确定期望的闭环极点位置、计算反馈增益矩阵 K以及仿真与性能分析的系统化设计流程，项目成功设计了一个状态反馈控制器。处理MIMO控制器设计非唯一性问题时，单位秩方法确实是一个有效策略。~~设置一个权重向量`q`来简化MIMO问题并仍然达到预期的性能。~~`q`的选择直接影响最终增益矩阵`K`的结构，从而影响系统的输入。
> 
> 关键的收获是理解了闭环极点位置与系统瞬态行为之间的直接关系，以及状态反馈作为一种强大的工具来操控这些极点的能力。


