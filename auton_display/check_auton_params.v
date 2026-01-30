/*--------------------------------------------------------------------
  版权声明（略）
  -------------------------------------------------------------------- */

//
//  ----------------------------------------------------------------------------
//                    设计信息
//  ----------------------------------------------------------------------------
//  文件名          : sync_support.v
//  所属组织        : MCO
//  版本标签        : 1.1.11
//  日期            : $Date: Wed Jun 12 23:47:03 2019 $
//  修订号          : $Revision: 1.61 $
//
//  IP 名称         : SYNC_ 通用同步模块
//  功能描述        : 为 I3C 从设备提供跨时钟域（CDC）同步支持
//    本文件包含多种跨时钟域同步机制，包括两相握手（2-phase handshake）、
//    脉冲/电平同步、FIFO 同步等，用于 SCL（I3C 时钟）与系统主时钟（CLK）之间的信号传递。
//    由于 SCL 可能突然停止（非自由运行时钟），因此采用两相握手机制确保可靠性。
//
//  ----------------------------------------------------------------------------
//                    实现细节
//  ----------------------------------------------------------------------------
//  - 使用命名块便于 CDC 检查工具（如 Spyglass）识别同步逻辑。
//  - 在现代工艺下，单级触发器通常足够；但在较老工艺中可能需使用专用同步触发器。
//  - I3C SCL 最低速率为约 25 MHz（周期 40ns），留有足够时间让亚稳态收敛。
//
//  命名约定说明：
//    SYNC_   = 同步模块
//    2PH_    = 两相握手（Two-Phase Handshake）
//    S2C_    = SCL 到 CLK 域（SCL-to-CLK）
//    C2S_    = CLK 到 SCL 域（CLK-to-SCL）
//    STATE   = 状态型输出（需显式清除）
//    LVL_    = 电平输入（Level）
//    LVLH_   = 高电平边沿检测（Level High Edge）
//    ASet    = 异步置位，本地清零
//    AClr    = 本地置位，异步清零
//    Seq2    = 两级序列器，确保本地产生单周期脉冲
//  ----------------------------------------------------------------------------

// ============================================================================
// 模块：SYNC_2PH_S2C_STATE
// 功能：将 SCL 域的**脉冲触发信号**通过两相握手同步到 CLK 域，并保持为状态信号，
//       直到被 CLK 域显式清除。
// 应用场景：例如 SCL 域检测到 START 条件，需通知 CLK 域进入新状态。
// ============================================================================
module SYNC_2PH_S2C_STATE( 
  input             rst_n,      // 异步复位（低有效）
  input             scl,        // SCL 时钟（用于采样 trig_scl）
  input             clk,        // 系统主时钟
  input             trig_scl,   // SCL 域输入的单周期脉冲触发信号
  output            out_clk,    // CLK 域输出的状态信号（高表示事件发生）
  input             clear_clk   // CLK 域输入的清除信号（高有效）
);

  reg scl_hshake;   // SCL 域握手信号：trig_scl 触发时翻转
  reg clk_state;    // CLK 域状态：当握手信号与 ACK 不同时置 1
  reg clk_ack;      // CLK 域对握手的确认信号

  // SCL 域：根据 trig_scl 脉冲翻转握手信号
  always @ (posedge scl or negedge rst_n)
    if (!rst_n)
      scl_hshake <= 1'b0;
    else if (trig_scl)                  // trig_scl 为单周期脉冲
      scl_hshake <= ~scl_hshake;        // 翻转状态，形成两相编码

  // CLK 域：检测握手变化，置位状态
  always @ (posedge clk or negedge rst_n)
    if (!rst_n)
      clk_state <= 1'b0;
    else if (scl_hshake ^ clk_ack)      // 握手信号与 ACK 不同 → 有新事件
      clk_state <= 1'b1;                // 置位状态
    else if (clear_clk)                 // 显式清除
      clk_state <= 1'b0;

  assign out_clk = clk_state;

  // CLK 域：仅在状态已置位且仍存在差异时更新 ACK，避免亚稳态外泄
  always @ (posedge clk or negedge rst_n)
    if (!rst_n)
      clk_ack <= 1'b0;
    else if (clk_state & (scl_hshake ^ clk_ack))
      clk_ack <= ~clk_ack;              // 翻转 ACK，完成一次握手

endmodule 

// ============================================================================
// 模块：SYNC_2PH_LVL_S2C_STATE
// 功能：将 SCL 域的**持续电平信号**同步到 CLK 域，作为状态信号，
//       直到被 CLK 域显式清除。
// 与上一模块区别：输入是电平而非脉冲，因此无需 SCL 域的翻转寄存器。
// 注意：若电平持续时间短于 CLK 周期，可能无法被捕获。
// ============================================================================
module SYNC_2PH_LVL_S2C_STATE(
  input             rst_n,
  input             scl,        // 保留但未使用（仅为接口一致性）
  input             clk,
  input             trig_scl,   // SCL 域的电平信号（高表示有效）
  output            out_clk,
  input             clear_clk
);

  reg clk_state;
  reg clk_ack;

  // CLK 域直接采样 trig_scl（跨时钟域）
  always @ (posedge clk or negedge rst_n)
    if (!rst_n)
      clk_state <= 1'b0;
    else if (trig_scl ^ clk_ack)        // 电平变化（相对于 ACK）
      clk_state <= 1'b1;
    else if (clear_clk)
      clk_state <= 1'b0;
  assign out_clk = clk_state;

  // 更新 ACK
  always @ (posedge clk or negedge rst_n)
    if (!rst_n)
      clk_ack <= 1'b0;
    else if (clk_state & (trig_scl ^ clk_ack))
      clk_ack <= ~clk_ack;

endmodule

// ============================================================================
// 模块：SYNC_2PH_LVLH_S2C_STATE
// 功能：检测 SCL 域**高电平的上升沿**（即从低到高的跳变），
//       并在 CLK 域生成一个单次状态信号（需显式清除）。
// 设计难点：因跨时钟域，不能直接用 trig_scl 上升沿检测（可能漏检）。
// 解决方案：用两级寄存器记录电平历史，通过异或检测边沿。
// ============================================================================
module SYNC_2PH_LVLH_S2C_STATE(
  input             rst_n,
  input             scl,        // 保留
  input             clk,
  input             trig_scl,   // SCL 域电平信号
  output            out_clk,
  input             clear_clk
);

  reg clk_state;
  reg [1:0] clk_ack;  // clk_ack[0]: 当前采样值；clk_ack[1]: 上一周期采样值

  // 第一级：同步采样 trig_scl
  // 第二级：延迟一拍，用于边沿检测
  always @ (posedge clk or negedge rst_n)
    if (!rst_n)
      clk_ack <= 2'b00;
    else if (clk_ack[0] ^ trig_scl)     // 电平变化（CDC 安全）
      clk_ack[0] <= ~clk_ack[0];        // 翻转表示变化
    else if (^clk_ack)                  // 若两级不同（即刚发生边沿）
      clk_ack[1] <= ~clk_ack[1];        // 更新历史值

  // 边沿检测：(^clk_ack) 表示当前与历史不同，且 ~clk_ack[1] 确保是上升沿首次捕获
  always @ (posedge clk or negedge rst_n)
    if (!rst_n)
      clk_state <= 1'b0;
    else if (^clk_ack & ~clk_ack[1])    // 检测到上升沿
      clk_state <= 1'b1;
    else if (clear_clk)
      clk_state <= 1'b0;
  assign out_clk = clk_state;

endmodule

// ============================================================================
// 模块：SYNC_Pulse_S2C
// 功能：将 CLK 域的**本地置位信号**（local_set）以脉冲形式传递到 SCL 域，
//       并在 SCL 域生成一个单周期脉冲（o_pulse）。
// 采用四相握手机制（隐含在 svalue/cvalue 循环中），确保可靠传递。
// ============================================================================
module SYNC_Pulse_S2C(
  input SCL,
  input CLK,
  input RSTn,
  input local_set,      // CLK 域置位请求（高有效）
  output o_pulse        // SCL 域输出的单周期脉冲
);

  reg svalue, cvalue, cpulse;

  // SCL 域：接收来自 CLK 的置位请求，并等待确认
  always @ (posedge SCL or negedge RSTn)
    if (!RSTn)
      svalue <= 1'b0;
    else if (local_set)
      svalue <= 1'b1;           // 置位
    else if (cvalue)            // 收到 CLK 域确认（CDC）
      svalue <= 1'b0;           // 清除

  // CLK 域：检测 svalue，生成确认并产生内部脉冲标记
  always @ (posedge CLK or negedge RSTn)
    if (!RSTn)
      cvalue <= 1'b0;
    else if (svalue)            // 检测到 SCL 域请求（CDC）
      cvalue <= 1'b1;
    else 
      cvalue <= 1'b0;

  // 记录 cvalue 的前一状态，用于生成单周期脉冲
  always @ (posedge CLK or negedge RSTn)
    if (!RSTn)
      cpulse <= 1'b0;
    else 
      cpulse <= cvalue;

  // 脉冲 = 当前高 & 上一周期低
  assign o_pulse = cvalue & ~cpulse;

endmodule

// ============================================================================
// 模块：SYNC_ASet_Seq2
// 功能：将**异步置位信号**（async_set）同步到 CLK 域，并生成**单周期脉冲**。
//       异步信号可能来自其他时钟域或外部引脚。
//       使用两级序列器（seq）确保只输出一个 CLK 周期的高脉冲。
// ============================================================================
module SYNC_ASet_Seq2(
  input CLK,
  input RSTn,
  input async_set,      // 异步置位（高有效）
  input local_clear,    // 本地清除（高有效）
  output o_pulse        // 单周期脉冲输出
);

  reg value, seq;

  // 同步 async_set 到 CLK 域（单级同步器，假设 async_set 脉宽足够）
  always @ (posedge CLK or negedge RSTn)
    if (!RSTn)
      value <= 1'b0;
    else if (async_set)
      value <= 1'b1;
    else if (local_clear)
      value <= 1'b0;

  // 序列器：每当 value 变化，seq 翻转
  always @ (posedge CLK or negedge RSTn)
    if (!RSTn)
      seq <= 1'b0;
    else if (seq ^ value)
      seq <= ~seq;

  // 脉冲 = value 为高 且 seq 为低（即刚置位后的第一个周期）
  assign o_pulse = value & ~seq;

endmodule

// ============================================================================
// 模块：SYNC_AClr_Seq2
// 功能：与 SYNC_ASet_Seq2 对称，但用于**异步清除**场景。
//       本地置位（local_set），异步清除（async_clear），
//       同样输出单周期脉冲。
// ============================================================================
module SYNC_AClr_Seq2(
  input CLK,
  input RSTn,
  input local_set,      // 本地置位
  input async_clear,    // 异步清除（高有效）
  output o_pulse
);

  reg value, seq;

  always @ (posedge CLK or negedge RSTn)
    if (!RSTn)
      value <= 1'b0;
    else if (local_set) 
      value <= 1'b1;
    else if (async_clear)
      value <= 1'b0;

  always @ (posedge CLK or negedge RSTn)
    if (!RSTn)
      seq <= 1'b0;
    else if (seq ^ value)
      seq <= ~seq;

  assign o_pulse = value & ~seq;

endmodule

// ============================================================================
// 模块：SYNC_S2C
// 功能：通用多比特信号从 SCL 域同步到 CLK 域（单级同步器）。
//       适用于数据总线、状态字等。
//       注意：不保证无毛刺，仅用于对亚稳态不敏感的场景（如 FIFO 指针需 Gray 编码）。
// ============================================================================
module SYNC_S2C #(parameter WIDTH=1) ( 
  input             rst_n,
  input             clk,
  input  [WIDTH-1:0] scl_data,   // SCL 域输入数据
  output [WIDTH-1:0] out_clk     // CLK 域输出
);

  reg [WIDTH-1:0] clk_copy;

  assign out_clk = clk_copy;

  always @ (posedge clk or negedge rst_n)
    if (!rst_n)
      clk_copy <= {WIDTH{1'b0}};
    else 
      clk_copy <= scl_data;       // 单级同步

endmodule

// ============================================================================
// 模块：SYNC_C2S
// 功能：通用多比特信号从 CLK 域同步到 SCL 域（单级同步器）。
// ============================================================================
module SYNC_C2S #(parameter WIDTH=1) ( 
  input             rst_n,
  input             scl,
  input  [WIDTH-1:0] clk_data,
  output [WIDTH-1:0] out_scl
);

  reg [WIDTH-1:0] scl_copy;

  assign out_scl = scl_copy;

  always @ (posedge scl or negedge rst_n)
    if (!rst_n)
      scl_copy <= {WIDTH{1'b0}};
    else 
      scl_copy <= clk_data;

endmodule

// ============================================================================
// 模块：SYNC_AClr_C2S
// 功能：将 CLK 域的信号（本地置位，异步清除）传递到 SCL 域，
//       输出为电平信号（非脉冲）。
//       常用于控制信号（如使能、复位等）。
// ============================================================================
module SYNC_AClr_C2S(
  input CLK,
  input RSTn,
  input local_set,      // CLK 域置位
  input async_clear,    // 异步清除（可能来自 SCL 域）
  output o_value        // SCL 域输出电平
);

  reg value;

  always @ (posedge CLK or negedge RSTn)
    if (!RSTn)
      value <= 1'b0;
    else if (local_set) 
      value <= 1'b1;
    else if (async_clear)
      value <= 1'b0;

  assign o_value = value;  // 直接输出，无额外同步（因已在 CLK 域处理）

endmodule