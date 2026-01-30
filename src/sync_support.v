/*--------------------------------------------------------------------
  版权所有 (C) 2015-2019, NXP B.V.
  保留所有权利。
  
  在满足以下条件的情况下，允许以源代码和二进制形式重新分发和使用（无论是否经过修改）：
  
  1. 源代码的重新分发必须保留上述版权声明、此条件列表以及以下免责声明。
  
  2. 二进制形式的重新分发必须在随分发提供的文档和/或其他材料中保留上述版权声明、
     此条件列表以及以下免责声明。
  
  3. 未经事先明确的书面许可，不得使用版权持有者或贡献者的姓名来背书或推广
     衍生自此 HDL 软件的产品。
  
  本（HDL）软件由版权持有者和贡献者“按原样”提供，不承诺任何明示或暗示的保证，
  包括但不限于针对特定用途的适销性和适用性的暗示保证。在任何情况下，版权持有者
  或贡献者均不对任何直接、间接、附带、特别、惩戒性或后果性损害（包括但不限于
  替代商品或服务的采购；使用、数据或利润损失；或业务中断）承担责任，无论其
  起因及基于何种责任理论（无论是合同、严格责任还是侵权行为，包括疏忽或其他原因），
  即使已被告知存在此类损害的可能性。

  注：未授予或暗示任何第三方专利许可。受让方有责任获得任何所需的第三方专利许可。

  关于上述术语的说明：
  1. 软件（Software）和 HDL 软件可以互换使用。
  2. 二进制（Binary）形式包括 FPGA、仿真以及芯片物理形式（如硅片）。
  3. 第 2 条允许在网页或其他不属于分发部分的电子形式中显示此类声明。
  原始 BSD 源码获取地址：
    https://github.com/NXP/i3c-slave-design
  -------------------------------------------------------------------- */

//
//  ----------------------------------------------------------------------------
//                     设计信息
//  ----------------------------------------------------------------------------
//  文件      : sync_support.v
//  组织      : MCO
//  标签      : 1.1.11
//  日期      : $Date: Wed Jun 12 23:47:03 2019 $
//  版本      : $Revision: 1.61 $
//
//  IP 名称   : 通用 SYNC_ 模块库
//  描述      : 从机（Slave）跨时钟域支持
//    此文件包含跨时钟域（CDC）同步支持，类型包括 2 相及其他方案，
//    涵盖脉冲和电平信号。用于 SCL 到（系统）CLK 以及 CLK 到 SCL。
//    适用于单根信号线、握手、FIFO 等。
//    注意：由于 SCL 时钟可能突然停止，因此使用了 2 相握手机制。
//
//  ----------------------------------------------------------------------------
//                     修订历史
//  ----------------------------------------------------------------------------
//
//
//  ----------------------------------------------------------------------------
//                     实现细节
//  ----------------------------------------------------------------------------
//  参见微架构规范和 MIPI I3C 规范。
//
//    此模块支持使用命名块进行跨时钟域处理，以便 Spyglass（或同类工具）识别同步位置。
//    对于大多数现代工艺，单级触发器模型（single flop model）已足够，
//    尽管对于旧工艺，可能需要通过约束更改触发器类型（例如同步触发器，其具有
//    更快的“重力”稳定速度）。
//    在正常情况下，1 级触发器是可行的，因为在较低速路径下，Q 输出的亚稳态噪声
//    在时间上足够短。也就是说，至少有 ~40ns 的时间用于信号稳定并到达另一端。
//  ----------------------------------------------------------------------------

// 命名约定：SYNC_=同步, 2PH_=2相, S2P_=SCL到CLK, STATE=带清除输入的内部状态
//           LVL_=电平输入（相对于脉冲）, LVLH_=高电平有效
// 其他命名：ASet=异步置位，本地清除; AClr=本地置位，异步清除
//           Seq2=2级触发器序列器，确保在本地域获得1个脉冲宽度
module SYNC_2PH_S2C_STATE( 
  input             rst_n,
  input             scl,
  input             clk,
  input             trig_scl,   // 来自 SCL 时钟域的触发输入
  output            out_clk,    // CLK 时钟域的输出状态
  input             clear_clk   // 来自 CLK 时钟域的输入清除信号
  );

  reg               scl_hshake;
  reg               clk_state;
  reg               clk_ack;

  // 注意：这部分逻辑可能有点难懂。scl_ 到 clk_ 的同步是 2 相的。
  // scl_hshake 根据触发信号翻转状态。
  // clk_state 在 (hshake ^ ack == 1) 时置位，用于 CLK 域。
  // clk_ack 仅在 clk_state 改变后才改变，这样我们既不会错过信号，
  // 也不会将亚稳态逻辑（hshake ^ ack）暴露到外部。
  // 我们使用 2 相是因为 SCL 不是持续运行的时钟，可能会停止。
  // 因此，由 CLK 完成所有的采样工作。
  // 请注意，基于状态（state-based）的同步器只能通过显式请求清除；
  // 因此在清除之前可能会发生多次状态改变，这是允许的。

  always @ (posedge scl or negedge rst_n)
    if (!rst_n)
      scl_hshake <= 1'b0;
    else if (trig_scl)                  // 触发脉冲（SCL 域的 1 个周期）
      scl_hshake <= ~scl_hshake;        // 翻转状态

  always @ (posedge clk or negedge rst_n)
    if (!rst_n)
      clk_state  <= 1'b0;
    else if (scl_hshake ^ clk_ack)      // CDC
      clk_state  <= 1'b1;               // 存在差异时置位
    else if (clear_clk)
      clk_state  <= 1'b0;               // 显式清除时清零
  assign out_clk = clk_state;

  // ACK 仅在状态改变后且仍存在差异时改变
  always @ (posedge clk or negedge rst_n)
    if (!rst_n)
      clk_ack    <= 1'b0;
    else if (clk_state & (scl_hshake^clk_ack)) // 随状态改变 - CDC
      clk_ack    <= ~clk_ack; 

endmodule 

module SYNC_2PH_LVL_S2C_STATE(
  input             rst_n,
  input             scl,
  input             clk,
  input             trig_scl,   // 来自 SCL 域的触发电平信号
  output            out_clk,    // CLK 域的输出状态
  input             clear_clk   // 来自 CLK 域的清除输入
  );

  reg               clk_state;
  reg               clk_ack;

  // 详见 SYNC_2PH_S2C_STATE。唯一的区别是电平保持不需要 scl_hshake，
  // 因为它本身就是电平（如果电平相对于 CLK 太短，则不会被采样）。
  // 如果我们需要确保被观察到，则需要独立的位。
  // 未来改进：研究针对 CLK 停止情况下的处理。

  always @ (posedge clk or negedge rst_n)
    if (!rst_n)
      clk_state   <= 1'b0;
    else if (trig_scl ^ clk_ack) // 电平改变 - CDC
      clk_state   <= 1'b1; 
    else if (clear_clk)
      clk_state   <= 1'b0; 
  assign out_clk = clk_state;

  always @ (posedge clk or negedge rst_n)
    if (!rst_n)
      clk_ack     <= 1'b0;
    else if (clk_state &
            (trig_scl ^ clk_ack)) // 仍然存在差异 - CDC
      clk_ack     <= ~clk_ack; 

endmodule

module SYNC_2PH_LVLH_S2C_STATE(
  input             rst_n,
  input             scl,
  input             clk,
  input             trig_scl,   // 来自 SCL 域的触发电平信号
  output            out_clk,    // CLK 域的输出状态
  input             clear_clk   // 来自 CLK 域的清除输入信号
  );

  reg               clk_state;
  reg          [1:0] clk_ack;

  // 详见 SYNC_2PH_S2C_STATE。
  // 高电平有效（Level High）处理起来比较复杂。
  // clk_state 仅在电平变为高电平时才置高（类似于边沿触发）。
  // 因此，我们需要记住之前的状态以捕捉边沿。但是，因为是跨时钟域，
  // 我们不能直接与 SCL 电平对比，因为可能会由于采样早晚导致错过边沿。
  // 所以我们使用一个额外的触发器。
  always @ (posedge clk or negedge rst_n)
    if (!rst_n)
      clk_ack     <= 2'b00;
    else if (clk_ack[0] ^ trig_scl)
      clk_ack[0]  <= ~clk_ack[0];       // CDC
    else if (^clk_ack)
      clk_ack[1]  <= ~clk_ack[1];       // 延迟一个周期

  always @ (posedge clk or negedge rst_n)
    if (!rst_n)
      clk_state   <= 1'b0;
    else if (^clk_ack & ~clk_ack[1])
      clk_state   <= 1'b1; 
    else if (clear_clk)
      clk_state   <= 1'b0; 
  assign out_clk = clk_state;

endmodule

// 下一个是异步置位，本地清除
module SYNC_Pulse_S2C(
  input SCL,
  input CLK,
  input RSTn,
  input local_set,
  output o_pulse);

  // 4 相握手，但输出脉冲
  reg  svalue, cvalue, cpulse;
  always @ (posedge SCL or negedge RSTn)
    if (!RSTn)
      svalue <= 1'b0;
    else if (local_set)
      svalue <= 1'b1;
    else if (cvalue) // CDC
      svalue <= 1'b0;

  always @ (posedge CLK or negedge RSTn)
    if (!RSTn)
      cvalue <= 1'b0;
    else if (svalue) // CDC
      cvalue <= 1'b1;
    else 
      cvalue <= 1'b0;
  always @ (posedge CLK or negedge RSTn)
    if (!RSTn)
      cpulse <= 1'b0;
    else if (cvalue)
      cpulse <= 1'b1;
    else
      cpulse <= 1'b0;
  assign o_pulse = cvalue & ~cpulse;

endmodule


// 下一个是异步置位和本地清除，带有输出 1 个脉冲的序列器
module SYNC_ASet_Seq2(
  input CLK,
  input RSTn,
  input async_set,
  input local_clear, 
  output o_pulse);

  reg  value, seq;
  always @ (posedge CLK or negedge RSTn)
    if (!RSTn)
      value <= 1'b0;
    else if (async_set) // CDC
      value <= 1'b1;
    else if (local_clear) 
      value <= 1'b0;
  always @ (posedge CLK or negedge RSTn)
    if (!RSTn)
      seq <= 1'b0;
    else if (seq ^ value)
      seq <= ~seq;
  // 以下在 1 个时钟周期内输出高脉冲，其余为低电平
  assign o_pulse = value & ~seq;

endmodule

// 下一个是本地置位和异步清除，带有输出 1 个脉冲的序列器
module SYNC_AClr_Seq2(
  input CLK,
  input RSTn,
  input local_set,
  input async_clear, 
  output o_pulse);

  reg  value, seq;
  always @ (posedge CLK or negedge RSTn)
    if (!RSTn)
      value <= 1'b0;
    else if (local_set) 
      value <= 1'b1;
    else if (async_clear) // CDC
      value <= 1'b0;
  always @ (posedge CLK or negedge RSTn)
    if (!RSTn)
      seq <= 1'b0;
    else if (seq ^ value)
      seq <= ~seq;
  // 以下在 1 个时钟周期内输出高脉冲，其余为低电平
  assign o_pulse = value & ~seq;

endmodule

// 下一个是直接从一个域到另一个域的同步
// 用于本地使用 - 1 级触发器
module SYNC_S2C #(parameter WIDTH=1) ( 
  input             rst_n,
  input             clk,
  input  [WIDTH-1:0]scl_data,
  output [WIDTH-1:0]out_clk     // CLK 域内的输出副本
  );

  reg  [WIDTH-1:0]  clk_copy;

  assign out_clk = clk_copy;

  // 注意：可以使用 clk_copy ^ scl_data 作为测试以允许使用 ICG
  always @ (posedge clk or negedge rst_n)
    if (!rst_n)
      clk_copy <= {WIDTH{1'b0}};
    else 
      clk_copy <= scl_data;
endmodule

// 下一个是直接从一个域到另一个域的同步
// 用于本地使用 - 1 级触发器
module SYNC_C2S #(parameter WIDTH=1) ( 
  input             rst_n,
  input             scl,        // 也可以是 SCL_n
  input  [WIDTH-1:0]clk_data,
  output [WIDTH-1:0]out_scl     // CLK 域内的输出副本
  );

  reg   [WIDTH-1:0] scl_copy;

  assign out_scl = scl_copy;

  // 注意：可以使用 clk_copy ^ scl_data 作为测试以允许使用 ICG
  always @ (posedge scl or negedge rst_n)
    if (!rst_n)
      scl_copy <= {WIDTH{1'b0}};
    else 
      scl_copy <= clk_data;
endmodule

// 下一个是本地置位以及从 CLK 到 SCL 的异步清除
module SYNC_AClr_C2S(
  input CLK, // 系统域
  input RSTn,
  input local_set,
  input async_clear,
  output o_value);

  reg  value;                   // CLK 域
  always @ (posedge CLK or negedge RSTn)
    if (!RSTn)
      value <= 1'b0;
    else if (local_set) 
      value <= 1'b1;
    else if (async_clear) // CDC
      value <= 1'b0;
  assign o_value = value;

endmodule