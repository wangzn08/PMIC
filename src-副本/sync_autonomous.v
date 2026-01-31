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
  1. “软件”与“HDL 软件”可互换使用。
  2. “二进制”包括 FPGA、仿真和物理形式（如硅芯片）。
  3. 第 2 条允许在网页或其他不属于分发部分的电子表单上发布此类通知。
  原始 BSD 源码获取地址：
    https://github.com/NXP/i3c-slave-design
  -------------------------------------------------------------------- */

//
//  ----------------------------------------------------------------------------
//                     设计信息
//  ----------------------------------------------------------------------------
//  文件      : sync_autonomous.v
//  组织      : MCO
//  标签      : 1.1.11
//  日期      : $Date: Wed Jun 12 23:47:03 2019 $
//  版本      : $Revision: 1.61 $
//
//  IP 名称   : 用于 Autonomous 封装的 SYNC_ 模块
//  描述      : 针对 Autonomous 的跨时钟域支持
//    此文件包含了专门为 autonomous 寄存器所需的跨时钟域（CDC）同步支持。
//    基本上仅包含 4 相握手逻辑。
//
//  ----------------------------------------------------------------------------
//                     修订历史
//  ----------------------------------------------------------------------------
//
//
//  ----------------------------------------------------------------------------
//                     实现细节
//  ----------------------------------------------------------------------------
//  详见微架构规范和 MIPI I3C 规范。
//
//    此模块通过命名块支持跨时钟域处理，以便 Spyglass（或同类工具）识别同步器。
//    对于大多数现代工艺，单触发器模型已经足够，尽管对于旧工艺可能需要通过
//    约束更改触发器类型（例如同步触发器，其具有更快的“重力”稳定速度）。
//    在常规情况下 1 个触发器就足够了，因为在低速路径下，Q 输出的亚稳态噪声
//    持续时间足够短。也就是说，大约有 ~40ns 的最小时间来完成稳定并到达另一端。
//  ----------------------------------------------------------------------------


// 命名说明：SYNC_= 同步，2PH_= 2相，S2C_= SCL 到 CLK，STATE= 带清除输入的内部状态
//           LVL_= 电平输入（相对于脉冲），LVLH_= 高电平有效
// 其他命名：ASet= 异步置位，本地清除；AClr= 本地置位，异步清除
//           ASelfClr= 本地置位并自动清除
//           Seq2= 2 级触发器序列器，确保在本地时钟域获得 1 个脉冲宽度


// 模块：SYNC_AClr_S2C
// 说明：本地置位，异步清除（SCL 到 CLK）
module SYNC_AClr_S2C(
  input SCL, // 可能是 SCL_n 
  input RSTn,
  input local_set,
  input async_clear,
  output o_value);

  reg  value;                   // SCL 时钟域
  always @ (posedge SCL or negedge RSTn)
    if (!RSTn)
      value <= 1'b0;
    else if (local_set) 
      value <= 1'b1;
    else if (async_clear) // CDC 信号
      value <= 1'b0;
  assign o_value = value;

endmodule

/* // 下一个是本地置位并从 CLK 到 SCL 异步清除
module SYNC_AClr_C2S(
  input CLK, // 系统时钟域
  input RSTn,
  input local_set,
  input async_clear,
  output o_value);

  reg  value;                   // CLK 时钟域
  always @ (posedge CLK or negedge RSTn)
    if (!RSTn)
      value <= 1'b0;
    else if (local_set) 
      value <= 1'b1;
    else if (async_clear) // CDC 信号
      value <= 1'b0;
  assign o_value = value;
endmodule
*/

// 下一个是本地置位并自动异步清除，
// 从 SCL 到 CLK，带有用于输出 1 个脉冲的序列器
module SYNC_ASelfClr_LVL_S2C_Seq2(
  input SCL, // 可能是 SCL_n 
  input CLK, // 系统时钟
  input RSTn,
  input local_set,
  input local_hold,
  output o_pulse);

  reg  value;                   // SCL 时钟域
  reg  clear, seq;              // CLK 时钟域
  always @ (posedge SCL or negedge RSTn)
    if (!RSTn)
      value <= 1'b0;
    else if (local_set) 
      value <= 1'b1;
    else if (clear)  // CDC (反馈清除)
      value <= 1'b0;

  always @ (posedge CLK or negedge RSTn)
    if (!RSTn)
      clear <= 1'b0;
    else if (value | local_hold)
      clear <= 1'b1; // CDC (握手阶段)
    else
      clear <= 1'b0; // 第 4 阶段拉低信号
  always @ (posedge CLK or negedge RSTn)
    if (!RSTn)
      seq <= 1'b0;
    else if (clear)
      seq <= 1'b1; 
    else
      seq <= 1'b0; 
  // 以下产生 1 个时钟周期的高脉冲，其余时间为低
  assign o_pulse = clear & ~seq;
endmodule

// 下一个是本地置位并自动异步清除，
// 从 SCL 到 CLK，带有用于输出 1 个脉冲的序列器
module SYNC_ASelfClr_S2C_Seq2(
  input SCL, // 可能是 SCL_n 
  input CLK, // 系统时钟
  input RSTn,
  input local_set,
  output o_pulse);

  reg  value;                   // SCL 时钟域
  reg  clear, seq;              // CLK 时钟域
  always @ (posedge SCL or negedge RSTn)
    if (!RSTn)
      value <= 1'b0;
    else if (local_set) 
      value <= 1'b1;
    else if (clear)  // CDC (反馈清除)
      value <= 1'b0;

  always @ (posedge CLK or negedge RSTn)
    if (!RSTn)
      clear <= 1'b0;
    else if (value)
      clear <= 1'b1; // CDC (握手阶段)
    else
      clear <= 1'b0; // 第 4 阶段拉低信号
  always @ (posedge CLK or negedge RSTn)
    if (!RSTn)
      seq <= 1'b0;
    else if (clear)
      seq <= 1'b1; 
    else
      seq <= 1'b0; 
  // 以下产生 1 个时钟周期的高脉冲，其余时间为低
  assign o_pulse = clear & ~seq;
endmodule

// 下一个是本地置位并自动异步清除，
// 从 CLK 到 SCL，带有用于输出 1 个脉冲的序列器
module SYNC_ASelfClr_C2S_Seq2(
  input CLK, // 系统时钟
  input SCL, // 可能是 SCL_n 
  input RSTn,
  input local_set,
  output o_pulse);

  reg  value;                   // CLK 时钟域
  reg  clear, seq;              // SCL 时钟域
  always @ (posedge CLK or negedge RSTn)
    if (!RSTn)
      value <= 1'b0;
    else if (local_set) 
      value <= 1'b1;
    else if (clear)  // CDC (反馈清除)
      value <= 1'b0;

  always @ (posedge SCL or negedge RSTn)
    if (!RSTn)
      clear <= 1'b0;
    else if (value)
      clear <= 1'b1; // CDC (握手阶段)
    else
      clear <= 1'b0; // 第 4 阶段拉低信号
  always @ (posedge SCL or negedge RSTn)
    if (!RSTn)
      seq <= 1'b0;
    else if (clear)
      seq <= 1'b1; 
    else
      seq <= 1'b0; 
  // 以下产生 1 个时钟周期的高脉冲，其余时间为低
  assign o_pulse = clear & ~seq;
endmodule

// 下一个是直接从一个域同步到另一个域
// 用于本地使用 - 1 级触发器
module SYNC_S2B #(parameter WIDTH=1) ( 
  input             rst_n,
  input             clk,
  input  [WIDTH-1:0]scl_data,
  output [WIDTH-1:0]out_clk     // CLK 域中的输出副本
  );

  reg  [WIDTH-1:0]  clk_copy;

  assign out_clk = clk_copy;

  // 注：可以使用 clk_copy^scl_data 作为测试来允许使用集成门控时钟 (ICG)
  always @ (posedge clk or negedge rst_n)
    if (!rst_n)
      clk_copy <= {WIDTH{1'b0}};
    else 
      clk_copy <= scl_data;
endmodule