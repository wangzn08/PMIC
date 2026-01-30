/*--------------------------------------------------------------------
  Copyright (C) 2015-2019, NXP B.V.
  All rights reserved.
  
  Redistribution and use in source and binary forms, with or without 
  modification, are permitted provided that the following conditions 
  are met:
  
  1. Redistributions of source code must retain the above copyright 
     notice, this list of conditions and the following disclaimer.
  
  2. Redistributions in binary form must reproduce the above copyright 
     notice, this list of conditions and the following disclaimer in
     the documentation and/or other materials provided with the 
     distribution.
  
  3. Neither the name of the copyright holder nor the names of its 
     contributors may be used to endorse or promote products derived 
     from this HDL software without specific prior written permission.
  
  THIS (HDL) SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
  CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, 
  INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR 
  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
  NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR 
  OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, 
  EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

  NOTE: No license under any third-party patent is granted or implied. 
        It is the responsibility of the licensee to obtain any required 
        third-party patent licenses.

  Note on terms used above:
  1. Software and HDL software are used interchangeably.
  2. Binary includes FPGA, simulation, and physical forms such 
     as Silicon chips.
  3. Clause 2 allows for such notice on a Web page or other 
     electronic form not part of a distribution.
  The original BSD source is available from:
    https://github.com/NXP/i3c-slave-design
  -------------------------------------------------------------------- */

//
//  ----------------------------------------------------------------------------
//                    Design Information
//  ----------------------------------------------------------------------------
//  File            : sync_autonomous.v
//  Organisation    : MCO
//  Tag             : 1.1.11
//  Date            : $Date: Wed Jun 12 23:47:03 2019 $
//  Revision        : $Revision: 1.61 $
//
//  IP Name         : SYNC_ modules for Autonomous wrapper use
//  Description     : Clock crossing support for Autonomous
//    This contains support for cross-clock-domain (CDC) synchronization
//    as needed specifically for autonomous regs. This is basically
//    only 4-phase handshakes
//
//  ----------------------------------------------------------------------------
//                    Revision History
//  ----------------------------------------------------------------------------
//
//
//  ----------------------------------------------------------------------------
//                    Implementation details
//  ----------------------------------------------------------------------------
//  See the micro-arch spec and MIPI I3C spec
//
//    This suppports the Clock Domain Crossing using named blocks
//    so Spyglass (or equiv) can be told that the sync is here.
//    For most modern process, the single flop model is fine, although
//    the flop type may need to be changed by constraint for older
//    process (e.g. a sync flop, which has a "gravity" to settle
//    faster). 
//    1 flop is fine in normal cases because the Q out metastable
//    noise will be short enough in time for the paths used at the 
//    lower speeds. That is, there is ~40ns minimum to both settle
//    and arrive settled at the other end. 
//  ----------------------------------------------------------------------------


// Note naming: SYNC_= synchronizing, 2PH_=2-phase, S2C_=SCL to CLK, STATE=state with clr in
//              LVL_=level vs. pulse input, LVLH_=level high
// Other naming: ASet=async set, local clear, AClr=local set, async clear
//               ASelfClr=local set and auto clear
//               Seq2=2 flop sequencer to ensure we get 1 pulse in local domain


// 模块：SYNC_AClr_S2C
// 说明：
//  该模块在SCL时钟域内保存一个标志位，当本地在SCL域产生
//  `local_set` 时将标志置位；当来自另一个时钟域（例如CLK）的
//  `async_clear` 信号指示清除时，将该标志清零。输出 `o_value`
//  为SCL域中的当前标志值。此模块用于简单的跨时钟域清除
//  场景（SCL -> CLK 清除路径由CDC处理）。
//  端口：
//   - SCL: 本模块所在的时钟域（通常是SCL）
//   - RSTn: 异步复位
//   - local_set: 在SCL域由本地产生的置位信号
//   - async_clear: 来自其它域的清除请求（通过CDC传递）
//   - o_value: 当前标志输出（SCL域）
module SYNC_AClr_S2C(
  input SCL, // may be SCL_n 
  input RSTn,
  input local_set,
  input async_clear,
  output o_value);

  reg  value;                   // SCL domain
  always @ (posedge SCL or negedge RSTn)
    if (!RSTn)
      value <= 1'b0;
    else if (local_set) 
      value <= 1'b1;
    else if (async_clear) // CDC
      value <= 1'b0;
  assign o_value = value;

endmodule

/* moved to normal sync for slave
// Next is set local and clear async from CLK to SCL
module SYNC_AClr_C2S(
  input CLK, // system domain
  input RSTn,
  input local_set,
  input async_clear,
  output o_value);

  reg  value;                   // CLK domain
  always @ (posedge CLK or negedge RSTn)
    if (!RSTn)
      value <= 1'b0;
    else if (local_set) 
      value <= 1'b1;
    else if (async_clear) // CDC
      value <= 1'b0;
  assign o_value = value;
endmodule
*/

// 模块：SYNC_ASelfClr_LVL_S2C_Seq2
// 说明：
//  该模块实现从SCL域到CLK域的4相握手：在SCL域发生`local_set`
//  时置位内部寄存器；在CLK域检测到该置位或`local_hold`时
//  产生清除信号（`clear`），并通过简单的两级序列（`seq`）
//  将清除脉冲限定为在CLK域内只有一个时钟周期的高脉冲，
//  以便将SCL域的事件转换为在CLK域单周期脉冲（`o_pulse`）。
//  端口：
//   - SCL: 事件发生的时钟域
//   - CLK: 输出脉冲所在的时钟域（系统时钟）
//   - RSTn: 异步复位
//   - local_set: SCL域置位请求
//   - local_hold: 在CLK域保持clear的条件（可选，用于延长握手）
//   - o_pulse: 在CLK域输出的单周期脉冲
//  用途：将SCL域的level信号转换为CLK域的单周期脉冲，应对CDC
//  场景，保证在CLK域仅产生一次脉冲响应。
// Next is set local and auto clear async, 
// from SCL to CLK with sequence for 1 pulse out
module SYNC_ASelfClr_LVL_S2C_Seq2(
  input SCL, // may be SCL_n 
  input CLK, // system clock
  input RSTn,
  input local_set,
  input local_hold,
  output o_pulse);

  reg  value;                   // SCL domain
  reg  clear, seq;              // CLK domain
  always @ (posedge SCL or negedge RSTn)
    if (!RSTn)
      value <= 1'b0;
    else if (local_set) 
      value <= 1'b1;
    else if (clear)  // CDC
      value <= 1'b0;

  always @ (posedge CLK or negedge RSTn)
    if (!RSTn)
      clear <= 1'b0;
    else if (value | local_hold)
      clear <= 1'b1; // CDC (handshake)
    else
      clear <= 1'b0; // 4th phase going down
  always @ (posedge CLK or negedge RSTn)
    if (!RSTn)
      seq <= 1'b0;
    else if (clear)
      seq <= 1'b1; 
    else
      seq <= 1'b0; 
  // below pulses for 1 clock high, else is low
  assign o_pulse = clear & ~seq;
endmodule

// 模块：SYNC_ASelfClr_S2C_Seq2
// 说明：
//  与 `SYNC_ASelfClr_LVL_S2C_Seq2` 类似，但不带 `local_hold`，
//  用于将SCL域的置位事件转换为CLK域的单周期脉冲输出 `o_pulse`。
//  典型用途是将来自SCL域的瞬时事件安全传递到CLK域执行一次性操作。
//  端口说明同上（移除了 local_hold）。
// Next is set local and auto clear async, 
// from SCL to CLK with sequence for 1 pulse out
module SYNC_ASelfClr_S2C_Seq2(
  input SCL, // may be SCL_n 
  input CLK, // system clock
  input RSTn,
  input local_set,
  output o_pulse);

  reg  value;                   // SCL domain
  reg  clear, seq;              // CLK domain
  always @ (posedge SCL or negedge RSTn)
    if (!RSTn)
      value <= 1'b0;
    else if (local_set) 
      value <= 1'b1;
    else if (clear)  // CDC
      value <= 1'b0;

  always @ (posedge CLK or negedge RSTn)
    if (!RSTn)
      clear <= 1'b0;
    else if (value)
      clear <= 1'b1; // CDC (handshake)
    else
      clear <= 1'b0; // 4th phase going down
  always @ (posedge CLK or negedge RSTn)
    if (!RSTn)
      seq <= 1'b0;
    else if (clear)
      seq <= 1'b1; 
    else
      seq <= 1'b0; 
  // below pulses for 1 clock high, else is low
  assign o_pulse = clear & ~seq;
endmodule

// 模块：SYNC_ASelfClr_C2S_Seq2
// 说明：
//  该模块为 `CLK` 域 -> `SCL` 域的对称实现：在CLK域产生 `local_set`
//  时置位内部寄存器；SCL域检测到置位后产生 `clear` 并通过两级
//  序列 `seq` 将其收窄为单周期脉冲，输出到SCL域的 `o_pulse`。
//  用于将系统时钟域的事件安全地通知到SCL域。
// Next is set local and auto clear async, 
// from CLK to SCL with sequence for 1 pulse out
module SYNC_ASelfClr_C2S_Seq2(
  input CLK, // system clock
  input SCL, // may be SCL_n 
  input RSTn,
  input local_set,
  output o_pulse);

  reg  value;                   // CLK domain
  reg  clear, seq;              // SCL domain
  always @ (posedge CLK or negedge RSTn)
    if (!RSTn)
      value <= 1'b0;
    else if (local_set) 
      value <= 1'b1;
    else if (clear)  // CDC
      value <= 1'b0;

  always @ (posedge SCL or negedge RSTn)
    if (!RSTn)
      clear <= 1'b0;
    else if (value)
      clear <= 1'b1; // CDC (handshake)
    else
      clear <= 1'b0; // 4th phase going down
  always @ (posedge SCL or negedge RSTn)
    if (!RSTn)
      seq <= 1'b0;
    else if (clear)
      seq <= 1'b1; 
    else
      seq <= 1'b0; 
  // below pulses for 1 clock high, else is low
  assign o_pulse = clear & ~seq;
endmodule

// 模块：SYNC_S2B
// 说明：
//  简单的宽度为 `WIDTH` 的跨域复制器（1个寄存器），将来自 `scl_data`
//  的数据在上升沿 `clk` 时采样并写入 `out_clk`。这是最简单的跨域
//  复制器，适用于延迟/时序要求不严格的场合。对于更高可靠性
//  的CDC，应使用多级同步或专门的同步器模块。
//  端口：
//   - rst_n: 复位
//   - clk: 目标时钟域（out_clk 所在域）
//   - scl_data: 源时钟域的数据输入（宽度 WIDTH）
//   - out_clk: 在 clk 域输出的数据副本
// next is straight sync from one domain to the other
// for local use - 1 flop
module SYNC_S2B #(parameter WIDTH=1) ( 
  input             rst_n,
  input             clk,
  input  [WIDTH-1:0]scl_data,
  output [WIDTH-1:0]out_clk     // output copy in CLK domain
  );

  reg  [WIDTH-1:0]  clk_copy;

  assign out_clk = clk_copy;

  // note: could use clk_copy^scl_data as test to allow ICG
  always @ (posedge clk or negedge rst_n)
    if (!rst_n)
      clk_copy <= {WIDTH{1'b0}};
    else 
      clk_copy <= scl_data;
endmodule

