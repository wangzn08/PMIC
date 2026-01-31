/*--------------------------------------------------------------------
  版权所有 (C) 2015-2019, NXP B.V.
  保留所有权利。

  此模块为 MIPI I3C 从设备复位检测器，可置于 GO2 或常开电源域中。
  用于检测 Slave Reset (SRST) 信号，可从深度睡眠中唤醒设备或进行系统复位。
--------------------------------------------------------------------*/

module i3c_reset_detector
  #(parameter USE_RST_FLOP=1)           // 避免复位信号的紧时序问题
  (
  // 此模块包含三个时钟域
  input         clk_SDA,                // SDA 上升沿作为复位检测时钟
  input         clk_SDA_n,              // SDA 下降沿作为时钟
  input         clk_SCL,                // SCL 上升沿作为时钟
  input         RSTn,                   // 全局复位（包括来自 oRstAll 的复位）
  // I3C 总线引脚及使能信号
  input         i3c_slave_active,       // I3C 作为从设备激活标志
  input         pin_SCL_in,             // SCL 引脚输入（非时钟）
  input         pin_SDA_in,             // SDA 引脚输入（非时钟）
  // 复位/唤醒输出信号
  input         iDeepestSleep,          // 可选：当前处于深度睡眠状态
  output        oWake,                  // 可选：从深度睡眠中唤醒
  output        oRstBlock,              // 仅复位 I3C 外设
  output        oRstAll,                // 复位 I3C 模块及整个系统
  output        oRstCustom,             // 客户自定义触发输出
  // 与主模块交互信号：iRstAction[2:0] 为动作码，[3] 为清除标志
  input   [3:0] iRstAction,             // [2:0]=RSTACT 请求类型，[3]=清除标志
  output        oRstRstAction,          // 复位动作恢复信号
  // 扫描测试控制
  input         scan_no_rst             // 禁止分层复位（用于扫描测试）
  );
`include "i3c_params.v"

  //==================================================================
  // 1. 信号定义与基本连接
  //==================================================================
  wire          enable_RSTn;            // 仅在从设备激活时的全局复位使能
  wire          scl_rst_n;              // SCL 复位信号（低有效）
  reg     [6:0] stp_cnt;                // HDR-Exit 计数器（移位链）
  reg           srst_act0, srst_act1, srst_act2; // 复位动作状态寄存器
  wire    [2:0] rst_action;             // 仅取动作码部分
  wire          rst_clear;              // 清除标志
  reg           rst_all;                // 系统复位触发锁存
  reg           srst_escalate;          // 复位升级标志
  wire          rst_escalate_n;         // 升级标志复位（低有效）
  wire          rst0_act_n, rst1_act_n, rst2_act_n; // 各阶段复位信号

  // 全局复位使能：仅在从设备激活或扫描测试时有效
  assign enable_RSTn = RSTn & (i3c_slave_active | scan_no_rst);

  // 动作解码逻辑
  assign rst_action   = iDeepestSleep ? `RACT_NONE : iRstAction[2:0];
  assign rst_clear    = iDeepestSleep ? 1'b0       : iRstAction[3];
  assign oWake        = srst_act2 & iDeepestSleep;               // 深度睡眠唤醒
  assign oRstBlock    = srst_act2 & (rst_action==`RACT_DEF) & ~oRstAll; // 仅复位外设
  assign oRstAll      = rst_all;                                 // 系统复位
  assign oRstRstAction= srst_act2;                               // 复位动作恢复
  assign oRstCustom   = srst_act2 & (rst_action==`RACT_CUST);    // 客户自定义复位

  //==================================================================
  // 2. 扩展退出检测器 -> 复位检测器（检测7个SDA下降沿）
  //==================================================================
  // 在SCL为低时检测连续的SDA下降沿，SCL变高则清零计数器
  always @ (posedge clk_SDA_n or negedge scl_rst_n)
    if (~scl_rst_n)
      stp_cnt <= 7'd0;                  // SCL为高或未使能时清零
    else 
      stp_cnt <= {stp_cnt[5:0], 1'b1};  // 移位链计数器

  generate if (USE_RST_FLOP) begin : rst_flop
    // 使用寄存器避免SCL上升沿的时序竞争
    reg rst_r;
    wire rst_reset_r = enable_RSTn & (|stp_cnt | scan_no_rst);
    always @ (posedge clk_SCL or negedge rst_reset_r)
      if (~rst_reset_r)
        rst_r <= 1'b1;
      else if (|stp_cnt)
        rst_r <= 1'b0;
    assign scl_rst_n = enable_RSTn & (rst_r | scan_no_rst);
    `Observe(observe_srstp_rst, clk_SDA_n, i3c_slave_active & rst_r) // 可选的DFT观测点
  end else begin
    // 直接组合逻辑复位（无寄存器）
    assign scl_rst_n = enable_RSTn & (~pin_SCL_in | scan_no_rst);
    `Observe(observe_srstp_rst, clk_SDA_n, i3c_slave_active & ~pin_SCL_in)
  end endgenerate

  //==================================================================
  // 3. SCL上升沿处理、Sr检测、P（STOP）触发
  //==================================================================
  // a) 检测到stp_cnt[6]=1且SDA为高时，在SCL上升沿锁存srst_act0
  assign rst0_act_n = enable_RSTn & (~(srst_act0 & (~pin_SCL_in | srst_act2)) | scan_no_rst);
  `Observe(observe_sract0_rst, clk_SCL, i3c_slave_active & (srst_act0 & (~pin_SCL_in | srst_act2)))
  always @ (posedge clk_SCL or negedge rst0_act_n)
    if (~rst0_act_n)
      srst_act0    <= 1'b0;
    else if (stp_cnt[6] & pin_SDA_in)   // SCL上升沿前SDA必须为高
      srst_act0    <= 1'b1;

  // b) 检测重复起始条件（Sr）：SCL为高时SDA下降沿，且srst_act0为高
  assign rst1_act_n = enable_RSTn & (~(srst_act1 & (~pin_SCL_in | pin_SDA_in)) | scan_no_rst);
  `Observe(observe_sract1_rst, clk_SDA_n, i3c_slave_active & (srst_act1 & (~pin_SCL_in | pin_SDA_in)))
  always @ (posedge clk_SDA_n or negedge rst1_act_n)
    if (~rst1_act_n)
      srst_act1     <= 1'b0;
    else if (srst_act0 & pin_SCL_in)
      srst_act1     <= 1'b1;

  // c) 检测停止条件（P）：SCL为高时SDA上升沿，且srst_act1为高
  assign  rst2_act_n = enable_RSTn & (~(srst_act2 & (~pin_SCL_in | rst_clear)) | scan_no_rst);
  `Observe(observe_sract2_rst, clk_SDA, i3c_slave_active & (srst_act2 & (~pin_SCL_in | rst_clear)))
  wire    final_match = srst_act1 & pin_SCL_in; // 完整的停止条件匹配
  always @ (posedge clk_SDA or negedge rst2_act_n)
    if (~rst2_act_n)
      srst_act2    <= 1'b0;
    else if (final_match)
      srst_act2    <= 1'b1;             // 触发复位动作

  //==================================================================
  // 4. 特殊情况处理：系统复位保持与升级逻辑
  //==================================================================
  // 系统复位锁存：仅由芯片复位或从设备激活脉冲清除
  `Observe(observe_srall_rst, clk_SDA, i3c_slave_active & (srst_act2 & (~pin_SCL_in | rst_clear)))
  always @ (posedge clk_SDA or negedge enable_RSTn)
    if (~enable_RSTn)
      rst_all <= 1'b0;
    else if (final_match)
      rst_all <= (rst_action==`RACT_FULL) | srst_escalate; // 仅当请求系统复位或已升级

  // 复位升级逻辑：当默认（仅外设）复位未解决时升级为系统复位
  assign rst_escalate_n = enable_RSTn & (~(srst_escalate & rst_clear) | scan_no_rst);
  `Observe(observe_srescal_rst, clk_SDA, i3c_slave_active & (srst_escalate & rst_clear))
  always @ (posedge clk_SDA or negedge rst_escalate_n)
    if (~rst_escalate_n)
      srst_escalate <= 1'b0;
    else if (final_match & (rst_action==`RACT_DEF))
      srst_escalate <= 1'b1;            // 设置升级标志，直到被清除

endmodule