/*--------------------------------------------------------------------
  版权所有 (C) 2015-2019, NXP B.V.
  保留所有权利。
  
  该 HDL 软件由版权持有者和贡献者“按原样”提供，不承诺任何明示或暗示的保证。
  在任何情况下，版权持有者或贡献者均不对任何直接、间接、附带、特别、惩戒性或
  后果性损害承担责任。
  
  原始源码地址：https://github.com/NXP/i3c-slave-design
  -------------------------------------------------------------------- */

//
//  ----------------------------------------------------------------------------
//                     设计信息
//  ----------------------------------------------------------------------------
//  文件      : i3c_slow_counters.v
//  组织      : MCO
//  IP 名称   : i3c_slow_counters (MIPI I3C 从机低速计数器)
//  描述      : 
//    该模块用于实现 I3C 从机的定时功能，包括：
//    1. 等待总线可用（Bus Available）以发起带内中断（IBI）。
//    2. 错误检测与超时处理（如 S0/S1 状态机挂死检测）。
//    3. 热加入（Hot Join）所需的 tIDLE 200us 等待。
//    该模块运行在低速时钟（Slow Clock）域。
//
//  ----------------------------------------------------------------------------
//                     实现细节
//  ----------------------------------------------------------------------------
//  计数模型：
//  1. 总线空闲检测（SCL 无变化且除读操作外处于 STOP 状态）。
//  2. 1us 计数器：基础计时单位，用于计算总线可用时间（Bus Available）。当 SCL 变化时复位。
//  3. 100us 计数模型：派生出 60us（S0/S1 结束检测）和 100us（读操作停顿检测）。
//  4. 200us 计数模型：用于热加入（Hot Join）。
//  ----------------------------------------------------------------------------

`include "i3c_params.v"                 // 包含本地参数/常量

module i3c_slow_counters #(
    parameter ENA_IBI_MR_HJ   = 0,      // 事件使能掩码：0 表示无事件
    parameter CLK_SLOW_BITS   = 6,      // 总线可用计数所需的位数
    parameter CLK_SLOW_MATCH  = 6'd47,  // 1us 计数值（例如 48MHz 时为 47）
    parameter CLK_SLOW_HJMUL  = 10'd1000,// 1ms 所需的匹配次数
    parameter ENA_TIMEC       = 6'b000010, 
    parameter TIMEC_FREQ_ACC  = {8'd24,8'd10},
    parameter ENA_CCC_HANDLING= 6'd0    
  )
  (
  input               RSTn,             // 系统复位
  input               CLK_SLOW,         // 低速时钟（用于 IBI 强制触发）
  output              slow_gate,        // 低速时钟门控标志（1 表示可关闭时钟以省电）
  input               clk_SCL,          // SCL 时钟
  input               clk_SCL_n,        // SCL 时钟反相
  input               cf_SlvEna,        // 从机总线处理使能信号
  input         [7:0] cf_BAMatch,       // 总线可用（Bus Available）匹配值
  input         [2:0] event_pending,    // 挂起事件：IBI, 主机请求(MR), 或 热加入(HJ)
  input               run_60,           // 运行 60us S0/S1 停顿检测
  input               run_100,          // 运行 100us 读挂死检测
  input               int_in_STOP,      // 同步后的 STOP 状态信号
  input               pin_SCL_in,       // SCL 引脚输入（用于状态检测）
  input               pin_SDA_in,       // SDA 引脚输入（用于状态检测）
  output              force_sda,        // 触发 IBI/MR/HJ 时拉低 SDA
  output              done_60,          // 60us 计时完成
  output              done_100,         // 100us 计时完成
  output              hold_engine       // 保持引擎直到 HJ 发出 START
  );

  // 内部信号定义
  wire                    run_cnt, run_hotjoin;
  wire                    is_1us;
  wire                    is_hj;
  reg [CLK_SLOW_BITS-1:0] microsec_cnt; // 微秒级计数器
  reg               [6:0] mid_cnt;      // 中级计数器（最大 127）
  reg               [1:0] hj_cnt;       // HJ 计数器
  reg                     hj_done;      // 标记上电后 HJ 是否已执行
  reg                     request_start, check_idle_n, check_idle;
  wire                    remote_check_idle, remote_check_idle_n;
  reg                     scl_check_idle, scl_check_idle_n;
  wire                    sclsda_state;
  wire                    safe_ibi_hj;

  // 运行控制逻辑
  assign run_cnt     = |event_pending[1:0] | run_60 | run_100 | run_hotjoin;
  assign run_hotjoin = &event_pending[1:0] & ENA_IBI_MR_HJ[`EV_HJ_b];
  
  // 确保 IBI 和 HJ 触发前 SCL 和 SDA 均为高电平（处于 STOP 状态）
  wire   is_stop     = int_in_STOP & (sclsda_state | force_sda); 
  assign safe_ibi_hj = (~|event_pending[1:0] | run_60 | run_100) | is_stop;
  
  // 门控逻辑：如果不需要计时且计数器已清零，则可关闭 CLK_SLOW
  assign slow_gate   = ~(run_cnt | (|microsec_cnt) | request_start | (|mid_cnt));
  
  // SDA 强制拉低逻辑：用于发起 IBI、MR 或 HJ
  assign force_sda   = request_start &
                       ((|event_pending[1:0] & ~&event_pending[1:0] & is_1us) |
                        is_hj);

  // 1us 匹配逻辑：支持常量匹配或 MMR 寄存器配置匹配
  generate if (ENA_IBI_MR_HJ[`EV_BAMATCH_b]) begin : ba_math
    assign is_1us = microsec_cnt == cf_BAMatch[CLK_SLOW_BITS-1:0];
  end else begin
    assign is_1us = microsec_cnt == CLK_SLOW_MATCH[CLK_SLOW_BITS-1:0];
  end endgenerate

  // 1us 基础计数器逻辑
  always @ (posedge CLK_SLOW or negedge RSTn)
    if (~RSTn) 
      microsec_cnt   <= {CLK_SLOW_BITS{1'b0}};
    else if (~request_start & |microsec_cnt)
      microsec_cnt   <= {CLK_SLOW_BITS{1'b0}};
    else if (request_start) 
      if (~|is_1us)
        microsec_cnt <= microsec_cnt + {{CLK_SLOW_BITS-1{1'b0}},1'b1};
      else if (run_60 | run_100 | (run_hotjoin & ~hj_done)) 
        microsec_cnt <= {CLK_SLOW_BITS{1'b0}}; // 循环计数模式

  // 计数器启动与总线空闲检测
  always @ (posedge CLK_SLOW or negedge RSTn)
    if (~RSTn) begin
      check_idle      <= 1'b0;
      check_idle_n    <= 1'b0;
      request_start   <= 1'b0;
    end else if (run_cnt & cf_SlvEna & safe_ibi_hj) begin
      // 检测 SCL 是否发生翻转。如果 check 信号与 remote 信号不再匹配，说明 SCL 发生了变化
      if ((check_idle   == remote_check_idle) | 
          (check_idle_n == remote_check_idle_n)) begin
        check_idle    <= ~remote_check_idle;   // 重新开始计时，标记 SCL 有变动
        check_idle_n  <= ~remote_check_idle_n; 
        request_start <= 1'b0;                 // 重新启动计数
      end else if (~request_start) begin
        request_start <= 1'b1;                 // 启动计时器
      end 
    end else begin
      check_idle      <= remote_check_idle; 
      check_idle_n    <= remote_check_idle_n;
      request_start   <= 1'b0;
    end

  // 跨时钟域（CDC）总线活动检测：用于确保在测量空闲时间时没有 SCL 时钟产生
  always @ (posedge clk_SCL or negedge RSTn)
    if (!RSTn)
      scl_check_idle <= 1'b0;
    else if (scl_check_idle ^ check_idle)  
      scl_check_idle <= ~scl_check_idle;   // 与 slow clock 域信号同步
  always @ (posedge clk_SCL_n or negedge RSTn)
    if (!RSTn)
      scl_check_idle_n <= 1'b0;
    else if (scl_check_idle_n ^ check_idle_n)
      scl_check_idle_n <= ~scl_check_idle_n; 

  // 同步器实例化：将 SCL 域信号同步回 CLK_SLOW 域
  SYNC_S2C #(.WIDTH(1)) sync_idle_check(.rst_n(RSTn), .clk(CLK_SLOW), .scl_data(scl_check_idle), 
                                 .out_clk(remote_check_idle));
  SYNC_S2C #(.WIDTH(1)) sync_idle_check_n(.rst_n(RSTn), .clk(CLK_SLOW), .scl_data(scl_check_idle_n), 
                                   .out_clk(remote_check_idle_n));
  SYNC_S2C #(.WIDTH(1)) sync_sclsda_state(.rst_n(RSTn), .clk(CLK_SLOW), .scl_data(pin_SCL_in&pin_SDA_in), 
                                   .out_clk(sclsda_state));

  // 100us 和 60us 计数器逻辑
  // 通过 1ms 的参数值换算得出
  localparam LOAD_100 = (CLK_SLOW_HJMUL+9)/10;  // 1ms 的 1/10
  localparam LOAD_60  = (CLK_SLOW_HJMUL+15)/16; // 1ms 的 1/16 (约为 60us)
  wire [0:0] bad_load;
  assign     bad_load[LOAD_100>127] = 1'b0; 

  always @ (posedge CLK_SLOW or negedge RSTn)
    if (~RSTn) 
      mid_cnt       <= 7'd0;
    else if (~run_60 & ~run_100 & ~run_hotjoin) begin
      if (|mid_cnt) mid_cnt <= 7'd0;
    end else begin
      if (~request_start)
        mid_cnt     <= 7'd0;                // 条件变化时重置
      else if (is_1us) begin
        if (run_60) begin
          if (mid_cnt != LOAD_60)
            mid_cnt <= mid_cnt + 7'd1;      // 增加至 60us
        end else if (mid_cnt != LOAD_100)
          mid_cnt   <= mid_cnt + 7'd1;      // 增加至 100us
        else if (run_hotjoin)
          mid_cnt   <= 7'd0;                // HJ 模式下循环计数
      end
    end
    
  assign done_100  = (run_100 & (mid_cnt==LOAD_100));
  assign done_60   = (run_60  & (mid_cnt==LOAD_60));

  // 热加入 (Hot Join) 专用逻辑
  // 协议要求上电后等待 200us (即 2 个 100us 周期)
  always @ (posedge CLK_SLOW or negedge RSTn)
    if (~RSTn) begin 
      hj_cnt       <= 2'd0;
      hj_done      <= 1'b0;                 // 仅在 POR（上电复位）后执行一次
    end else if (~run_hotjoin) begin
      if (|hj_cnt) hj_cnt <= 2'd0;
    end else if (~hj_done) begin
      if (~request_start)
        hj_cnt     <= 2'd0;
      else if (is_1us & (mid_cnt==LOAD_100)) begin
        if (hj_cnt != 2'd2) begin
          hj_cnt   <= hj_cnt + 2'd1;
          if (hj_cnt[0]) begin              // 完成 2 个周期
            hj_done  <= 1'b1;               // 标记已完成一次性等待
            hj_cnt   <= 2'd0; 
          end
        end
      end
    end

  // HJ 准备就绪信号
  assign is_hj = run_hotjoin & (hj_done & is_1us);

  // 阻塞引擎：在 HJ 未完成 200us 等待前，总线状态未知，需保持引擎挂起
  assign hold_engine = run_hotjoin & ~hj_done;

endmodule