// 设计信息
// 文件名称        : i3c_exit_detectors.v
// 描述            : MIPI I3C 组合检测器，用于 HDR 退出和重启检测
//                    此模块包含所有 I3C 从设备和主设备所需的退出检测器，
//                    以便在退出 HDR 模式前忽略 HDR 模式，无论是否支持 HDR 模式。
//                    仅在支持 HDR 时才检查重启。

module i3c_exit_detector #(
    parameter ENA_HDR = 3'b000 // 默认为无 HDR
  )
  (
  // 此小型模块中有 3 个时钟域（!）
  // 来自引脚的时钟源及其反相必须在上一层安全完成
  input         clk_SDA_n,              // 反相 SDA 作为时钟
  input         clk_SDA,                // 非反相 SDA 作为时钟
  input         clk_SCL,                // SCL 作为时钟
  input         RSTn,                   // 全局复位
  // 现在正常网络。注意分层复位控制
  input         pin_SCL_in,             // SCL 作为引脚（非时钟）
  input         in_HDR_mode,            // SDR 检测到 CCC ENTHDRn
  input         HDR_restart_ack,        // HDR 块发出的信号
  output        oHDR_exit,              // 为 1 表示检测到退出（在 SCL 上清除）
  output        oHDR_restart,           // 为 1 表示重启（在 ack 时清除）
  input         scan_no_rst             // 防止分层复位
  );
`include "i3c_params.v"                 // 局部参数/常量

  wire          scl_rst_n;
  reg [3:0]     stp_cnt;                // HDR STOP 计数器（移位链）

  // SCL 必须为 0 才能退出/重启，因此 SCL=1 会复位
  assign scl_rst_n     = RSTn & (~pin_SCL_in | scan_no_rst);

  // 当 SCL 为高时，3 和 4 次 SDA 上升沿的计数器
  //（否则 SCL 复位）。
  // 注意：我们允许此逻辑一直处于活动状态，
  // 即使不在 HDR 模式下。这是为了作为安全措施捕获错过的
  // HDR 进入情况。这意味着当 SDR SDA 变化时一个触发器会翻转。
  // 注意：由 SDA_n（SDA 下降沿）驱动
  always @ (posedge clk_SDA_n or negedge scl_rst_n)
    if (~scl_rst_n)
      stp_cnt <= 4'd0;                  // SCL 高或非 HDR
    else 
      stp_cnt <= {stp_cnt[2:0], 1'b1};  // 移位链计数器
  assign oHDR_exit = stp_cnt[3];        // 检测退出（STOP）

  // 
  // HDR 重启仅在此块中启用 HDR 支持时使用
  //
  generate if (ENA_HDR != 0) begin : HDR_restart 
    wire        restart_rst_n;
    reg         poss_restart;           // 可能的重启
    reg         is_restart;

    // 在退出过程的大部分完成之前不可能重启，但
    // 重启会被不在 HDR 模式和重启确认复位
    assign restart_rst_n = RSTn & 
             ((~HDR_restart_ack & in_HDR_mode)| scan_no_rst);
    `Observe(observe_restart_rst, clk_SCL, ~HDR_restart_ack & in_HDR_mode) // 可选的 DFT 观察点

    // 可能的重启意味着恰好 2 次 SDA 下降沿
    // 然后 SDA 上升沿。实际
    // 重启来自 SCL 然后上升沿
    // 注意 SDA 驱动（SDA 上升沿）
    always @ (posedge clk_SDA or negedge restart_rst_n)
      if (~restart_rst_n)
        poss_restart <= 1'b0;     // 重启 ACK 或非 HDR
      else if (stp_cnt[1] & ~stp_cnt[2]) // 仅在第二次下降后
        poss_restart <= 1'b1;     // 2 次下降后 SDA 上升
      else 
        poss_restart <= 1'b0;     // 否则不可能重启

    // 注意 SCL 驱动（SCL 上升沿）
    always @ (posedge clk_SCL or negedge restart_rst_n)
      if (~restart_rst_n)
        is_restart <= 1'b0;       // 重启 ACK 或非 HDR
      else if (poss_restart)
        is_restart <= 1'b1;       // SDA 上升后 SCL 上升
      else 
        is_restart <= 1'b0;       // 否则不是重启
    assign oHDR_restart = is_restart; // 发出重启信号，ACK 时清除
  end else begin : no_HDR
    // 如果无 HDR 支持，我们从不检查重启
    assign oHDR_restart = 1'b0;   
  end endgenerate

endmodule