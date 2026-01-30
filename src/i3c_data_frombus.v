// 设计信息
// 文件名称        : i3c_data_frombus.v
// 描述            : MIPI I3C 主设备入站数据缓冲/FIFO 模块（主设备->从设备）
//                    此模块支持数据缓冲模型以及 SCL 时钟域与系统（如 PCLK）时钟域之间的映射。
//                    用于“写”消息以及不由该块处理的命令。
//                    支持类似 FIFO 的映射，包括乒乓缓冲区（2 入口 FIFO）的基础。
//                    使用内部更深 FIFO 是可选的。外部 FIFO 支持隐含在浅乒乓版本中。
//                    此模块还处理来自总线的错误同步。

`include "i3c_params.v"                 // 局部参数/常量

module i3c_data_frombus #(
    parameter FIFO_TYPE       = 0,      // [1:0]==b01 表示内部 FIFO
    parameter ENA_FROMBUS_FIFO= 0,      // 来自总线的深度，2 的幂，从 2 开始
    parameter ANY_HDR         = 0       // 任何类型的 HDR 模式
  )
  (
  // 定义时钟和复位
  input               RSTn,             // 全局复位
  input               CLK,              // 系统时钟（总线或其他）
  input               SCL,              // 从 SDR 传递上来 - 不是 SCL_n
  input               SCL_n,            // 从 SDR 传递上来 - 与引擎使用的相同
  // 现在 CLK 域中的网络连接到内存映射寄存器或网络
  output              notify_fb_ready,  // 来自总线的字节已准备就绪
  output        [7:0] notify_fb_data,   // 字节数据
  input               notify_fb_ack,    // 字节已取走
  input               fb_flush,         // 希望我们清空 RX 缓冲区
  output        [3:0] set_fb_err,       // 奇偶校验错误、CRC、S0/S1 状态/中断
  output              set_fb_orun,      // 来自总线的溢出状态/中断
  input         [3:0] clear_fb_err,     // 由应用清除
  input               clear_fb_orun,    // 同上
  // 现在是与 FIFO 相关的信息（无论是否有 FIFO）
  output        [4:0] avail_byte_cnt,   // FIFO 中的字节计数
  output              avail_fb_empty,   // 真正为空且未触发
  input         [1:0] rx_trig,          // FIFO 触发器
  output              int_rx,           // RX 非空或触发
  // 现在 SCL_n 域的控制和缓冲区（它们拥有字节）
  output              fb_data_use,      // b1 表示 OK，b0 表示已满
  input         [7:0] fb_datab,         // 如果 done=1 且 use=1，则输出字节到系统
  input               fb_datab_done,    // 写入时，done 脉冲一个 clk_SCL_n 周期
  input               fb_datab_err,     // 与 done 同时设置，如果 i3c 奇偶校验错误
  input               fb_orun,          // 如果溢出则脉冲一个 clk_SCL_n 周期（数据进入时已满）
  input         [2:0] fb_ddr_errs,      // HDR 帧错误、奇偶校验错误、CRC 错误
  input               fb_s0s1_err,      // 如果在 S0 或 S1 错误保持状态，则保持为 1
  output              brk_s0s1_err,     // 在 scl 域中输出 1 以打破 S0/S1 错误保持
  input               scan_no_rst       // 防止分层复位
  );

  // 内容目录
  // 1. 缓冲区/FIFO 模型概述
  // 2. 支持用的寄存器和连线，包括字节缓冲区
  // 3. CLK 域中的缓冲区/FIFO 读取侧
  // 4. SCL 域中的缓冲区/FIFO 写入控制（非 SCL_n）
  // 5. 来自 SCL 域的错误同步握手

 //
 // 2 入口默认乒乓方案
 //
 generate if (FIFO_TYPE[`FIFO_INT_b]==0 || ENA_FROMBUS_FIFO==0) begin : fb_pingpong

  // 此块在 SCL 域中提供一个 2 入口 FIFO（乒乓缓冲区），用于来自主设备的数据。
  // 注意：这与引擎的字节缓冲区不同，因为（a）它需要按位访问（因此使用 FIFO 会产生多路选择器深度），
  // 并且（b）SCL 可能随时停止，因此我们希望从 CLK 域侧进行控制。
  // 2 入口 FIFO 允许非常简单的 CDC 边界，因为它减少了正常的二进制与格雷码问题。
  // 写入器（SCL 侧）和读取器（此 CLK 侧）的模型如下：
  // 1. SCL scl_widx（写入索引）是 2 位，比索引 FIFO 条目 [0] 和 [1] 所需的位数多一位。
  // 2. 系统侧 fb_ridx 同步到 scl_ridx，作为直接复制（因为是格雷码）。
  // 3. scl_widx（以及 scl_ridx）使用格雷码递增：00,01,11,10
  //    那么 FIFO 中的索引是 gray[1]^gray[0]，按预期给出 0,1,0,1。
  // 4. 空条件是 fb_widx==fb_ridx，其中 fb_ridx 是 scl_ridx 的同步版本。
  //    同样，空条件是 scl_widx==scl_ridx，其中 scl_ridx 是 fb_ridx 的同步版本。
  // 5. SCL 侧可以在为空或仅差一个（即未满）时推送。这由以下决定：
  //      scl_widx==scl_ridx 且 ^(scl_widx^scl_ridx)==1
  //    注意：widx 不能差 3 个，差 2 个即为满。SCL 侧使用 SCL 与 SCL_n 同步 fb_ridx，因此等待时间更短。
  // 6. 系统侧可以在非空时拉取。变为空的唯一方式是系统侧读取，因此很简单。系统侧同步 scl_widx。
  reg           [1:0] scl_widx;         // 寄存器：我们在 SCL 中的写入指针
  wire          [1:0] scl_ridx;         // 同步/寄存器：系统读取指针在 SCL 中
  wire          [1:0] fb_widx;          // 同步/寄存器：SCL 写入指针在系统域中
  reg           [1:0] fb_ridx;          // 寄存器：系统读取索引
  wire                scl_not_full;     // 未满时设置，因此可以推送
  wire                scl_empty;        // 空时设置
  wire                fb_empty;         // 系统侧的空检测
  wire                fb_1in;           // 特殊情况，返回深度
  reg           [1:0] next_widx, next_ridx; // 扩展格雷码的情况
  reg           [7:0] fbdata[0:1];      // 实际 FIFO
  wire          [8:0] opt_holding;
  wire                holding_data;

  // 现在同步 SCL 写入索引和 SCL 写入索引。
  // 格雷码，因此独立
  SYNC_S2C #(.WIDTH(2)) sync_scl_ridx(.rst_n(RSTn), .clk(CLK), .scl_data(scl_widx), 
                              .out_clk(fb_widx));
  SYNC_C2S #(.WIDTH(2)) sync_fb_widx(.rst_n(RSTn), .scl(SCL), .clk_data(fb_ridx), 
                             .out_scl(scl_ridx));

  //
  // 系统 CLK 域
  //

  // 管理索引
  always @ (posedge CLK or negedge RSTn)
    if (~RSTn)  
      fb_ridx      <= 2'b00;
    else if ((~opt_holding[8] | notify_fb_ack) & ~fb_empty) 
      fb_ridx      <= next_ridx;        // 当它们取走时前进
    else if (fb_flush) 
      fb_ridx      <= fb_widx;          // 匹配写入缓冲区以清空

  // 现在可选的保持缓冲区
  if (FIFO_TYPE[`FIFO_FBHOLD_b]) begin : fb_holding
    reg         [8:0] holding;          // 保持缓冲区
    // 如果它们需要，使用保持缓冲区。注意下面的逻辑与 fb_ridx 匹配。
    always @ (posedge CLK or negedge RSTn)
      if (~RSTn)
        holding      <= 9'd0;
      else if ((~holding[8] | notify_fb_ack) & ~fb_empty) begin
        holding[7:0] <= fbdata[^fb_ridx];
        holding[8]   <= 1'b1;
      end else if (notify_fb_ack) 
        holding[8]   <= 1'b0;
      else if (fb_flush) 
        holding[8]   <= 1'b0;

    assign opt_holding = holding;
    assign holding_data    = holding[8];
    assign avail_fb_empty  = fb_empty & ~holding[8];
  end else begin
    assign opt_holding = {~fb_empty, fbdata[^fb_ridx]};
    assign avail_fb_empty  = fb_empty;
    assign holding_data    = 1'b0;
  end

  assign notify_fb_ready = opt_holding[8];
  assign int_rx          = &rx_trig ? (~fb_empty & ~fb_1in) : notify_fb_ready;
  assign fb_empty        = fb_widx==fb_ridx;
  assign fb_1in          = ^(fb_widx ^ fb_ridx); // 未满，但有 1 个数据
  assign avail_byte_cnt  = fb_empty ? (holding_data ? 5'd1 : 5'd0) :
                           fb_1in   ? (holding_data ? 5'd2 : 5'd1) : // FIFO 中有 1 个
                                       holding_data ? 5'd3 : 5'd2;   // FIFO 中有 2 个
  assign notify_fb_data  = opt_holding[7:0];

  // 格雷码
  always @ ( * ) 
    case(fb_ridx)
    2'b00: next_ridx = 2'b01;
    2'b01: next_ridx = 2'b11;
    2'b11: next_ridx = 2'b10;
    2'b10: next_ridx = 2'b00;
    // 无默认，因为完整
    endcase

  //
  // SCL 时钟域（非 SCL_n）用于 CDC
  //

  assign scl_empty   = scl_widx==scl_ridx;
  assign scl_not_full= scl_empty | ^(scl_widx ^ scl_ridx);

  // 写入指针
  // 注意在 SCL 中而非 SCL_n - 可用更早
  always @ (posedge SCL or negedge RSTn)
    if (!RSTn) 
      scl_widx         <= 2'b00;
    else if (fb_datab_done & scl_not_full)
      // 可以安全推送下一个值。注意它们同步 scl_widx，因此位变化无风险（它们看到 widx 在保持稳定后）
      scl_widx         <= next_widx;

  // FIFO 复位用于清洁
  always @ (posedge SCL or negedge RSTn)
    if (~RSTn) begin
      fbdata[0]         <= 8'd0;
      fbdata[1]         <= 8'd0;
    end else if (fb_datab_done & scl_not_full)
      fbdata[^scl_widx] <= fb_datab;

  // 我们告诉它们是否已消耗它们的缓冲区，因此释放
  assign fb_data_use = scl_not_full;

  // 格雷码
  always @ ( * ) 
    case(scl_widx)
    2'b00: next_widx = 2'b01;
    2'b01: next_widx = 2'b11;
    2'b11: next_widx = 2'b10;
    2'b10: next_widx = 2'b00;
    // 无默认，因为完整
    endcase

 //
 // 内部 FIFO
 //
 end else begin : fb_fifo_inst
  // 实例化 FIFO
  i3c_internal_fb_fifo #(.BITS(ENA_FROMBUS_FIFO)) 
    fb_fifo(.RSTn(RSTn), .CLK(CLK), .SCL(SCL), .SCL_n(SCL_n), 
            .notify_fb_ready(notify_fb_ready), .notify_fb_data(notify_fb_data), 
            .notify_fb_ack(notify_fb_ack), .fb_flush(fb_flush), 
            .avail_fb_empty(avail_fb_empty), .avail_byte_cnt(avail_byte_cnt), 
            .rx_trig(rx_trig), .int_rx(int_rx), .fb_data_use(fb_data_use), 
            .fb_datab(fb_datab), .fb_datab_done(fb_datab_done), 
            .scan_no_rst(scan_no_rst));
 end endgenerate

  //
  // 使用 SCL 同步错误
  //

  // 来自总线的溢出 - 我们未足够快地排空
  SYNC_2PH_S2C_STATE synch_fb_orun(.rst_n(RSTn), .scl(SCL), .clk(CLK), 
                                   .trig_scl(fb_orun), 
                                   .out_clk(set_fb_orun), .clear_clk(clear_fb_orun));

  //
  // 使用 SCL_n 同步错误
  //

  // 来自总线的奇偶校验错误 - 第 9 位错误。
  // 必须使用 SCL_n，因为奇偶校验在 SCL 上升沿到达
  SYNC_2PH_S2C_STATE synch_fb_err0(.rst_n(RSTn), .scl(SCL_n), .clk(CLK), 
                                   .trig_scl(fb_datab_done & fb_datab_err), 
                                   .out_clk(set_fb_err[0]), .clear_clk(clear_fb_err[0]));
  generate if (ANY_HDR) begin : hdr_err
    // 如果启用了 DDR，则 DDR 奇偶校验和帧错误
    SYNC_2PH_S2C_STATE synch_fb_err1(.rst_n(RSTn), .scl(SCL_n), .clk(CLK), 
                                     .trig_scl(|fb_ddr_errs[1:0]), // 帧错误和奇偶校验错误
                                     .out_clk(set_fb_err[1]), .clear_clk(clear_fb_err[1]));
    SYNC_2PH_S2C_STATE synch_fb_err2(.rst_n(RSTn), .scl(SCL), .clk(CLK), 
                                     .trig_scl(fb_ddr_errs[2]), // CRC 错误
                                     .out_clk(set_fb_err[2]), .clear_clk(clear_fb_err[2]));
  end else begin
    assign set_fb_err[2:1] = 2'b00;
  end endgenerate

  // S0 和 S1 是电平保持，因此我们仅同步到 PCLK 域
    // 中断是复位控制，因此我们不必同步
  assign brk_s0s1_err = clear_fb_err[3]; // 复位，因此不同步
    // 直接映射
  SYNC_S2C sync_s0s1(.rst_n(RSTn), .clk(CLK), .scl_data(fb_s0s1_err), 
                     .out_clk(set_fb_err[3]));
endmodule