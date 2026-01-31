// 设计信息
// 文件名称        : i3c_data_tobus.v
// 描述            : MIPI I3C 主设备读取出站数据缓冲/FIFO 模块（从设备->主设备）
//                    此模块支持数据缓冲模型以及 SCL 时钟域与系统（如 PCLK）时钟域之间的映射。
//                    支持类似 FIFO 的映射，包括乒乓缓冲区（2 入口 FIFO）的基础。
//                    使用内部更深 FIFO 是可选的。外部 FIFO 支持隐含在浅乒乓版本中。
//                    此模块还处理总线传输期间的错误同步。

`include "i3c_params.v"                 // 局部参数/常量

module i3c_data_tobus #(
    parameter FIFO_TYPE       = 0,      // [1:0]==b01 表示内部 FIFO
    parameter EXT_FIFO        = 0,      // 仅用于外部 FIFO 类型
    parameter ENA_TOBUS_FIFO  = 0       // 到总线的深度，2 的幂，从 2 开始
  )
  (
  // 定义时钟和复位
  input               RSTn,             // 全局复位
  input               CLK,              // 系统时钟（总线或其他）
  input               SCL,              // 从 SDR 传递上来 - 不是 SCL_n
  input               SCL_n,            // 从 SDR 传递上来 - SCL_n
  // 现在 CLK 域中的网络连接到内存映射寄存器或网络
  input               avail_tb_ready,   // 到总线的字节已准备就绪
  input         [7:0] avail_tb_data,    // 字节数据
  output              avail_tb_ack,     // 我们已使用字节
  input               avail_tb_end,     // 最后一个字节 - 与 ready 同时
  input               tb_flush,         // 1 周期脉冲以刷新缓冲区
  output              avail_tb_full,    // 如果满则为 1（FIFO 触发与否）
  // 下一个是 FIFO 计数，即使没有 FIFO
  output        [4:0] avail_byte_cnt,   // FIFO 中的字节计数
  input         [1:0] tx_trig,          // 如果使用 FIFO，则为 FIFO 触发水平
  output              int_tb,           // 当 TX 未满或 <=trig 时设置
  // 下一个是错误同步的设置和清除
  output              set_tb_urun_nack, // 头部欠载（因此返回 NACK）
  output              set_tb_urun,      // 数据欠载
  output              set_tb_term,      // 主设备终止读取
  input               clear_tb_urun_nack,
  input               clear_tb_urun,
  input               clear_tb_term,
  // 现在 SCL_n 域中的网络
  output              tb_data_valid,    // b1 表示满，b0 表示空
  output        [7:0] tb_datab,         // 如果有效，供它们使用的数据
  output              tb_end,           // 保持直到数据被消耗
  input               tb_datab_ack,     // 消耗时的确认，持续 1 个 SCL_n 时钟
  input               tb_urun_nack,     // 如果因无数据而 NACK，则脉冲 1 个 SCL_n
  input               tb_urun,          // 欠载且非结束时的脉冲，持续 1 个 SCL_n
  input               tb_term,          // 当主设备终止读取时的脉冲，持续 1 个 SCL_n
  input               scan_no_rst       // 防止分层复位
  );

  // 内容目录
  // 1. 缓冲区/FIFO 模型概述
  // 2. 支持用的寄存器和连线
  // 3. CLK 域中的缓冲区/FIFO 写入控制
  // 4. SCL 域中的缓冲区/FIFO 读取（非 SCL_n）
  // 5. 来自 SCL 域的错误同步握手

 //
 // 2 入口默认乒乓方案
 //
 generate if (FIFO_TYPE[`FIFO_INT_b]==0 || ENA_TOBUS_FIFO==0) begin : tb_pingpong

  //
  // 此块为 SCL 域提供一个 2 入口 FIFO（乒乓缓冲区），用于发送给主设备的数据。
  // 2 入口 FIFO 允许非常简单的 CDC 边界，因为它减少了正常的二进制与格雷码问题。
  // 此外，由于 SCL 可能突然停止，所有工作都在系统侧完成。
  // 写入器（此块）和读取器（SCL 侧）的模型如下：
  // 1. 系统 tb_widx（写入索引）是 2 位，比索引 FIFO 条目 [0] 和 [1] 所需的位数多一位。
  // 2. SCL scl_ridx 同步到 tb_ridx，作为直接复制（因为是格雷码）。
  // 3. tb_widx（以及 tb_ridx）使用格雷码递增：00,01,11,10
  //    那么 FIFO 中的索引是 gray[1]^gray[0]，按预期给出 0,1,0,1。
  // 4. 空条件是 tb_widx==tb_ridx，其中 tb_ridx 是 scl_ridx 的同步版本。
  //    同样，空条件是 scl_widx==scl_ridx，其中 scl_widx 是 tb_widx 的同步版本。
  // 5. 系统侧可以在为空或仅差一个（即未满）时推送。这由以下决定：
  //      tb_widx==tb_ridx 且 ^(tb_widx^tb_ridx)==1
  //    注意：widx 不能差 3 个，差 2 个即为满。
  // 6. SCL 侧可以在非空时拉取。变为空的唯一方式是 SCL 侧读取，因此很简单。
  //    SCL 侧在 SCL 与 SCL_n 上同步 tb_widx，因此安全。
  reg           [1:0] tb_widx;          // 寄存器：系统写入指针
  wire          [1:0] tb_ridx;          // 同步/寄存器：SCL 读取指针
  wire          [1:0] scl_widx;         // 同步/寄存器：系统写入指针在 SCL 域中
  reg           [1:0] scl_ridx;         // 寄存器：SCL 读取索引
  wire                tb_not_full;      // 未满时设置，因此可以推送
  wire                tb_empty;         // 空时设置
  wire                tb_wptr;          // 从 tb_widx 指向 FIFO 的指针
  wire                scl_rptr;         // 从 scl_ridx 指向 FIFO 的指针
  wire                scl_empty;        // SCL 侧的空检测
  reg                 ack_push;         // 1 周期脉冲。这种方式最安全
  reg           [1:0] next_widx, next_ridx; // 扩展格雷码的情况
    // 现在 FIFO 缓冲区 - 用于从外部 FIFO 或寄存器拉取
  reg           [8:0] tbfifo[0:1];     // 额外位用于结束标记

  // 同步它们的读取索引和我们的写入索引。
  // 格雷码，因此独立
  SYNC_S2C #(.WIDTH(2)) sync_scl_ridx(.rst_n(RSTn), .clk(CLK), .scl_data(scl_ridx), 
                              .out_clk(tb_ridx));
  SYNC_C2S #(.WIDTH(2)) sync_tb_widx(.rst_n(RSTn), .scl(SCL), .clk_data(tb_widx), 
                             .out_scl(scl_widx));

  //
  // 系统 CLK 域
  //
  assign tb_empty    = tb_widx==tb_ridx;
  assign tb_not_full = tb_empty | ^(tb_widx ^ tb_ridx);
  assign tb_wptr     = ^tb_widx;        // 给出 0,1,0,1

  always @ (posedge CLK or negedge RSTn)
    if (!RSTn) begin
      tb_widx          <= 2'b00;
      ack_push         <= 1'b0;
      tbfifo[0]        <= 9'd0;
      tbfifo[1]        <= 9'd0;
    end else if (tb_flush) // 注意：flush 覆盖 push
      tb_widx          <= tb_ridx; 
    else if (avail_tb_ready & tb_not_full) begin
      // 可以安全推送下一个值。注意它们同步 tb_widx，
      // 因此位变化无风险（它们在保持稳定后看到 widx）
      tbfifo[tb_wptr]  <= {avail_tb_end, avail_tb_data};
      tb_widx          <= next_widx;
      ack_push         <= 1'b1;
    end else
      ack_push         <= 1'b0;

  assign avail_tb_ack   = (EXT_FIFO==`EXT_FIFO_REQ) ?
                          (avail_tb_ready & tb_not_full) : // 组合逻辑
                          ack_push;     // 寄存器输出
  assign int_tb         = |tx_trig ? tb_not_full : tb_empty; // 触发水平 0 特殊
  assign avail_tb_full  = ~tb_not_full;
  assign avail_byte_cnt = tb_empty ? 5'd0 : tb_not_full ? 5'd1 : 5'd2;

  // 格雷码
  always @ ( * ) 
    case(tb_widx)
    2'b00: next_widx = 2'b01;
    2'b01: next_widx = 2'b11;
    2'b11: next_widx = 2'b10;
    2'b10: next_widx = 2'b00;
    // 无默认，因为完整
    endcase

  //
  // SCL 时钟域（非 SCL_n）用于 CDC
  //

  // 读取指针
  always @ (posedge SCL or negedge RSTn)
    if (~RSTn) 
      scl_ridx <= 2'b00;
    else if (tb_datab_ack & ~scl_empty)
      scl_ridx <= next_ridx;

  assign scl_rptr      = ^scl_ridx;     // 给出 0,1,0,1
  assign scl_empty     = scl_widx==scl_ridx;
  assign tb_data_valid = ~scl_empty; 
  assign tb_end        = tbfifo[scl_rptr][8];
    // 注意：当未选择且尚未使用时，以下值不确定
  assign tb_datab      = tbfifo[scl_rptr][7:0];

  // 格雷码
  always @ ( * ) 
    case(scl_ridx)
    2'b00: next_ridx = 2'b01;
    2'b01: next_ridx = 2'b11;
    2'b11: next_ridx = 2'b10;
    2'b10: next_ridx = 2'b00;
    // 无默认，因为完整
    endcase

 //
 // 内部 FIFO
 //
 end else begin : tb_fifo_inst
  // 实例化 FIFO，具有正确的大小和可选的保持缓冲区
  i3c_internal_tb_fifo #(.BITS(ENA_TOBUS_FIFO), 
                         .USE_HOLDING(FIFO_TYPE[`FIFO_TBHOLD_b]))
    tb_fifo(.RSTn(RSTn), .CLK(CLK), .SCL(SCL), .SCL_n(SCL_n), 
            .avail_tb_ready(avail_tb_ready),
            .avail_tb_data(avail_tb_data), .avail_tb_ack(avail_tb_ack),
            .avail_tb_end(avail_tb_end), .tb_flush(tb_flush), 
            .avail_tb_full(avail_tb_full), .avail_byte_cnt(avail_byte_cnt), 
            .tx_trig(tx_trig), .int_tb(int_tb), .tb_data_valid(tb_data_valid), 
            .tb_datab(tb_datab), .tb_end(tb_end), .tb_datab_ack(tb_datab_ack), 
            .scan_no_rst(scan_no_rst));
 end endgenerate

  //
  // 使用 SCL 同步错误
  //

  // 到总线的欠载错误（头部和读取字节）
  SYNC_2PH_S2C_STATE synch_tb_urun_nack(.rst_n(RSTn), .scl(SCL), .clk(CLK), 
                                        .trig_scl(tb_urun_nack), 
                                        .out_clk(set_tb_urun_nack), .clear_clk(clear_tb_urun_nack));
  SYNC_2PH_S2C_STATE synch_tb_urun     (.rst_n(RSTn), .scl(SCL), .clk(CLK), 
                                        .trig_scl(tb_urun), 
                                        .out_clk(set_tb_urun), .clear_clk(clear_tb_urun));

  // 终止是主设备在我们完成之前终止读取
  SYNC_2PH_S2C_STATE synch_tb_term     (.rst_n(RSTn), .scl(SCL), .clk(CLK), 
                                        .trig_scl(tb_term), 
                                        .out_clk(set_tb_term), .clear_clk(clear_tb_term));

endmodule