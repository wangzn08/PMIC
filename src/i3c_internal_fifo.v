`include "i3c_params.v"                 // 本地参数/常量

// ============================================================================
// 模块：i3c_internal_tb_fifo
// 功能：I3C内部到总线FIFO模块（发送方向）
// 描述：实现从系统时钟域(CLK)到I3C总线时钟域(SCL)的数据缓冲和时钟域交叉
//       支持可配置深度的FIFO，可选的保持缓冲，以及触发中断控制
// ============================================================================
module i3c_internal_tb_fifo #(
    parameter BITS = 3,                 // FIFO深度位数（2^BITS）
    parameter USE_HOLDING = 0           // 是否使用保持缓冲（1=是）
  )
  (
  input               RSTn,             // 全局复位
  input               CLK,              // 系统时钟
  input               SCL,              // I3C总线时钟
  input               SCL_n,            // I3C总线反向时钟
  // CLK域接口：来自寄存器或逻辑
  input               avail_tb_ready,   // 到总线数据准备就绪
  input         [7:0] avail_tb_data,    // 数据字节
  output              avail_tb_ack,     // 数据已接收确认
  input               avail_tb_end,     // 最后一个字节标志（与ready同时有效）
  input               tb_flush,         // 缓冲刷新脉冲
  output              avail_tb_full,    // FIFO真正满标志（非触发）
  output        [4:0] avail_byte_cnt,   // FIFO中字节计数
  input         [1:0] tx_trig,          // 发送触发阈值百分比
  output              int_tb,           // 发送中断（FIFO非满/达到触发条件）
  // SCL域接口：到I3C总线
  output              tb_data_valid,    // 数据有效标志
  output        [7:0] tb_datab,         // 发送数据
  output              tb_end,           // 最后字节标志
  input               tb_datab_ack,     // 数据消耗确认（SCL_n时钟域）
  input               scan_no_rst       // 扫描测试时禁止分层复位
  );

  // 内部信号声明
  wire       [BITS:0] rclk_wgray, wclk_rgray;
  wire                read_empty, write_full;
  wire       [BITS:0] export_rgray, export_wgray;
  wire     [BITS-1:0] read_idx, write_idx, local_ridx;
  reg                 wr_ack;
  wire                reset_flush_n;
  wire          [4:0] tmp;
  wire                wr_is_empty;
  wire          [1:0] match;
  reg                 trig;
  wire     [BITS-1:0] hold_next_idx;
  // FIFO存储器：9位宽度（8位数据+1位结束标志）
  reg           [8:0] tb_fifo[0:(1<<BITS)-1];
  wire          [8:0] opt_holding;
  wire                write_one;

  // 时钟域同步：写指针（CLK域）同步到读时钟域（SCL域）
  SYNC_S2C #(.WIDTH(BITS+1)) w2r (.rst_n(RSTn), .clk(SCL), .scl_data(export_wgray), 
                          .out_clk(rclk_wgray));
  // 时钟域同步：读指针（SCL域）同步到写时钟域（CLK域）
  SYNC_S2C #(.WIDTH(BITS+1)) r2w (.rst_n(RSTn), .clk(CLK), .scl_data(export_rgray), 
                          .out_clk(wclk_rgray));

  // 复位逻辑：支持刷新操作
  assign reset_flush_n = RSTn & (~tb_flush | scan_no_rst);

  // FIFO写控制模块
  i3c_internal_fifo_write #(.BITS(BITS)) tb_wr_fifo(.RSTn(reset_flush_n), .WCLK(CLK), 
                                     .write_one(write_one), .wclk_rgray(wclk_rgray),
                                     .write_full(write_full), .export_wgray(export_wgray), 
                                     .write_idx(write_idx));
  // FIFO读控制模块
  i3c_internal_fifo_read #(.BITS(BITS)) tb_rd_fifo(.RSTn(reset_flush_n), .RCLK(SCL), 
                                     .read_one(tb_datab_ack), .rclk_wgray(rclk_wgray), 
                                     .read_empty(read_empty), .export_rgray(export_rgray), 
                                     .read_idx(read_idx), .next_read_idx(hold_next_idx));

  // 写控制逻辑
  assign write_one = avail_tb_ready & ~write_full;
  always @ (posedge CLK or negedge RSTn)
    if (~RSTn)
      wr_ack <= 1'b0;
    else if (write_one)
      wr_ack <= 1'b1;           // 支持多周期推送
    else 
      wr_ack <= 1'b0;
  assign avail_tb_ack   = wr_ack;     // 字节接收确认
  assign tb_data_valid  = ~read_empty; // 数据有效（FIFO非空）
  assign tb_datab       = opt_holding[7:0]; // 输出数据
  assign tb_end         = opt_holding[8] & tb_data_valid; // 结束标志

  // 保持缓冲生成逻辑
  generate if (USE_HOLDING) begin : to_bus_holding
    reg       [8:0] holding;           // 保持缓冲寄存器
    reg  [BITS-1:0] last_idx;          // 上次读取索引
    wire [BITS-1:0] use_idx = (tb_datab_ack & ~read_empty) ? hold_next_idx : read_idx;
    always @ (posedge SCL or negedge RSTn)
      if (!RSTn) begin
        holding   <= 9'd0;
        last_idx  <= {BITS{1'b0}};
      end else if ((last_idx != use_idx) |
                   (holding != tb_fifo[use_idx])) begin
        // 当索引变化或FIFO内容更新时更新保持缓冲
        holding   <= tb_fifo[use_idx];
        last_idx  <= use_idx;
      end
    assign opt_holding = holding;      // 使用保持缓冲输出
  end else begin
    assign opt_holding = tb_fifo[read_idx]; // 直接FIFO输出
  end endgenerate

  // FIFO存储器写入逻辑
  integer ini;
  always @ (posedge CLK or negedge RSTn)
    if (~RSTn) begin
      // DFT考虑：复位FIFO内容
      for (ini = 0; ini < (1<<BITS); ini = ini + 1)
        tb_fifo[ini]     <= 9'd0;
    end else if (avail_tb_ready & ~write_full)
      tb_fifo[write_idx] <= {avail_tb_end, avail_tb_data}; // 写入数据和结束标志
    
  // FIFO状态计算：字节计数和中断触发
  assign wr_is_empty    = export_wgray == wclk_rgray; // 写空判断
  assign avail_tb_full  = write_full;                 // 满标志
  assign tmp            = (write_idx-local_ridx);     // 写读索引差
  // 字节计数计算
  assign avail_byte_cnt = tmp[(BITS-1):0] ? tmp[(BITS-1):0] : wr_is_empty ? 5'd0 : (5'd1<<BITS);
  assign match          = tmp[(BITS-1) -: 2];         // 高两位用于满判断
  // 中断生成逻辑
  assign int_tb         = write_full ? 1'b0 :         // 满时无中断
                          wr_is_empty ? 1'b1 :        // 空时总有中断
                          trig;                       // 否则根据触发阈值
  always @ ( * )
    case (tx_trig)        // 触发阈值配置
    2'b00:   trig = wr_is_empty;        // 仅空时触发
    2'b01:   trig = tmp[(BITS-1):0] <= (1<<BITS)/4; // 25%以下
    2'b10:   trig = tmp[(BITS-1):0] <= (1<<BITS)/2; // 50%以下
    2'b11:   trig = 1'b1;               // 非满即触发
    default: trig = 1'b0;
    endcase

  // 格雷码转二进制：同步的读指针转换为二进制索引
  genvar i;
  generate for (i = BITS-1; i >= 0; i = i -1) begin : tb_gen_num
    assign local_ridx[i] = ^wclk_rgray[BITS:i];
  end endgenerate

endmodule

// ============================================================================
// 模块：i3c_internal_fb_fifo
// 功能：I3C内部从总线FIFO模块（接收方向）
// 描述：实现从I3C总线时钟域(SCL)到系统时钟域(CLK)的数据缓冲和时钟域交叉
//       支持可配置深度的FIFO，以及接收中断触发控制
// ============================================================================
module i3c_internal_fb_fifo #(
    parameter BITS = 3                  // FIFO深度位数（2^BITS）
  )
  (
  input               RSTn,             // 全局复位
  input               CLK,              // 系统时钟
  input               SCL,              // I3C总线时钟
  input               SCL_n,            // I3C总线反向时钟
  // CLK域接口：到寄存器或逻辑
  output              notify_fb_ready,  // 从总线数据准备就绪
  output        [7:0] notify_fb_data,   // 数据字节
  input               notify_fb_ack,    // 数据已取走确认
  input               fb_flush,         // 缓冲刷新脉冲
  output              avail_fb_empty,   // FIFO真正空标志（非触发）
  output        [4:0] avail_byte_cnt,   // FIFO中字节计数
  input         [1:0] rx_trig,          // 接收触发阈值百分比
  output              int_rx,           // 接收中断（FIFO非空/达到触发条件）
  // SCL域接口：从I3C总线
  output              fb_data_use,      // 数据可使用标志（非满）
  input         [7:0] fb_datab,         // 接收数据
  input               fb_datab_done,    // 数据写入完成脉冲
  input               scan_no_rst       // 扫描测试时禁止分层复位
  );

  // 内部信号声明
  wire       [BITS:0] rclk_wgray, wclk_rgray;
  wire                read_empty, write_full;
  wire       [BITS:0] export_rgray, export_wgray;
  wire     [BITS-1:0] read_idx, write_idx, local_widx;
  wire                reset_flush_n;
  wire          [4:0] tmp;
  wire          [1:0] match;
  reg                 trig;
  wire     [BITS-1:0] unused2;
  // FIFO存储器：8位宽度
  reg           [7:0] fb_fifo[0:(1<<BITS)-1];

  // 时钟域同步：写指针（SCL域）同步到读时钟域（CLK域）
  SYNC_S2C #(.WIDTH(BITS+1)) w2r (.rst_n(RSTn), .clk(CLK), .scl_data(export_wgray), 
                          .out_clk(rclk_wgray));
  // 时钟域同步：读指针（CLK域）同步到写时钟域（SCL域）
  SYNC_S2C #(.WIDTH(BITS+1)) r2w (.rst_n(RSTn), .clk(SCL), .scl_data(export_rgray), 
                          .out_clk(wclk_rgray));

  // 复位逻辑：支持刷新操作
  assign reset_flush_n = RSTn & (~fb_flush | scan_no_rst);

  // FIFO写控制模块（SCL域）
  i3c_internal_fifo_write #(.BITS(BITS)) fb_wr_fifo(.RSTn(reset_flush_n), .WCLK(SCL),
                                     .write_one(fb_datab_done), .wclk_rgray(wclk_rgray),
                                     .write_full(write_full), .export_wgray(export_wgray), 
                                     .write_idx(write_idx));
  // FIFO读控制模块（CLK域）
  i3c_internal_fifo_read #(.BITS(BITS)) fb_rd_fifo(.RSTn(reset_flush_n), .RCLK(CLK), .read_one(notify_fb_ack), 
                                     .rclk_wgray(rclk_wgray), 
                                     .read_empty(read_empty), .export_rgray(export_rgray), 
                                     .read_idx(read_idx), .next_read_idx(unused2));

  // 输出信号赋值
  assign notify_fb_ready= ~read_empty;  // 数据等待标志
  assign notify_fb_data = fb_fifo[read_idx]; // 读取数据
  assign fb_data_use    = write_full ? 1'b0 : 1'b1; // 可使用标志（非满）

  // FIFO存储器写入逻辑（SCL域）
  always @ (posedge SCL)
    if (fb_datab_done & ~write_full)
      fb_fifo[write_idx] <= fb_datab; 
    
  // FIFO状态计算：字节计数和中断触发
  assign tmp            = (local_widx-read_idx); // 写读索引差
  // 字节计数计算
  assign avail_byte_cnt = tmp[(BITS-1):0] ? tmp[(BITS-1):0] : 
                             read_empty ? 5'd0 : (5'd1<<BITS);
  assign match          = tmp[(BITS-1) -: 2]; // 高两位
  // 中断生成逻辑
  assign int_rx         = read_empty ? 1'b0 : // 空时无中断
                          ~|tmp[(BITS-1):0] ? 1'b1 : // 满时总有中断
                          trig;              // 否则根据触发阈值
  assign avail_fb_empty = read_empty;       // 空标志

  always @ ( * )
    case (rx_trig)        // 触发阈值配置
    2'b00:   trig = 1'b1;               // 非空即触发
    2'b01:   trig = tmp[(BITS-1):0] >= (1<<BITS)/4; // 25%以上
    2'b10:   trig = tmp[(BITS-1):0] >= (1<<BITS)/2; // 50%以上
    2'b11:   trig = tmp[(BITS-1):0] >= ((1<<BITS)/4)*3; // 75%以上
    default: trig = 1'b0;
    endcase

  // 格雷码转二进制：同步的写指针转换为二进制索引
  genvar i;
  generate  for (i = BITS-1; i >= 0; i = i -1) begin : fb_gen_num
    assign local_widx[i] = ^rclk_wgray[BITS:i];
  end endgenerate

endmodule


// ============================================================================
// 模块：i3c_internal_fifo_write
// 功能：FIFO写侧控制模块（基于Sunburst设计）
// 描述：管理FIFO的写指针、满状态判断和格雷码生成
//       注意：满状态使用组合逻辑实现以缩短路径延迟
// ============================================================================
module i3c_internal_fifo_write #(parameter BITS=3) // 位宽参数（深度2^BITS）
  (
  input               RSTn,             // 全局复位
  input               WCLK,             // 写时钟（CLK或SCL）
  input               write_one,        // 写请求（非满时可写入）
  input      [BITS:0] wclk_rgray,       // 同步到本域的读侧格雷码指针
  output              write_full,       // FIFO满标志
  output     [BITS:0] export_wgray,     // 输出到其他时钟域的格雷码指针
  output   [BITS-1:0] write_idx         // 写索引（FIFO地址）
  );
 
  // 设计说明：
  // 1. 写侧只关心满状态，非满时可写入
  // 2. 满状态只能由本侧写入操作产生，在本时钟域检测是安全的
  // 3. 非满状态从读侧同步而来，可能有延迟，但只影响写入时机，无风险
  // 计数器比FIFO索引多1位，用于区分空满状态：
  //   空：读写指针相等；满：读写指针最高位和次高位不同，其余位相等
  reg     [BITS:0] write_full_idx, write_gray;
  wire    [BITS:0] next_full_idx, next_gray;

  // 二进制计数器：写操作时递增
  always @ (posedge WCLK or negedge RSTn)
    if (~RSTn)
      write_full_idx <= {BITS+1{1'b0}};
    else if (write_one & ~write_full)
      write_full_idx <= next_full_idx;
  assign next_full_idx = write_full_idx + {{BITS{1'b0}}, 1'b1};
  assign write_idx     = write_full_idx[BITS-1:0]; // FIFO索引（少1位）

  // 满状态判断：组合逻辑比较格雷码
  // 比较规则：写格雷码等于{~读格雷码最高两位, 读格雷码低位}
  assign write_full    = write_gray == {~wclk_rgray[BITS -: 2],
                                       wclk_rgray[BITS-2:0]};

  // 格雷码计数器：用于跨时钟域同步
  always @ (posedge WCLK or negedge RSTn)
    if (~RSTn)
      write_gray <= {BITS+1{1'b0}};
    else if (write_one & ~write_full)
      write_gray <= next_gray;
  assign next_gray     = next_full_idx[BITS:1] ^ next_full_idx; // 二进制转格雷码
  assign export_wgray  = write_gray;                           // 输出格雷码

endmodule


// ============================================================================
// 模块：i3c_internal_fifo_read
// 功能：FIFO读侧控制模块（基于Sunburst设计）
// 描述：管理FIFO的读指针、空状态判断和格雷码生成
//       注意：空状态使用组合逻辑实现以缩短路径延迟
// ============================================================================
module i3c_internal_fifo_read #(parameter BITS=3) // 位宽参数（深度2^BITS）
  (
  input               RSTn,             // 全局复位
  input               RCLK,             // 读时钟（CLK或SCL）
  input               read_one,         // 读请求（非空时可读取）
  input      [BITS:0] rclk_wgray,       // 同步到本域的写侧格雷码指针
  output              read_empty,       // FIFO空标志
  output     [BITS:0] export_rgray,     // 输出到其他时钟域的格雷码指针
  output   [BITS-1:0] read_idx,         // 读索引（FIFO地址）
  // 用于保持缓冲的下一个读索引（可选）
  output   [BITS-1:0] next_read_idx
  );
 
  // 设计说明：
  // 1. 读侧只关心空状态，非空时可读取
  // 2. 空状态只能由本侧读取操作产生，在本时钟域检测是安全的
  // 3. 非空状态从写侧同步而来，可能有延迟，但只影响读取时机，无风险
  // 计数器比FIFO索引多1位，用于区分空满状态
  reg     [BITS:0] read_full_idx, read_gray;
  wire    [BITS:0] next_full_idx, next_gray;

  // 二进制计数器：读操作时递增
  always @ (posedge RCLK or negedge RSTn)
    if (~RSTn)
      read_full_idx <= {BITS+1{1'b0}};
    else if (read_one & ~read_empty)
      read_full_idx <= next_full_idx;
  assign next_full_idx = read_full_idx + {{BITS{1'b0}}, 1'b1};
  assign read_idx      = read_full_idx[BITS-1:0]; // FIFO索引（少1位）

  // 空状态判断：组合逻辑比较格雷码是否相等
  assign read_empty    = read_gray == rclk_wgray;

  // 格雷码计数器：用于跨时钟域同步
  always @ (posedge RCLK or negedge RSTn)
    if (~RSTn)
      read_gray <= {BITS+1{1'b0}};
    else if (read_one & ~read_empty)
      read_gray <= next_gray;
  assign next_gray     = next_full_idx[BITS:1] ^ next_full_idx; // 二进制转格雷码
  assign export_rgray  = read_gray;                           // 输出格雷码

  // 下一个读索引（用于保持缓冲提前获取）
  assign next_read_idx   = next_full_idx[BITS-1:0];

endmodule