/*--------------------------------------------------------------------
  设计信息
  --------------------------------------------------------------------*/

// 文件名称        : i3c_auton_wrapper.v
// 描述            : 支持自主寄存器的 MIPI I3C 从设备包装器
//                    此模块为 I3C 自主模式提供包装器，所有消息均通过生成的寄存器进行读写，
//                    以支持基于状态机的设计。i3c_autonomous_regs 模块根据参数生成一组寄存器
//                    和网络。本包装器用于封装组合逻辑，并处理来自系统的通知和 IBI（中断）请求。
// 注意            : 此模块仅实例化完整的自主包装器

`include "i3c_params.v"                 // 局部参数/常量

module i3c_auton_wrapper #(
    // 参数由上层驱动，用于控制此模块的构建和操作
    // 第一组与自主寄存器相关
      // 规则：[0]=仅在RUN[idx]=0时可写，[1]=在RUN[idx]=0时忽略写入，
      //       [2]=仅在RUN[idx]时触碰，[3]=写操作时触碰（非读）
      //       [4]=在S（非Sr）时重置索引，[5]=保留用于Magic寄存器（总线）
      //       [7:6]=DDR规则：0:CMD=索引，1:CMD=索引（除7F外），2:忽略CMD
      //       [8]=从无效寄存器读取时返回NACK，[15:9]=保留
    parameter REG_RULES       = 16'd0,  // 上述规则
    parameter MAX_REG         = 8'h00,  // 最后一个索引，范围1至255
    parameter REG_WRITABLE    = {MAX_REG+1{1'b0}}, // 可写寄存器
    parameter REG_READABLE    = {MAX_REG+1{1'b0}}, // 可读寄存器
    parameter REG_RUN         = {MAX_REG+1{1'b0}}, // 不连续边界
    parameter REG_MASK        = {((MAX_REG+1)*8){1'b1}}, // 全位或部分位掩码
    parameter REG_BLEND       = {MAX_REG+1{1'b0}}, // 混合写/读
    parameter REG_REMAP       = {((MAX_REG+1)*8){1'b1}}, // 可选重映射
    parameter REG_RESET       = {((MAX_REG+1)*8){1'b0}}, // 写触发器的复位值
    // 标准参数，即使不直接适用也保留
    parameter ENA_ID48B       = `ID48B_CONST, // 常量 vs. 寄存器
    parameter  ID_48B         = 48'h0,  // 必须由上层填充部分或全部48位ID
    parameter ID_AS_REGS      = 12'd0,  // [0]=IDINST, [1]=ISRAND, [2]=IDDCR, [3]=IDBCR
    // 注意：BCR在下方，以便使用其他参数
    parameter  ID_DCR         = 8'd0,   // 若非来自寄存器，则由上层填充DCR
    parameter ENA_SADDR       = `SADDR_NONE, // 无、常量、寄存器/网络
    parameter  SADDR_P        = 0,      // 7位地址，位置6:0
    parameter ENA_IBI_MR_HJ   = 0,      // 0表示无事件，否则为事件掩码
    parameter  CLK_SLOW_BITS  = 6,      // 总线可用计数所需位数
    parameter  CLK_SLOW_MATCH = 6'd47,  // 计数值（例如48MHz时钟下47对应1us）
    parameter  CLK_SLOW_HJMUL = 10'd1000,// MATCH次数（常量或寄存器）对应1ms
      // 错误处理：若启用IBI，则默认读取中止
    parameter  ERROR_HANDLING = 3'd0|((|ENA_IBI_MR_HJ)<<`ERR_RDABT_b), 
    parameter  ENA_TIMEC      = 6'b000010,    // 若为寄存器：保留、保留、模式1、模式0、同步
    parameter  TIMEC_FREQ_ACC = {8'd24,8'd10},// 频率=12MHz（12.0=24），精度1.0%
    parameter ENA_CCC_HANDLING= 6'd0,   // 传递所支持的CCC处理方式
    parameter MAX_RDLEN       = 0,      // 默认从设备->主设备最大长度
    parameter MAX_WRLEN       = 0,      // 默认主设备->从设备最大长度
    parameter MAX_DS_WR       = 0,      // 主设备->从设备的数据速度限制
    parameter MAX_DS_RD       = 0,      // 从设备->主设备的数据速度限制
    parameter MAX_DS_RDTURN   = 0,      // 从设备->主设备读取请求的延迟需求
    parameter ENA_HDR         = 0,      // HDR模式使能
    parameter PIN_MODEL       = `PINM_COMBO, // 组合引脚使用模式
      // BCR根据参数自动填充。但不能离线或桥接
    parameter  ID_BCR         = (|ENA_HDR<<5) | ((ENA_IBI_MR_HJ&8'h03)<<1) |
                                |(MAX_DS_RDTURN|MAX_DS_WR|MAX_DS_RD), // 限制在[0]
    // 下一参数在此计算，不传递
    parameter priv_sz         = PIN_MODEL[1] ? (ENA_HDR[0] ? 3 : 1) : 0// 若外部引脚+DDR则更宽
  )
  (
  // 时钟和复位定义
  input               RSTn,             // 系统复位
  input               CLK,              // 寄存器同步的系统时钟
  input               CLK_SLOW,         // 用于IBI强制的时钟
  output              slow_gate,        // 为1时可门控CLK_SLOW
  input               CLK_SLOW_TC,      // 时间控制时钟，可与CLK_SLOW相同
  output              tc_slow_gate,     // 为1时可门控CLK_SLOW_TC；若共用则与slow_gate相与
  // 引脚定义。SCL仅为输入
  input               pin_SCL_in,       // SCL：通常为输入时钟，若为三态则用作数据
  input               pin_SDA_in,       // SDA：主设备->从设备
    // 注意：当PIN_MODEL==`PINM_EXT_REG时，以下3个信号特殊使用
    //       它们驱动靠近焊盘的3个触发器
  output              pin_SDA_out,      // SDA：读取时从设备->主设备
  output              pin_SDA_oena,     // SDA：读取时从设备->主设备使能
  output  [priv_sz:0] pin_SDA_oena_rise,// 若为EXT_REG，则在SCL上升沿特殊使用，否则为0
    // 仅当SDA和SCL焊盘具有I2C 50ns尖峰滤波器且可通过网络开关时使用下一信号
  output              i2c_spike_ok,     // 为1时允许I2C尖峰滤波器
    // 自主使用：通知的o_网络从SCL域同步到CLK域。
    // 注意：start和done是到此从设备（非CCC）的消息，至少持续1个CLK周期。
    // wo_regs和raw来自SCL域（因此应仅在安全时使用通知采样），
    // ro_regs假定来自CLK域（因此应仅在安全时使用通知更改）。
    // touch网络按位原始。系统应使用提供的同步模块处理本地同步。
    // 这属于系统部分，以保持从Q到使用的路径短，防止亚稳态问题。
    // 注意：touch可能按寄存器或按运行（由REG_RULES控制）。
    // OK（完成 vs. 纯ACK）按位从CLK域脉冲。脉冲必须持续触发器复位所需时间
    // （异步且通常很短，因此通常不是问题）。
  output              osync_started,    // 当新传输开始时，脉冲1个CLK周期
  output              o_read,           // 为1表示开始从本设备读取，否则为写入
  output              osync_done,       // 读写结束时脉冲1个CLK周期
  output              osync_any_touch,  // 当osync_done=1时，若有任何寄存器被读写则脉冲1
  output  [MAX_REG:0] oraw_reg_touch,   // 若寄存器或运行被读写，则对应位为1
  input   [MAX_REG:0] iraw_touch_OK,    // 系统脉冲接受来自CLK域的touch[i]：作为复位
  output [(8*MAX_REG)+7:0] wo_regs,     // 主设备写入的寄存器 - 原始值
  input  [(8*MAX_REG)+7:0] ro_regs,     // 系统提供的供主设备读取的寄存器 - 原始值
    // 与自主相关的中断请求。请求来自CLK域并在本地同步。
    // 输出done也同步到CLK域。若o_ibi_done脉冲，则IBI已通过并被接受（包括字节）。
    // 若o_ibi_nacked脉冲，系统需决定是否重试。
  input               i_ibi_event,      // 脉冲1个CLK周期表示事件
  input               i_ibi_req,        // 保持1以请求IBI，完成（确认）或取消时清除
  input               i_hj_req,         // 保持1以请求HJ（若允许）；清除同ibi_done
  output              o_ibi_done,       // IBI完成且ACK时脉冲1个CLK周期
  output              o_ibi_nacked,     // IBI完成且NACK时脉冲1个CLK周期
  input         [7:0] i_ibi_byte,       // 若需要IBI字节。保持直到完成
  // 配置（稳定）输入。不使用时（由参数选择）应绑为0或1。
  input               cf_SlvEna,        // 为1时才处理总线
  input         [7:0] cf_SlvSA,         // 可选的从设备I2C地址（[0]=1表示启用）
  input         [3:0] cf_IdInst,        // ID实例（若使用，非零件号时）
  input               cf_IdRand,        // ID的随机零件号（若使用）
  input        [31:0] cf_Partno,        // ID的零件号（若使用）
  input         [7:0] cf_IdBcr,         // DAA的BCR（若非常量）
  input         [7:0] cf_IdDcr,         // DAA的DCR（若非常量）
  input        [14:0] cf_IdVid,         // 厂商ID（若非常量）
  // 来自SCL域的原始特殊状态 - 可实时同步用于STATUS读取。
  output      [29:28] raw_ActState,     // 来自ENTASn的总线活动状态
  output      [27:24] raw_EvState,      // 来自ENEC/DISEC的事件使能状态
  output        [6:0] raw_Request,      // 进行中的总线请求及细节
  output        [7:0] raw_DynAddr,      // 设置的动态地址 - 稳定
  output       [13:0] raw_timec_sync,   // 系统处理的SYNC时间控制。[13]=时钟
  // 以下为安全原始信号。意味着在总线STOP时或未执行GETSTATUS时更改，或已同步。
  // 若不需用，可绑为0。
  input         [7:6] sraw_ActMode,     // 系统活动模式（若不使用则为0）
  input         [3:0] sraw_PendInt,     // 待处理中断（若不使用则为0）
  input        [15:8] sraw_StatusRes,   // 状态保留位（若不使用则为0）
  // 检测到的错误（例如超过运行结束）
  // 可忽略。此处仅为需要时提供。
  // 这些是SCL域的原始信号，因此若使用需为粘性或握手。
  output              raw_tb_reg_err,
  output              raw_fb_reg_err,
  // 扫描相关
  input               scan_single_clock,// 扫描时引脚时钟的单时钟域
  input               scan_clk,         // 扫描时用于引脚时钟的时钟
  input               scan_no_rst,      // 防止分层复位
  input               scan_no_gates     // 防止架构时钟门控
  );

  // 仅实例化完整包装器并绑断Magic寄存器未使用的网络
  i3c_auton_wrap_full #(.ENA_ID48B(ENA_ID48B),.ID_48B(ID_48B),
                        .ID_AS_REGS(ID_AS_REGS),.ID_BCR(ID_BCR),.ID_DCR(ID_DCR),
                        .ENA_SADDR(ENA_SADDR),.SADDR_P(SADDR_P),
                        .ENA_IBI_MR_HJ(ENA_IBI_MR_HJ),
                          .CLK_SLOW_BITS(CLK_SLOW_BITS),
                          .CLK_SLOW_MATCH(CLK_SLOW_MATCH),
                          .CLK_SLOW_HJMUL(CLK_SLOW_HJMUL),
                        .ERROR_HANDLING(ERROR_HANDLING),
                        .ENA_CCC_HANDLING(ENA_CCC_HANDLING), 
                         .ENA_TIMEC(ENA_TIMEC),.TIMEC_FREQ_ACC(TIMEC_FREQ_ACC),
                        .MAX_RDLEN(MAX_RDLEN),.MAX_WRLEN(MAX_WRLEN),
                         .MAX_DS_WR(MAX_DS_WR),.MAX_DS_RD(MAX_DS_RD),
                         .MAX_DS_RDTURN(MAX_DS_RDTURN),
                        .ENA_HDR(ENA_HDR),
                        .PIN_MODEL(PIN_MODEL),
                        .REG_RULES(REG_RULES), .MAX_REG(MAX_REG), 
                         .REG_WRITABLE(REG_WRITABLE), .REG_READABLE(REG_READABLE),
                         .REG_RUN(REG_RUN), .REG_MASK(REG_MASK), .REG_BLEND(REG_BLEND),
                         .REG_REMAP(REG_REMAP),.REG_RESET(REG_RESET),
                        .MAGIC_RULES(0),.MAGIC_MASK(0),.MAGIC_MATCH(0))
    auton_full 
    (
    .RSTn             (RSTn), 
    .CLK              (CLK), 
    .CLK_SLOW         (CLK_SLOW), 
    .slow_gate        (slow_gate), 
    .CLK_SLOW_TC      (CLK_SLOW), // 与slow时钟相同
    .tc_slow_gate     (tc_slow_gate), // 若使用，因共用将与slow_gate相与
    .pin_SCL_in       (pin_SCL_in), 
    .pin_SDA_in       (pin_SDA_in), 
    .pin_SDA_out      (pin_SDA_out), 
    .pin_SDA_oena     (pin_SDA_oena), 
    .pin_SDA_oena_rise(pin_SDA_oena_rise), 
    .i2c_spike_ok     (i2c_spike_ok), 
    .osync_started    (osync_started), 
    .o_read           (o_read), 
    .osync_done       (osync_done), 
    .osync_any_touch  (osync_any_touch), 
    .oraw_reg_touch   (oraw_reg_touch), 
    .iraw_touch_OK    (iraw_touch_OK), 
    .wo_regs          (wo_regs), 
    .ro_regs          (ro_regs), 
    .i_ibi_event      (i_ibi_req), // 事件和请求相同
    .i_ibi_req        (i_ibi_req), 
    .i_hj_req         (i_hj_req), 
    .o_ibi_done       (o_ibi_done), 
    .o_ibi_nacked     (o_ibi_nacked), 
    .i_ibi_byte       (i_ibi_byte), 
    .cf_SlvEna        (cf_SlvEna), 
    .cf_SlvSA         (cf_SlvSA), 
    .cf_IdInst        (cf_IdInst), 
    .cf_IdRand        (cf_IdRand), 
    .cf_Partno        (cf_Partno), 
    .cf_IdVid         (cf_IdVid),
    .cf_SlvNack       (1'b0),
    .cf_BAMatch       (8'd0),
    .cf_IdBcr         (cf_IdBcr), 
    .cf_IdDcr         (cf_IdDcr), 
    .raw_ActState     (raw_ActState), 
    .raw_EvState      (raw_EvState), 
    .raw_Request      (raw_Request), 
    .raw_DynAddr      (raw_DynAddr), 
    .raw_timec_sync   (raw_timec_sync),
    .raw_slvr_reset   (),
    .iraw_rst_slvr_reset(1'b0),
    .sraw_ActMode     (sraw_ActMode), 
    .sraw_PendInt     (sraw_PendInt), 
    .sraw_StatusRes   (sraw_StatusRes), 
    .raw_fb_reg_err   (raw_fb_reg_err),
    .raw_tb_reg_err   (raw_tb_reg_err),
    .mr_clk_SCL       (mr_clk_SCL),
    .mr_clk_SCL_n     (mr_clk_SCL_n),
    .mr_idx           (),
    .mr_next_idx      (),
    .mr_dir           (),
    .mr_trans_w_done  (),
    .mr_trans_r_done  (),
    .mr_trans_ccc_done(),
    .mr_trans_ddrcmd  (),
    .mr_wdata         (),
    .mr_rdata         (8'd0),
    .mr_rdata_none    (1'b0),
    .mr_rdata_end     (1'b0),
    .mr_bus_new       (),
    .mr_bus_done      (),
    .scan_single_clock(scan_single_clock), 
    .scan_clk         (scan_clk), 
    .scan_no_rst      (scan_no_rst), 
    .scan_no_gates    (scan_no_gates)
  );

endmodule