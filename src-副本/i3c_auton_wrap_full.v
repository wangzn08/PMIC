/*--------------------------------------------------------------------
// 设计信息
// ------------------------------------------------------------------------------
// 文件: i3c_auton_wrap_full.v
// 组织: MCO
// 标签: 1.1.11
// 日期: $Date: Thu Oct 24 08:54:29 2019 $
// 版本: $Revision: 1.45 $
//
// IP名称: i3c_auton_wrap_full
// 描述: MIPI I3C从设备支持自主寄存器
// 该模块包含更全面的自主寄存器支持，包括映射到系统总线的"魔法寄存器"(使用SCL作为时钟)。
// 该模块是I3C的自主封装，所有消息都进入/离开生成的寄存器，以支持基于状态机的设计。
// i3c_autonomous_regs模块基于参数提供一组生成的寄存器和网络，以及为魔法寄存器(如果存在)导出总线。
// 这允许使用字节索引访问寄存器池(通过偏移增量)，从而实现索引寄存器。
// 该封装有助于组合这些功能。它还处理来自系统的通知和IBI(中断)请求。
// ------------------------------------------------------------------------------
// 实现细节
// ------------------------------------------------------------------------------
// 请参考自主规范和MIPI I3C规范
//
// 注意: 几乎不需要CDC(时钟域交叉)。系统应遵循正确的行为以避免读取数据的位撕裂
// (在读取过程中更改会导致位不匹配，因为来自不同的时钟域)。
// 与避免池撕裂(部分旧数据和部分新数据被读取)的方式相同。
// ----------------------------------------------------------------------------*/

`include "i3c_params.v" // 本地参数/常量

module i3c_auton_wrap_full #( // 参数由上层驱动，控制此模块的构建和操作
    // 1. 自主寄存器相关参数
    // 规则: [0]=W仅在RUN[idx]=0时有效, [1]=W在RUN[idx]=0时被忽略,
    // [2]=仅在RUN[idx]时触摸, [3]=在W而非R时触摸
    // [4]=在S(非Sr)时重置索引, [5]=魔法寄存器(总线)
    // [7:6]=DDR规则: 0:CMD=index, 1:CMD=index_except_7F, 2:CMD_ignored
    // (见REG_DDRCMD_WRIDX, 当[7:6]=2时设置索引)
    // [8]=读取无效寄存器时NACK, [15:9]=保留
    parameter REG_RULES = 16'd0, // 规则说明
    parameter MAX_REG = 8'h00, // 设置为1-255作为最后一个索引
    parameter REG_WRITABLE = {MAX_REG+1{1'b0}}, // 可写的寄存器
    parameter REG_READABLE = {MAX_REG+1{1'b0}}, // 可读的寄存器
    parameter REG_RUN = {MAX_REG+1{1'b0}}, // 不连续边界
    parameter REG_MASK = {((MAX_REG+1)*8){1'b1}}, // 全部或部分掩码
    parameter REG_BLEND = {MAX_REG+1{1'b0}}, // W/R混合
    parameter REG_REMAP = {((MAX_REG+1)*8){1'b1}}, // 可选重映射
    parameter REG_RESET = {((MAX_REG+1)*8){1'b0}}, // W寄存器的重置值
    parameter REG_DDRCMD_WRIDX= 7'h00, // 仅当REG_RULES[7:6]=2时使用
    parameter MAGIC_RULES = 4'b0000, // 规则位: [0]=RW vs. WO, [1]=CCCs, [2]=无索引
    parameter MAGIC_MASK = 8'h00, // 匹配掩码
    parameter MAGIC_MATCH = 8'h00, // 通过掩码匹配(索引&掩码)==匹配
    
    // 标准参数，即使不直接适用
    parameter ENA_ID48B = `ID48B_CONST, // 常量 vs. 寄存器
    parameter ID_48B = 48'h0, // 必须从上层填充，48位ID的部分或全部
    parameter ID_AS_REGS = 12'd0, // [0]=IDINST, [1]=ISRAND, [2]=IDDCR, [3]=IDBCR
    // 注意: BCR在下面，所以可以使用其他参数
    parameter ID_DCR = 8'd0, // 从上层填充DCR(如果不是来自寄存器)
    parameter ENA_SADDR = `SADDR_NONE, // 无, 常量, 寄存器/网络
    parameter SADDR_P = 0, // 7位(6:0)
    parameter ENA_IBI_MR_HJ = 0, // 0表示无事件，否则为事件掩码
    parameter CLK_SLOW_BITS = 6, // 用于Bus Avail计数所需的位数
    parameter CLK_SLOW_MATCH = 6'd47, // 计数: 例如48MHz时钟下1us为47(0相对)
    parameter CLK_SLOW_HJMUL = 10'd1000, // 1ms的MATCH数(常量或寄存器)(1相对)
    
    // 错误处理。如果启用IBI，则默认为Read Abort
    parameter ERROR_HANDLING = 3'd0|((|ENA_IBI_MR_HJ)<<`ERR_RDABT_b),
    parameter ENA_TIMEC = 6'b000010, // 如果寄存器, res, res, mode 1, mode 0, sync
    parameter TIMEC_FREQ_ACC = {8'd24,8'd10}, // 频率=12MHz (12.0=24) 1.0%精度
    parameter ENA_CCC_HANDLING= 6'd0, // 传递到支持的内容
    parameter RSTACT_CONFIG = 26'd0, // 从设备重置RSTACT CCC配置(见RSTA_xxx_b字段)
    parameter MAX_RDLEN = 0, // 默认S->M长度最大值
    parameter MAX_WRLEN = 0, // 默认M->S长度最大值
    parameter MAX_DS_WR = 0, // M->S的数据速度限制
    parameter MAX_DS_RD = 0, // S->M的数据速度限制
    parameter MAX_DS_RDTURN = 0, // S->M读取请求的延迟需求
    parameter ENA_HDR = 0, // HDR模式启用
    parameter PIN_MODEL = `PINM_COMBO, // 组合式引脚使用
    // BCR从参数自动填充。但不能离线或桥接
    parameter ID_BCR = (|ENA_HDR<<5) | ((ENA_IBI_MR_HJ&8'h03)<<1) | |(MAX_DS_RDTURN|MAX_DS_WR|MAX_DS_RD),
    // 下面不传递 - 在此处计算
    parameter priv_sz = PIN_MODEL[1] ? (ENA_HDR[0] ? 3 : 1) : 0 // 如果ext-pad+ddr则更宽
) (
    // 定义时钟和复位输入
    input RSTn, // 系统复位
    input CLK, // 用于寄存器和同步的系统时钟
    input CLK_SLOW, // 用于IBI的强制慢速时钟
    output slow_gate, // 1表示可能门控CLK_SLOW
    input CLK_SLOW_TC, // 时序控制时钟，可能与CLK_SLOW相同
    output tc_slow_gate, // 1表示CLK_SLOW_TC可能被门控；与slow_gate逻辑与
    // 定义引脚。SCL仅输入
    input pin_SCL_in, // SCL: 通常为时钟输入，三态时为数据
    input pin_SDA_in, // SDA: M->S
    // 注意: 以下3个在PIN_MODEL==`PINM_EXT_REG时特殊使用
    // 它们为靠近引脚的3个寄存器提供输入
    output pin_SDA_out, // SDA: S->M在读取时
    output pin_SDA_oena, // SDA: S->M在读取时
    output [priv_sz:0] pin_SDA_oena_rise, // 仅在EXT_REG时在SCL上升沿特殊，否则为0
    // 仅当SDA和SCL引脚有i2c 50ns尖峰滤波器时使用
    // 并且可以通过网络打开/关闭
    output i2c_spike_ok, // 1表示允许i2c尖峰滤波器
    
    // 自主使用: 通知的o_网络同步到CLK域，来自SCL。
    // 注意: start和done是此从设备的消息(但不是CCC)，在至少一个CLK周期内为1。
    // wo_regs和来自SCL的raw(因此应在使用通知时安全采样)，
    // ro_regs假设来自CLK(因此应在使用通知时安全更改)。
    // touch网络是每比特原始数据。系统应使用提供的同步模块处理本地同步。
    // 这属于系统，因此从Q到使用的路径保持较短，以防止亚稳态问题。
    // 注意: touch可能是每寄存器或每运行(由REG_RULES控制)。
    // OK(完成与纯ACK)每比特从CLK域脉冲。
    // 脉冲必须持续到所需的寄存器复位时间(通常是异步且非常短，因此通常不是问题)。
    output osync_started, // 1 CLK周期，当新传输开始时
    output o_read, // 1如果开始从我们读取，否则为写
    output osync_done, // 1 CLK周期，读或写结束
    output osync_any_touch, // 1如果任何寄存器读取或写入，当osync_done=1时
    output [MAX_REG:0] oraw_reg_touch, // 如果寄存器或运行已读/写，则位为1
    input [MAX_REG:0] iraw_touch_OK, // 系统脉冲接受touch[i]，来自CLK域: 作为复位
    output [(8*MAX_REG)+7:0] wo_regs, // 主机写入的寄存器 - 原始
    input [(8*MAX_REG)+7:0] ro_regs, // 系统提供的寄存器，由主机读取 - 原始
    
    // 与自主相关的中断请求。请求来自CLK域，并在本地同步。
    // 输出done也同步到CLK域。
    // 如果o_ibi_done脉冲，IBI已通过并被接受(包括字节如果使用)。
    // 如果o_ibi_nacked脉冲，系统必须决定是否尝试再次。
    input i_ibi_event, // 1 CLK脉冲，表示事件
    input i_ibi_req, // 保持1请求IBI，通过完成(ACK)或取消清除
    input i_hj_req, // 保持1请求HJ(如果允许)；通过ibi_done清除
    output o_ibi_done, // 1 CLK周期，IBI完成并ACK
    output o_ibi_nacked, // 1 CLK周期，IBI完成并NACK
    input [7:0] i_ibi_byte, // IBI字节，如果需要。保持直到完成
    
    // 现在是配置(稳定)输入。这些应根据参数设置为0或1(当未使用时)。
    input cf_SlvEna, // 仅当为1时处理总线
    input [7:0] cf_SlvSA, // 可选从设备i2c地址([0]=1如果启用)
    input [3:0] cf_IdInst, // ID选择时的实例(如果未使用partno)
    input cf_IdRand, // 如果使用，ID的随机partno
    input [31:0] cf_Partno, // 如果使用，ID的partno
    input [7:0] cf_IdBcr, // DAA的BCR如果不是常量
    input [7:0] cf_IdDcr, // DAA的DCR如果不是常量
    input [14:0] cf_IdVid, // 如果不是常量，供应商ID
    
    // 下一个只能在总线空闲时设置/清除，除非在SCL时钟域内完成
    // 0表示不需要
    input cf_SlvNack, // 设置为1以导致我们的地址NACK(R或W)
    
    // 以下仅在param中设置EV_BAMATCH_b时使用。可能动态更改
    input [7:0] cf_BAMatch, // 现在SCL域的特殊状态 - 这些可以即时同步用于STATUS读取
    
    // 以下输出为raw，这意味着要么在总线STOP时更改，要么不进行GETSTATUS或同步。
    // 它们可以设置为0(如果不需要)。
    output [29:28] raw_ActState, // 从ENTASn获取的总线活动状态
    output [27:24] raw_EvState, // 从ENEC/DISEC获取的事件使能状态
    output [6:0] raw_Request, // 正在处理的总线请求，带详细信息
    output [7:0] raw_DynAddr, // 如果设置，动态地址 - 稳定
    output [13:0] raw_timec_sync, // SYNC时间控制由系统处理。[13]=clk
    output [3:0] raw_slvr_reset, // SlaveRst控制
    input iraw_rst_slvr_reset, // SCL域中SlaveRst控制的清除
    
    // 这些接下来的是安全raw。这意味着在总线STOP时更改，或不进行GETSTATUS或同步。
    // 它们可以设置为0(如果不需要)。
    input [7:6] sraw_ActMode, // 系统活动模式(如果未使用则为0)
    input [3:0] sraw_PendInt, // 待处理中断(如果未使用则为0)
    input [15:8] sraw_StatusRes, // 状态的保留位(如果未使用则为0)
    
    // 下面是检测到的错误(例如通过运行结束)
    // 它们可以被忽略。简单在这里如果需要
    // 这些在SCL域是raw，所以如果需要，需要是粘性或握手
    output raw_tb_reg_err,
    output raw_fb_reg_err,
    
    // 现在魔法寄存器，意味着使用基于SCL的总线映射到系统。
    // 如果不使用这个，通常会使用注册autonomous_wrap。
    output mr_clk_SCL, // 我们使用的SCL时钟(另见SCL_n)
    output mr_clk_SCL_n, // 反相作为有效时钟(当不扫描时)
    output [7:0] mr_idx, // 当前寄存器索引(自动递增)
    output [7:0] mr_next_idx, // 传输完成后下一个索引
    output mr_dir, // 0表示写，1表示读(注意时序)
    output mr_trans_w_done, // 脉冲1周期；写数据结束(不是索引)
    output mr_trans_r_done, // 脉冲1周期；读数据结束
    output mr_trans_ddrcmd, // 脉冲1周期；DDR CMD字节结束
    output mr_trans_ccc_done, // 如果启用: 脉冲1周期，与_w_done相同
    output [7:0] mr_wdata, // mr_dir=0且trans_w_done=1时的数据
    input [7:0] mr_rdata, // 系统的实时数据 - 在trans_r_done时更改
    input mr_rdata_none, // 1表示无读取数据 - 如果是第一个则NACK
    input mr_rdata_end, // 读取时为1，如果是最后一个字节
    
    // 注意: 以下信号与osync相同，但未同步 - 在SCL域
    output mr_bus_new, // 在匹配到我们时脉冲(注意延迟)
    output mr_bus_done, // 在消息结束时脉冲
    
    // 现在是扫描相关
    input scan_single_clock, // 从scan_clk的单时钟域，用于基于引脚的时钟
    input scan_clk, // 用于扫描中引脚时钟的时钟
    input scan_no_rst, // 防止分层复位
    input scan_no_gates // 防止架构时钟门控
);

// 以下线用于从CRTL寄存器启动事件
wire [2:0] event_pending;
wire force_sda;
wire ibi_has_byte;
wire [7:0] opt_ibi_byte;
wire [7:0] timec_ibi_byte;
wire ibi_timec; // 1如果时间控制开启
wire [1:0] ibi_timec_marks; // SC1和SC2停止 - 脉冲
wire [2:0] ibi_timec_sel; // 输出选择器
wire timec_oflow;
wire timec_oflow_clr;
wire hold_engine; // 在HJ期间保持

// 以下两个导出到上层，如果需要
wire clk_SCL;
wire clk_SCL_n;

// 以下用于事务的线
wire [7:0] tb_datab;
wire tb_datab_ack;
wire tb_end;
wire tb_urun_nack;
wire tb_urun;
wire tb_term;
wire [7:0] fb_datab;
wire fb_datab_done;
wire fb_datab_err;
wire [2:0] fb_ddr_errs; // HDR帧, 奇偶校验, CRC
wire fb_s0s1_err; // S0/S1错误锁定(退出清除)
wire abt_s0s1_err;
wire read_abort;
wire fb_orun;
wire tx_valid; // 用于NACK头例如
wire tb_err; // 中断和来自SCL域的状态

wire int_start_seen;
wire int_start_err;
wire int_da_matched;
wire int_sa_matched;
wire int_7e_matched;
wire int_ddr_matched;
wire int_in_STOP;
wire int_event_sent;
wire int_event_ack;
wire int_in_evproc;
wire int_ccc_handled;
wire int_ccc;
wire state_in_ccc;
wire state_in_daa;
wire state_in_hdr;
wire state_dir;

reg [1:0] bus_start;
reg bus_ddr;

wire willbe_ccc;
wire is_ccc = (willbe_ccc|state_in_ccc) & ~state_in_hdr;

// 从设备实例处理SCL域中从设备的所有方面
i3c_slave_wrapper #(.ENA_ID48B(ENA_ID48B), .ID_48B(ID_48B),
    .ID_AS_REGS(ID_AS_REGS), .ID_BCR(ID_BCR), .ID_DCR(ID_DCR),
    .ENA_SADDR(ENA_SADDR), .SADDR_P(SADDR_P),
    .ENA_MAPPED(0), .MAP_CNT(1), .MAP_I2CID(24'd0), .MAP_DA_AUTO(0),
    .MAP_DA_DAA(0), .ENA_TIMEC(ENA_TIMEC), .TIMEC_FREQ_ACC(TIMEC_FREQ_ACC),
    .ENA_IBI_MR_HJ(ENA_IBI_MR_HJ), .ENA_CCC_HANDLING(ENA_CCC_HANDLING),
    .RSTACT_CONFIG(RSTACT_CONFIG), .MAX_RDLEN(MAX_RDLEN),
    .MAX_WRLEN(MAX_WRLEN), .MAX_DS_WR(MAX_DS_WR), .MAX_DS_RD(MAX_DS_RD),
    .MAX_DS_RDTURN(MAX_DS_RDTURN), .ENA_HDR(ENA_HDR), .ENA_MASTER(0),
    .PIN_MODEL(PIN_MODEL))
slave (
    .RSTn (RSTn),
    .pin_SCL_in (pin_SCL_in),
    .pin_SCL_out (),
    .pin_SCL_oena (),
    .pin_SDA_in (pin_SDA_in),
    .pin_SDA_out (pin_SDA_out),
    .pin_SDA_oena (pin_SDA_oena),
    .pin_SDA_oena_rise(pin_SDA_oena_rise),
    .i2c_spike_ok (i2c_spike_ok),
    .i2c_hs_enabled (),
    .slv_enable (cf_SlvEna & ~hold_engine),
    .int_start_seen (int_start_seen),
    .int_start_err (int_start_err),
    .int_da_matched (int_da_matched),
    .int_sa_matched (int_sa_matched),
    .int_7e_matched (int_7e_matched),
    .int_ddr_matched (int_ddr_matched),
    .int_in_STOP (int_in_STOP),
    .int_event_sent (int_event_sent),
    .int_event_ack (int_event_ack),
    .int_in_evproc (int_in_evproc),
    .int_ccc_handled (int_ccc_handled),
    .int_ccc (int_ccc),
    .state_req (raw_Request),
    .opt_state_AS (raw_ActState),
    .opt_ev_mask (raw_EvState),
    .opt_TimeC (raw_TimeC),
    .opt_timec_sync (raw_timec_sync[12:0]),
    .opt_slvr_reset (raw_slvr_reset),
    .opt_match_idx (),
    .i_rst_slvr_reset (iraw_rst_slvr_reset),
    .clr_slvr_cause (1'b0),
    .state_in_ccc (state_in_ccc),
    .state_in_daa (state_in_daa),
    .state_in_hdr (state_in_hdr),
    .state_dir (state_dir),
    .dyn_addr (raw_DynAddr),
    .dyn_addr_chg (),
    .dyn_chg_cause (),
    .opt_static_addr ((ENA_SADDR==`SADDR_CONST) ? {SADDR_P[6:0],1'b1} : (ENA_SADDR==`SADDR_CONFIG)? cf_SlvSA : (ENA_SADDR==`SADDR_NET) ? cf_SlvSA : 8'h0),
    .event_pending (event_pending),
    .force_sda (force_sda),
    .ibi_has_byte (ibi_has_byte),
    .opt_ibi_byte (opt_ibi_byte),
    .ibi_timec (ibi_timec),
    .ibi_timec_marks (ibi_timec_marks),
    .ibi_timec_sel (ibi_timec_sel),
    .ibi_extdata (1'b0),
    .ibi_mapidx (4'b0),
    .ibi_in_extdata (),
    .timec_oflow (timec_oflow),
    .timec_oflow_clr (timec_oflow_clr),
    .oclk_SCL (clk_SCL),
    .oclk_SCL_n (clk_SCL_n),
    .tb_data_valid (tx_valid),
    .tb_datab (tb_datab),
    .tb_end (tb_end),
    .tb_datab_ack (tb_datab_ack),
    .tb_urun_nack (tb_urun_nack),
    .tb_urun (tb_urun),
    .tb_term (tb_term),
    .fb_data_use (1'b1),
    .fb_datab (fb_datab),
    .fb_datab_done (fb_datab_done),
    .fb_datab_err (fb_datab_err),
    .fb_orun (fb_orun),
    .fb_ddr_errs (fb_ddr_errs),
    .fb_s0s1_err (fb_s0s1_err),
    .brk_s0s1_err (abt_s0s1_err),
    .rd_abort (read_abort),
    .fb_hdr_exit (),
    .obit_cnt (),
    .ccc_byte (),
    .willbe_ccc (willbe_ccc),
    .cf_IdInst (cf_IdInst),
    .cf_IdRand (cf_IdRand),
    .cf_Partno (cf_Partno),
    .cf_IdBcr (cf_IdBcr),
    .cf_IdDcr (cf_IdDcr),
    .cf_IdVid (cf_IdVid),
    .cf_MaxRd (12'd0),
    .cf_MaxWr (12'd0),
    .cf_RstActTim ({RSTACT_CONFIG[`RSTA_CUS_b] ? RSTACT_CONFIG[`RSTA_TIM_PER_b] : 8'd0, RSTACT_CONFIG[`RSTA_TIM_SYS_b], RSTACT_CONFIG[`RSTA_TIM_PER_b]}),
    .cf_SlvNack (cf_SlvNack),
    .cf_SdrOK (1'b1),
    .cf_DdrOK (1'b0),
    .cf_TspOK (1'b0),
    .cf_TslOK (1'b0),
    .cf_TCclk (16'd0),
    .cf_s0ignore (1'b0),
    .cf_SetDA (8'd0),
    .cf_SetMappedDASA (10'd0),
    .cf_SetSA10b (3'd0),
    .cf_HdrCmd (2'b00),
    .cf_vgpio (9'd0),
    .cf_CccMask (7'b1111111),
    .cf_MasterAcc (1'b0),
    .opt_MasterAcc (),
    .map_daa_use (1'b0),
    .map_daa_dcr (8'd0),
    .map_daa_pid (2'd0),
    .raw_vgpio_done (),
    .raw_vgpio_byte (),
    .raw_hdr_newcmd (),
    .raw_hdr_cmd (),
    .raw_setaasa (),
    .raw_rstdaa (),
    .raw_daa_ena (),
    .map_sa_idx (),
    .map_daa_da (),
    .i2c_dev_rev (3'd0),
    .i2c_sw_rst (),
    .sraw_ActMode (sraw_ActMode),
    .sraw_PendInt (sraw_PendInt),
    .sraw_StatusRes (sraw_StatusRes),
    .opt_ChgMaxRd (),
    .opt_ChgMaxWr (),
    .opt_MaxRdWr (),
    .slv_debug_observ (),
    .scan_single_clock(scan_single_clock),
    .scan_clk (scan_clk),
    .scan_no_rst (scan_no_rst),
    .scan_no_gates (scan_no_gates)
);

assign raw_timec_sync[13] = clk_SCL;

// 自主寄存器是基于参数构建的寄存器集
i3c_autonomous_reg #(.REG_RULES(REG_RULES), .MAX_REG(MAX_REG),
    .REG_WRITABLE(REG_WRITABLE), .REG_READABLE(REG_READABLE),
    .REG_RUN(REG_RUN), .REG_MASK(REG_MASK), .REG_BLEND(REG_BLEND),
    .REG_REMAP(REG_REMAP), .REG_RESET(REG_RESET),
    .REG_DDRCMD_WRIDX(REG_DDRCMD_WRIDX), .MAGIC_RULES(MAGIC_RULES),
    .MAGIC_MASK(MAGIC_MASK), .MAGIC_MATCH(MAGIC_MATCH))
autonomous_reg (
    .clk_SCL_n (clk_SCL_n),
    .clk_SCL (clk_SCL),
    .CLK (CLK),
    .RSTn (RSTn),
    .bus_new1 (bus_ddr ? bus_start[0] : bus_start[1]),
    .bus_read (state_dir),
    .bus_ddr (bus_ddr),
    .bus_in_ddr (raw_Request[6]),
    .bus_done1 (int_start_seen),
    .bus_in_STOP (int_in_STOP),
    .fb_new1 (fb_datab_done & (~is_ccc | MAGIC_RULES[1])),
    .fb_data (fb_datab),
    .fb_is_ccc (is_ccc & MAGIC_RULES[1]),
    .fb_err (raw_fb_reg_err),
    .tb_need1 (tb_datab_ack),
    .tb_data (tb_datab),
    .tb_end (tb_end),
    .tb_err (tb_err),
    .tx_valid (tx_valid),
    .osync_started (osync_started),
    .o_read (o_read),
    .osync_done (osync_done),
    .osync_any_touch (osync_any_touch),
    .oraw_reg_touch (oraw_reg_touch),
    .iraw_touch_OK (iraw_touch_OK),
    .wo_regs (wo_regs),
    .ro_regs (ro_regs),
    .mr_idx (mr_idx),
    .mr_next_idx (mr_next_idx),
    .mr_dir (mr_dir),
    .mr_trans_w_done (mr_trans_w_done),
    .mr_trans_r_done (mr_trans_r_done),
    .mr_trans_ddrcmd (mr_trans_ddrcmd),
    .mr_trans_ccc_done(mr_trans_ccc_done),
    .mr_wdata (mr_wdata),
    .mr_rdata (mr_rdata),
    .mr_rdata_none (mr_rdata_none),
    .mr_rdata_end (mr_rdata_end),
    .scan_no_rst (scan_no_rst)
);

assign mr_clk_SCL = clk_SCL;
assign mr_clk_SCL_n= clk_SCL_n;
assign mr_bus_new = bus_ddr ? bus_start[0] : bus_start[1];
assign mr_bus_done = int_start_seen;

// 下面是延迟，所以读取在相同时间已知
always @(posedge clk_SCL or negedge RSTn) begin
    if (!RSTn) begin
        bus_start <= 2'b00;
        bus_ddr <= 1'b0;
    end else if ((int_da_matched|int_sa_matched) & ~is_ccc & // SDR
                (~state_dir | tx_valid)) // 如果读取，有数据
        bus_start <= 2'b01;
    else if (int_ddr_matched) begin // DDR如果启用
        bus_ddr <= 1'b1;
        bus_start <= 2'b01;
    end else if (bus_start == 2'b01)
        bus_start <= 2'b10;
    else begin
        bus_start <= 2'b00;
        bus_ddr <= 1'b0;
    end
end

// 下面信号也表示错误，用于SDR/DDR读取和CCC GET的下溢NACK
assign raw_tb_reg_err = tb_err | (state_dir&(int_da_matched|int_sa_matched)&~tx_valid);

// 事件管理在CDC中
// 该部分将事件请求在CLK和SCL之间转换
generate
    if (ENA_IBI_MR_HJ != 0) begin : sync_events
        // 我们简单地将i_ibi_req从CLK转换到CLK_SLOW域。
        // ibi_byte是稳定/保持的，所以没有问题。
        // IBI请求仅在STOP到START时发出。
        // 但总线可用的停止期间可用于强制开始(保持SDA低)。
        // 他们必须在ACK(完成)或放弃时清除请求。
        wire syn_ibi_req, syn_hj_req;
        SYNC_S2B sync_ibi_req(.rst_n(RSTn), .clk(CLK_SLOW), .scl_data(i_ibi_req), .out_clk(syn_ibi_req));
        
        if (ENA_IBI_MR_HJ[`EV_HJ_b]) begin : hj_syn
            SYNC_S2B sync_hj_req(.rst_n(RSTn), .clk(CLK_SLOW), .scl_data(i_hj_req), .out_clk(syn_hj_req));
        end else
            assign syn_hj_req = 1'b0;
        
        // 我们可以通过ACK或NACK完成
        SYNC_ASelfClr_S2C_Seq2 sync_ibi_done(.SCL(clk_SCL_n), .CLK(CLK), .RSTn(RSTn), .local_set(int_event_sent&int_event_ack), .o_pulse(o_ibi_done));
        SYNC_ASelfClr_S2C_Seq2 sync_ibi_nack(.SCL(clk_SCL_n), .CLK(CLK), .RSTn(RSTn), .local_set(int_event_sent&~int_event_ack), .o_pulse(o_ibi_nacked));
        
        wire allow_ev = (syn_hj_req & raw_EvState[27]) | (syn_ibi_req & raw_EvState[24]);
        
        // 下面是掩码，所以如果事件完成，我们停止请求 - 允许CLK_SLOW赶上并关闭
        // 我们使用事件消失时的重置来移除掩码。
        reg ev_maskoff;
        wire rst_ev_maskoff_n = RSTn & (allow_ev | scan_no_rst);
        
        always @(posedge clk_SCL_n or negedge rst_ev_maskoff_n) begin
            if (!rst_ev_maskoff_n)
                ev_maskoff <= 1'b0; // 无掩码，除非启用 - 事件请求清除时清除
            else if (int_event_sent&int_event_ack)
                ev_maskoff <= 1'b1; // 掩码事件直到请求清除
        end
        
        assign event_pending = (~allow_ev | ev_maskoff) ? 3'b000 :
            (syn_hj_req) ? 3'b111 :
            {(syn_ibi_req&int_in_STOP),(syn_ibi_req ? 2'b01 : 2'b00)};
        
        // 下面是IBI、MR、S0/S1分解、读取停滞检测和HJ的计数器
        wire syn_s0s1;
        wire syn_inread;
        wire raw_rd_abort;
        wire sync_gate;
        
        // S0S1错误同步到慢速时钟
        SYNC_S2C sync_s0s1_err(.rst_n(RSTn), .clk(CLK_SLOW), .scl_data(fb_s0s1_err), .out_clk(syn_s0s1));
        
        // 在读取且不是i2c时同步到慢速时钟
        SYNC_S2C sync_in_read(.rst_n(RSTn), .clk(CLK_SLOW), .scl_data(raw_DynAddr[0]&raw_Request[3]), .out_clk(syn_inread));
        
        // 读取中止导致状态机重置，所以不需要同步
        assign read_abort = raw_rd_abort & raw_Request[3];
        
        // 慢速门必须允许同步
        // 因此是同步和非同步的组合
        wire raw_gate = ~(raw_DynAddr[0]&raw_Request[3]) & ~fb_s0s1_err & ~i_ibi_req & ~(ENA_IBI_MR_HJ[`EV_HJ_b]&i_hj_req);
        wire local_syn_gate = ~syn_ibi_req & ~syn_hj_req & ~syn_s0s1;
        assign slow_gate = sync_gate & raw_gate & local_syn_gate;
        
        i3c_slow_counters #(.ENA_IBI_MR_HJ(ENA_IBI_MR_HJ), .CLK_SLOW_BITS(CLK_SLOW_BITS),
            .CLK_SLOW_MATCH(CLK_SLOW_MATCH), .CLK_SLOW_HJMUL(CLK_SLOW_HJMUL),
            .ENA_TIMEC(ENA_TIMEC), .TIMEC_FREQ_ACC(TIMEC_FREQ_ACC),
            .ENA_CCC_HANDLING(ENA_CCC_HANDLING))
        counters (
            .RSTn (RSTn),
            .clk_SCL_n (clk_SCL_n),
            .clk_SCL (clk_SCL),
            .CLK_SLOW (CLK_SLOW),
            .slow_gate (sync_gate),
            .cf_SlvEna (cf_SlvEna),
            .cf_BAMatch (cf_BAMatch),
            .event_pending (event_pending),
            .pin_SCL_in (pin_SCL_in),
            .pin_SDA_in (pin_SDA_in),
            .run_60 (syn_s0s1),
            .run_100 (syn_inread),
            .int_in_STOP (int_in_STOP),
            .force_sda (force_sda),
            .done_60 (abt_s0s1_err),
            .done_100 (raw_rd_abort),
            .hold_engine (hold_engine)
        );
        
        // 下面拉入时间控制(如果启用，仅当IBI时)
        if (|ENA_TIMEC) begin : timec_handling
            wire ev_pend;
            wire time_oflow;
            wire syn_tc_gate;
            
            // 同步到SLOW时钟从CLK - 1位作为真或假
            // 保持信号同步
            SYNC_C2S #(1) sync_IBI_pend(.rst_n(RSTn), .scl(CLK_SLOW_TC), .clk_data(event_pending[0]), .out_scl(ev_pend));
            SYNC_AClr_C2S sync_tcoflow(.CLK(CLK_SLOW_TC), .RSTn(RSTn), .local_set(time_oflow), .async_clear(timec_oflow_clr), .o_value (timec_oflow));
            
            // CLK_SLOW_TC不被门控如果需要用于同步和结果
            // 注意timec_oflow等待清除(在定时设置时)
            assign tc_slow_gate = syn_tc_gate & ~event_pending[0] & ~timec_oflow;
            
            i3c_time_control time_ctrl(
                .RSTn (RSTn),
                .CLK_SLOW (CLK_SLOW_TC),
                .clk_SCL_n (clk_SCL_n),
                .timec_ena (raw_TimeC), // 启用的内容 - one hot
                .event_start (ev_pend), // 开始 - 保持为真
                .was_nacked (o_ibi_nacked), // 如果NACKed
                .sc1_stop (ibi_timec_marks[0]), // SC1和SC2边缘上的标记
                .sc2_stop (ibi_timec_marks[1]),
                .time_info_sel (ibi_timec_sel), // 选择字节输出 >=1/2周期提前
                .time_info_byte(timec_ibi_byte), // 选择的输出字节
                .ibi_timec (ibi_timec), // 现在触发IBI
                .time_overflow (time_oflow), // 1如果定时器溢出
                .slow_gate (syn_tc_gate), // CLK_SLOW_TC的门控控制
                .scan_no_rst (scan_no_rst)
            );
        end else begin
            assign tc_slow_gate = 1; // 从未使用
            assign timec_ibi_byte = 8'd0;
            assign ibi_timec = 1'b0;
            assign timec_oflow = 1'b0;
        end
        
    end else begin // 无IBI, MR, HJ
        assign event_pending = 3'd0;
        assign force_sda = 1'b0;
        assign hold_engine = 1'b0;
        assign slow_gate = 1'b0;
        assign tc_slow_gate = 1'b0;
        assign o_ibi_done = 1'b0;
        assign o_ibi_nacked = 1'b0;
        assign abt_s0s1_err = 1'b0;
        assign read_abort = 1'b0;
        assign ibi_timec = 1'b0;
    end
    
    if (ENA_IBI_MR_HJ[`EV_IBI_DAT_b]) begin : ibi_byte
        assign ibi_has_byte = 1'b1;
        if (|ENA_TIMEC)
            assign opt_ibi_byte = ~|ibi_timec_sel[1:0] ?
                // IBI字节始终首先使用，如果BCR要求则强制
                (i_ibi_byte | (ibi_timec?8'h80:8'h00)) :
                // 时间c字节通过多路选择器改变
                timec_ibi_byte;
        else
            assign opt_ibi_byte = i_ibi_byte;
    end else begin
        assign ibi_has_byte = 1'b0;
        assign opt_ibi_byte = 8'd0;
    end
    
endgenerate

// 以下逻辑是一种断言形式，用于捕获错误参数或组合
// 仿真或综合编译器将在超出范围时发出警告
// ID必须是一些常量的混合，如果有一些寄存器，则多路选择器必须匹配
// 注意: 即使是寄存器，VID也不能为0，因为这形成了重置值
localparam ID_TST = ~|ENA_ID48B || ENA_ID48B > `ID48B_CONST_NONE || ID_48B == 48'd0 ||
                   (ENA_ID48B <= `ID48B_CONST_INST && ID_48B[32]);
wire [0:0] bad_id;
assign bad_id[ID_TST] = 1'b0;

localparam IAR_TST = ID_AS_REGS >= 32 ||
                    ID_AS_REGS[`IDREGS_INST_b] & (ENA_ID48B != `ID48B_CONST_INST) ||
                    ID_AS_REGS[`IDREGS_RAND_b] & (ENA_ID48B != `ID48B_CONST_PARTNO) ||
                    ID_AS_REGS[`IDREGS_VID_b] & (ENA_ID48B != `ID48B_CONST_NONE);
wire [0:0] bad_id_as_reg;
assign bad_id_as_reg[IAR_TST] = 1'b0;

// 注意: 无法检查DCR，因为它允许为0
// 以下说明如果常量BCR，则必须有效
localparam BCR_TST = ~ID_AS_REGS[`IDREGS_BCR_b] &
                    (ID_BCR[7:6] != 2'b00 || // 必须作为从设备匹配
                     ////ID_BCR[5] != |ENA_HDR || -- OK 因为可以使用DDR_OK
                     ID_BCR[1] != ENA_IBI_MR_HJ[`EV_IBI_b] ||
                     ID_BCR[2] != ENA_IBI_MR_HJ[`EV_IBI_DAT_b] ||
                     (ENA_IBI_MR_HJ[`EV_IBI_DAT_b] & ~ENA_IBI_MR_HJ[`EV_IBI_b]) ||
                     (|ENA_TIMEC & ~ENA_IBI_MR_HJ[`EV_IBI_DAT_b]));
wire [0:0] bad_bcr;
assign bad_bcr[BCR_TST] = 1'b0;

// 现在静态地址
localparam SA_INV = SADDR_P < 3 || SADDR_P >= 8'h7E ||
                   SADDR_P == 8'h7C || SADDR_P == 8'h7A || SADDR_P == 8'h76 ||
                   SADDR_P == 8'h6E || SADDR_P == 8'h5E || SADDR_P == 8'h3E;
localparam SA_TST = ENA_SADDR > `SADDR_CONFIG ||
                   (ENA_SADDR != `SADDR_CONST & |SADDR_P) || // 不允许
                   (ENA_SADDR == `SADDR_CONST & SA_INV);
wire [0:0] bad_sa;
assign bad_sa[SA_TST] = 1'b0;

// 现在慢速时钟
localparam CLK_TST = (|(CLK_SLOW_BITS|CLK_SLOW_MATCH) & ~|ENA_IBI_MR_HJ) ||
                    CLK_SLOW_MATCH >= (1 << CLK_SLOW_BITS);
wire [0:0] bad_clk;
assign bad_clk[CLK_TST] = 1'b0;

// CCC处理
localparam MAXES = MAX_RDLEN | MAX_WRLEN;
localparam ALLMX = MAXES | MAX_DS_WR | MAX_DS_RD | MAX_DS_RDTURN;
localparam CCC_TST = ENA_CCC_HANDLING > 6'h3F ||
                    MAXES > 16'hFFFF ||
                    (~ENA_CCC_HANDLING[`ENCCC_MAXES_b] & |ALLMX) ||
                    (ENA_CCC_HANDLING[`ENCCC_MAXES_b] &
                          (MAX_DS_WR > 7 || MAX_DS_RD > 63 || MAX_DS_RDTURN >= (1 << 24)));
wire [0:0] bad_ccc;
assign bad_ccc[CCC_TST] = 1'b0;

// 时间控制 - 如果同步则需要异步模式0 - 对于从设备也是如此
localparam TIMEC_TST = (ENA_TIMEC[2] & ~ENA_TIMEC[1]) |
                      (ENA_TIMEC[0] & ~ENA_TIMEC[1]) |
                      (|ENA_TIMEC[5:3]);
wire [0:0] bad_timec;
assign bad_timec[TIMEC_TST] = 1'b0;

// 总线信息 - 自主模式无
// FIFO类型 - 自主模式无

// 引脚模型
localparam PINMODEL_TST = PIN_MODEL > 2;
wire [0:0] bad_pin_model;
assign bad_pin_model[PINMODEL_TST] = 1'b0;

// HDR仅允许DDR，但现在不允许TSP或TSL
localparam HDR_TST = |ENA_HDR[2:1];
wire [0:0] bad_hdr;
assign bad_hdr[HDR_TST] = 1'b0;

// 魔法寄存器必须启用
localparam MAG_TST = |(MAGIC_RULES|MAGIC_MASK|MAGIC_MATCH) & ~REG_RULES[5];
wire [0:0] bad_magic;
assign bad_magic[MAG_TST] = 1'b0;

endmodule