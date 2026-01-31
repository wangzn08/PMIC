/*--------------------------------------------------------------------
  此模块为 MIPI I3C 内存映射寄存器，供本地处理器通过 APB 或类 APB 总线访问。
  根据参数配置生成实际寄存器。
  注意：
  1. 写操作不在第一个周期使用 PWDATA，因此可方便地映射自 AHB 等两相总线。
  2. 读操作依赖两个周期，因为第一个周期用于锁存可能亚稳态的状态位（预读）。
     若需单周期读，则应屏蔽这些位或确保不组合使用。
  -------------------------------------------------------------------- */

`include "i3c_params.v"                 // 本地参数/常量

module i3c_regs #(
    // 参数由上层驱动，用于控制模块构建和操作方式
    parameter ENA_ID48B       = `ID48B_CONST, // ID 类型：常量或寄存器
    parameter  ID_48B         = 48'h0,  // 48位ID（由上层填入部分或全部）
    parameter ID_AS_REGS      = 12'd0,  // 用于ID和其他用途的寄存器（如掩码）
    parameter  ID_BCR         = 8'd0,   // 若非寄存器，则填入BCR
    parameter  ID_DCR         = 8'd0,   // 若非寄存器，则填入DCR
    parameter ENA_SADDR       = `SADDR_NONE, // 从地址配置：无、常量、寄存器/网络
    parameter  SADDR_P        = 0,      // 7位从地址，位置为7:1
    parameter ENA_MAPPED      = 5'd0,   // 是否允许额外的动态/静态地址及相关配置
    parameter  MAP_CNT        = 4'd1,   // 允许的额外动态/静态地址数量
    parameter  MAP_I2CID      = 24'd0,  // 若I2C扩展设备ID不为0
    parameter  MAP_DA_AUTO    = {5'd0,1'b0,1'b0,3'd0,1'b0,1'b0,1'b0},
    parameter  MAP_DA_DAA     = 0,      // 若非MMR且PID/DCR不为0，则为位数组
    parameter ENA_IBI_MR_HJ   = 0,      // 事件使能掩码（0表示无事件）
    parameter  CLK_SLOW_BITS  = 6,      // 总线空闲计数器的位宽
    parameter  CLK_SLOW_MATCH = 6'd47,  // 计数值（例如48MHz时钟下47对应1us）
    parameter  CLK_SLOW_HJMUL = 10'd1000,// 用于1ms的匹配次数（常量或寄存器）
    parameter  ERROR_HANDLING = 3'd0,
    parameter ENA_CCC_HANDLING= 6'd0,   // 支持的CCC处理配置
    parameter RSTACT_CONFIG   = 26'd0,  // 从机复位RSTACT CCC配置
    parameter MAX_RDLEN       = 0,      // 默认从机->主机最大读取长度
    parameter MAX_WRLEN       = 0,      // 默认主机->从机最大写入长度
    parameter MAX_DS_WR       = 0,      // 主机->从机数据速率限制
    parameter MAX_DS_RD       = 0,      // 从机->主机数据速率限制
    parameter MAX_DS_RDTURN   = 0,      // 从机->主机读请求延迟需求
    parameter SEL_BUS_IF      = 0,      // 总线接口支持类型
    parameter FIFO_TYPE       = 0,      // FIFO宽度（若使用）
    parameter  EXT_FIFO       = 3'd0,   // 外部FIFO选择（非0表示启用）
    parameter  ENA_TOBUS_FIFO = 0,      // 发送至总线的FIFO深度
    parameter  ENA_FROMBUS_FIFO=0,      // 从总线接收的FIFO深度
    parameter ENA_HDR         = 0,      // HDR模式使能
    parameter ENA_TIMEC       = 6'b000010,    // 时钟控制寄存器配置
    parameter TIMEC_FREQ_ACC  = {8'd24,8'd10},// 频率精度（如12MHz，精度1.0%）
    parameter BLOCK_ID        = 0,      // 若非0，在地址FFC处返回ID
    parameter ENA_MASTER      = 0,      // 为1时支持主机模式
    parameter [7:0] PID_CNT   = MAP_DA_AUTO[`MAPDA_DAA_PID_lb+4:`MAPDA_DAA_PID_lb] // 计算得出
  )
  (
  // APB总线接口
  input               PRESETn,          // 系统复位
  input               PCLK,             // 系统时钟（用于寄存器和同步）
  input               PSEL,             // 选择信号（与PENA配合）
  input               PENA,             // 读写周期使能
  input        [11:2] PADDR,            // 地址（字对齐）
  input               PWRITE,           // 写操作标识
  output       [31:0] PRDATA,           // 读数据
  input        [31:0] PWDATA,           // 写数据
  output              PREADY,           // 就绪信号（用于等待）
  output              wr_err,           // 写只读寄存器错误
  input         [1:0] ign_mwrite,       // 特殊信号（仅主机）：抑制寄存器写入

  // 输出到SCL时钟域的配置寄存器
  output              cf_SlvEna,        // 从机使能（为1时处理总线）
  output              cf_SlvNack,       // 不响应私有消息
  output        [7:0] cf_SlvSA,         // 可选的从机I2C地址
  output        [3:0] cf_IdInst,        // ID实例号（若使用）
  output              cf_IdRand,        // ID随机部分号（若使用）
  output              cf_Offline,       // 为1时，在SlvEna置1时检查总线状态
  output       [31:0] cf_Partno,        // ID部件号（若使用）
  output        [7:0] cf_IdBcr,         // DAA的BCR（若非常量）
  output        [7:0] cf_IdDcr,         // DAA的DCR（若非常量）
  output       [14:0] cf_IdVid,         // MIPI厂商ID（若非常量）
  output              cf_DdrOK,         // 允许DDR消息
  output              cf_TspOK,         // 允许TSP消息
  output              cf_TslOK,         // 允许TSL消息
  output       [11:0] cf_MaxRd,         // 最大读取字节数
  output       [11:0] cf_MaxWr,         // 最大写入字节数
  output       [23:0] cf_RstActTim,     // 复位激活时间值（可选，替代参数）
  output        [7:0] cf_BAMatch,       // 总线空闲计数器匹配值
  output       [15:0] cf_TCclk,         // 时间控制时钟信息（若基于寄存器）
  output              cf_s0ignore,      // 抑制S0错误
  output              cf_matchss,       // 仅当匹配时为1时产生Start/Stop
  output              cf_IbiExtData,    // IBI扩展数据使能
  output        [3:0] cf_IbiMapIdx,     // IBI使用的映射动态地址索引
  output        [2:0] cf_i2c_dev_rev,   // I2C设备版本号（若使用）
  output        [1:0] cf_HdrCmd,        // HDR命令MMR配置（读、写）
  output        [8:0] cf_vgpio,         // VGPIO作为CCC的控制
  output        [6:0] cf_CccMask,       // 未处理CCC的掩码
  output  [MAP_CNT-1:0] map_daa_use,      // 哪些映射条目用于自动DAA
  output[(MAP_CNT*8)-1:0] map_daa_dcr,      // 映射自动DAA的DCR值
  output[(MAP_CNT*PID_CNT)-1:0] map_daa_pid, // 映射自动DAA的PID部分值

  // ID_ASREGS相关：动态地址覆盖配置
  output        [7:0] SetDA,            // 设置动态地址（若配置）
  output [(MAP_CNT*10)-1:0] SetMappedDASA, // 映射的动态/静态地址列表
  output        [2:0] SetSA10b,         // 10位静态地址（索引1，若使用）

  // 主从混合构建相关
  input               is_slave,         // 1表示从机，0表示主机
  input               master_comp,      // 主机完成信号（用于DMA）

  // 来自SCL时钟域的原始状态（在状态寄存器读操作中同步）
  input       [29:28] raw_ActState,     // 总线活动状态
  input       [27:24] raw_EvState,      // 事件状态（HJ、P2P、MR、IBI）
  input         [2:0] raw_TimeC,        // 使能的时间控制（独热编码）
  input         [6:0] raw_Request,      // 总线请求处理及详情
  input         [7:0] raw_DynAddr,      // 当前动态地址（稳定）
  input         [2:0] raw_DynChgCause,  // 动态地址变更原因
  input        [12:0] raw_match_idx,    // 最近3个匹配的映射地址索引

  // PCLK时钟域内输入/输出寄存器（已同步）
  output       [19:8] reg_IntEna,       // 中断使能位
  output        [5:0] reg_DmaCtrl,      // DMA控制位（若使用）
  input        [19:8] inp_IntStates,    // 中断状态
  output        [2:0] reg_EvPend,       // 事件请求，[2]为变更标志
  output        [7:0] reg_EvIbiByte,    // 可选字节（若BCR要求）
  input               inp_EvNoCancel,   // 阻止取消（当IBI正在进行时）
  input       [22:20] inp_EvDet,        // 事件待处理/完成详情
  input         [5:0] inp_GenErr,       // 引擎常规错误
  input        [11:8] inp_DataErr,      // 引擎数据错误
  output        [5:0] msk_GenErr,       // 状态位和IRQ的错误掩码
  output       [11:8] msk_DataErr,      // 状态位和IRQ的数据错误掩码
  input               inp_err_loc,      // 用于DMA停止的错误定位
  output       [19:8] reg_clrIntStates, // 清除中断状态（写1清除）
  output        [5:0] reg_clrGenErr,    // 清除常规错误（写1清除）
  output       [11:8] reg_clrDataErr,   // 清除数据错误（写1清除）
  output              reg_holdOErr,     // 保持中断错误（若溢出为1）
  input               inpflg_MaxRd,     // 主机已更改MaxRd标志
  input               inpflg_MaxWr,     // 主机已更改MaxWr标志
  input        [11:0] inp_MaxRW,        // 主机设置的读写最大值
  output              reg_TbEnd,        // 发送至总线数据结束
  output              reg_TbFlush,      // 脉冲：刷新内部发送缓冲区
  output              reg_FbFlush,      // 脉冲：刷新内部接收缓冲区
  input               inp_dma_last_tb,  // DMA的最后一个发送数据（PCLK域）

  // FIFO相关信号（若使用内部FIFO）
  output        [5:4] reg_TxTrig,       // 发送FIFO触发水位
  output        [7:6] reg_RxTrig,       // 接收FIFO触发水位
  input       [20:16] inp_TxCnt,        // 当前发送FIFO计数
  input       [28:24] inp_RxCnt,        // 当前接收FIFO计数
  input               inp_TxFull,       // 发送FIFO满
  input               inp_RxEmpty,      // 接收FIFO空
  output        [1:0] regflg_wr_cnt,    // 发送：0=无，1=写字节，[1]未使用
  output        [7:0] reg_wdata,        // 发送：字节数据
  output        [1:0] regflg_rd_cnt,    // 接收：0=无，1=读字节，[1]未使用
  input         [7:0] inp_fb_data,      // 接收：字节数据
  output        [8:7] reg_ActMode,      // 系统活动模式（若使用）
  output        [3:0] reg_PendInt,      // 待处理中断（若使用）
  output       [15:8] reg_StatusRes,    // 状态寄存器保留位（若使用）
  input               hdr_new_cmd,      // 新HDR命令脉冲（PCLK域同步）
  input         [7:0] raw_hdr_cmd,      // 原始HDR命令（由new_cmd同步）
  input               map_rstdaa,       // 脉冲：映射自动DAA且发生RSTDAA
  input               map_setaasa,      // 脉冲：映射自动DAA且发生SETAASA
  input               map_daa_ena,      // 脉冲：映射动态地址分配/DAA使能
  input         [3:0] map_sa_idx,       // 动态地址分配时的索引
  input         [7:1] map_daa_da,       // 动态地址分配时的新地址
  output       [10:0] ibi_wr_fifo,      // IBI写入FIFO的4个信号
  input               ibi_wr_ack,       // IBI写入应答信号
  // 仅主机模式下使用，否则忽略
  output              exp_owrite_err,
  output              exp_oread_err
  `ifdef USE_D_RESET
  ,input        d_reset_r               // 特殊D输入复位（用于特定项目）
  `endif
  );

  // 内部信号定义
  wire                is_read, is_pre_read, is_write;
  reg          [14:0] pre_reg;          // 用于亚稳态位的预读寄存器
  wire                ma_config, ma_stat, ma_ctrl, ma_intset, ma_intclr;
  wire                ma_intmasked, ma_errwarn, ma_dmactrl, ma_datactrl;
  wire                ma_wdatab, ma_wdatabe, ma_rdatab, ma_capable, ma_capable2;
  wire                ma_dynaddr, ma_maxlimits, ma_idpartno, ma_idext, ma_idvid;
  wire                ma_wdatah, ma_wdatahe, ma_rdatah; // 包含SDR/DDR情况
  wire                ma_wibidata, ma_wdatab1, ma_tcclk, ma_msgmap, ma_rsttim, ma_id;
  wire                ma_vgpio, ma_hdrcmd, ma_cccmask, ma_errwarnmask;
  wire                ma_mapctrl0, ma_mapctrln;
  wire                ma_merrwarn;      // 仅主机模式专用
  wire         [31:0] dmac, maxl, idpart, idext;
  wire         [14:0] idvid;
  // 可写的寄存器
  reg                 slv_ena;
  reg                 slv_nack;
  wire        [31:25] opt_slv_saddr;    // 从地址（若使能）
  reg                 s0ignore_r;
  reg                 matchss_r;
  wire                opt_idrand;
  reg                 slv_offline;
  wire                opt_ddrok, opt_tspok, opt_tslok;
  wire          [2:0] opt_ctrl_ev;
  wire         [15:8] opt_ctrl_ibidata;
  wire        [19:16] opt_pendint;
  wire        [21:20] opt_actmode;
  wire        [31:24] opt_statusres;
  wire          [7:0] opt_bamatch;
  wire         [15:0] opt_tcclk;
  wire          [5:4] opt_txtrig;
  wire          [7:6] opt_rxtrig;
  wire                opt_ibi_wr_empty;
  wire         [23:0] opt_rsttimes;
  wire          [1:0] opt_hdrcmd;
  wire          [1:0] hdr_cmd;
  wire         [15:8] opt_vgpio_match;
  wire                opt_vgpio_ccc;
  wire          [7:0] write_data;       // 写入的字节数据
  wire          [7:0] read_buff;        // 读取的8位数据
  wire          [7:0] read_buff2;
  wire         [19:8] int_ena;
  reg                 tb_end_r;         // 发送结束寄存器或数据高位
  reg                 tb_flush_r;
  reg                 fb_flush_r;
  wire         [31:0] capable;          // 根据参数构建的能力寄存器
  wire         [31:0] capable2;         // 第二能力寄存器
  reg                 oread_err;
  reg                 owrite_err;
  wire          [3:0] MapLastIdx;
  wire          [4:0] MapLastDet;       // Nack、SA10、SA信息
  wire          [7:0] MapLastDA;
  wire         [31:0] mapctrl_reg;      // 用于显示映射寄存器的值

  // 总线属性判断
  assign is_read     = PSEL & PENA  & ~PWRITE;
  assign is_pre_read = PSEL & ~PENA & ~PWRITE;
  assign is_write    = PSEL & PENA  & PWRITE;
  assign PREADY  = 1;                   // 未使用

  // 根据地址判断当前访问的寄存器
  generate                              // 部分寄存器仅在允许时存在
    assign ma_config     = PADDR == (12'h004>>2);
    assign ma_stat       = PADDR == (12'h008>>2);
    assign ma_ctrl       = PADDR == (12'h00C>>2);
    if (SEL_BUS_IF[`SBIF_IRQ_b]) begin : allow_irq
      assign ma_intset   = PADDR == (12'h010>>2);
      assign ma_intclr   = PADDR == (12'h014>>2);
      assign ma_intmasked= PADDR == (12'h018>>2);
    end else begin
      assign ma_intset   = 1'b0;
      assign ma_intclr   = 1'b0;
      assign ma_intmasked= 1'b0;
    end
    assign ma_errwarn    = PADDR == (12'h01C>>2);
    assign ma_merrwarn   = ENA_MASTER[0]&(PADDR == (12'h09C>>2));
    assign ma_dmactrl    = SEL_BUS_IF[`SBIF_DMA_b] &
                           ((PADDR == (12'h020>>2)) | (ENA_MASTER[0]&(PADDR == (12'hA0>>2))));
    assign ma_datactrl   = (PADDR == (12'h02C>>2)) | (ENA_MASTER[0]&(PADDR == (12'hAC>>2)));
    assign ma_wdatab     = (PADDR == (12'h030>>2)) | (ENA_MASTER[0]&(PADDR == (12'hB0>>2)));
    assign ma_wdatab1    = (PADDR == (12'h054>>2)) | (ENA_MASTER[0]&(PADDR == (12'hCC>>2)));
    assign ma_wdatabe    = (PADDR == (12'h034>>2)) | (ENA_MASTER[0]&(PADDR == (12'hB4>>2)));
    assign ma_rdatab     = (PADDR == (12'h040>>2)) | (ENA_MASTER[0]&(PADDR == (12'hC0>>2)));
    assign ma_wdatah     = (SEL_BUS_IF[`SBIF_HALF_b] & (PADDR == (12'h038>>2))) | 
                           (ENA_MASTER[0]&(PADDR == (12'hB8>>2))) |
                           (ENA_MASTER[0]&(PADDR == (12'hD0>>2))) |
                           (ENA_MASTER[0]&(PADDR == (12'hD8>>2)));
    assign ma_wdatahe    = (SEL_BUS_IF[`SBIF_HALF_b] & (PADDR == (12'h03C>>2)))|
                           (ENA_MASTER[0]&(PADDR == (12'hBC>>2)));
    assign ma_rdatah     = (SEL_BUS_IF[`SBIF_HALF_b] & (PADDR == (12'h048>>2))) | 
                           (ENA_MASTER[0]&(PADDR == (12'hC8>>2))) |
                           (ENA_MASTER[0]&(PADDR == (12'hD4>>2))) |
                           (ENA_MASTER[0]&(PADDR == (12'hDC>>2)));
    assign ma_wibidata   = (ENA_IBI_MR_HJ[`EV_EXTFIFO_b]&(PADDR == (12'h050>>2)));
    assign ma_capable    = PADDR == (12'h060>>2);
    assign ma_capable2   = PADDR == (12'h05C>>2);
    assign ma_dynaddr    = PADDR == (12'h064>>2);
    assign ma_maxlimits  = ENA_CCC_HANDLING[`ENCCC_MAXES_b] & (PADDR == (12'h068>>2));
    assign ma_idpartno   = ((ENA_ID48B == `ID48B_CONST_PARTNO) |
                            (ENA_ID48B == `ID48B_CONST_NONE)) &
                           (PADDR == (12'h06C>>2));
    assign ma_idext      = (ID_AS_REGS[`IDREGS_BCR_b] | ID_AS_REGS[`IDREGS_DCR_b] |
                            (ENA_ID48B == `ID48B_CONST_INST) | (|MAP_I2CID)) &
                           (PADDR == (12'h070>>2));
    assign ma_rsttim     = RSTACT_CONFIG[`RSTA_MMR_b] & (PADDR == (12'h100>>2));
    assign ma_idvid      = ID_AS_REGS[`IDREGS_VID_b] & (PDR == (12'h074>>2));
    assign ma_tcclk      = ENA_TIMEC[`TC_FREQ_REG] & (PADDR == (12'h078>>2));
    assign ma_msgmap     = ENA_MAPPED[`MAP_ENA_b] & (PADDR == (12'h07C>>2));
    assign ma_vgpio      = ID_AS_REGS[`IDREGS_VGPIO_b] & (PADDR == (12'h104>>2));
    assign ma_hdrcmd     = ID_AS_REGS[`IDREGS_HDRCMD_b] & (PADDR == (12'h108>>2));
    assign ma_cccmask    = ID_AS_REGS[`IDREGS_CCCMSK_b] & (PADDR == (12'h10C>>2));
    assign ma_errwarnmask= ID_AS_REGS[`IDREGS_MSK_EW_b] & (PADDR == (12'h110>>2));
    localparam MPMX = 12'h11C+(MAP_CNT*4);
    assign ma_mapctrln   = ENA_MAPPED[`MAP_ENA_b] & 
                             ((PADDR >= (12'h120>>2)) & (PADDR <= (MPMX>>2)));
    assign ma_mapctrl0   = ENA_MAPPED[`MAP_ENA_b] & (PADDR == (12'h11C>>2));
    assign ma_id         = PADDR == (12'hFFC>>2);
  endgenerate

  // 写错误检测：写只读寄存器
  assign wr_err = is_write & 
                  (ma_intmasked | ma_rdatab | ma_rdatah | ma_capable | ma_capable2 |
                   ma_msgmap | ma_id);

  // 读数据返回逻辑（注意：CDC以不同方式处理）
  assign PRDATA = {32{is_read}} & (
    ({32{ma_config}}   & {cf_SlvSA[7:1],1'b0,cf_BAMatch,4'd0,hdr_cmd,cf_Offline,
                          cf_IdRand,1'b0,cf_TslOK,cf_TspOK,cf_DdrOK,cf_s0ignore,
                          cf_matchss,cf_SlvNack,cf_SlvEna}) |
    ({32{ma_stat}}     & {pre_reg[14:7],2'd0,inp_EvDet[21:20],inp_IntStates[19:8],
                          1'b0,pre_reg[6:0]}) |
    ({32{ma_ctrl}}     & {opt_statusres,2'b00,opt_actmode, opt_pendint,
                          opt_ctrl_ibidata[15:8],cf_IbiMapIdx,cf_IbiExtData,1'b0,
                          opt_ctrl_ev[1:0]}) |
    ({32{ma_intset}}   & {12'd0,int_ena,8'd0}) |
    ({32{ma_intclr}}   & {12'd0,int_ena,8'd0}) |
    ({32{ma_intmasked}}& {12'd0,int_ena[19:8]&inp_IntStates[19:8],8'd0}) |
    ({32{ma_errwarn}}  & {14'd0,owrite_err,oread_err,4'd0,inp_DataErr,2'd0,inp_GenErr}) |
    ({32{ma_datactrl}} & {inp_RxEmpty,inp_TxFull,1'b0,inp_RxCnt,3'd0,inp_TxCnt, 
                          8'd0,opt_rxtrig,opt_txtrig, 2'b00,2'b00}) |
    ({32{ma_rdatab}}   & {24'd0,read_buff}) |
    ({32{ma_rdatah}}   & {16'd0,read_buff,read_buff2}) |
    ({32{ma_wibidata}} & {31'd0, opt_ibi_wr_empty}) |
    ({32{ma_capable}}  & {capable}) |
    ({32{ma_capable2}} & {capable2}) |
    ({32{ma_dynaddr}}  & (|MapLastIdx ?
                          {15'd0,MapLastDet,MapLastIdx,MapLastDA} :
                          {15'd0,SetDA[0],5'd0, 
                           SetDA[0]?{3'd0,SetDA[7:0]} :
                                    {pre_reg[10:8],{8{pre_reg[0]}} & pre_reg[7:0]}})) |
    // 以下为可选寄存器，未使用时返回0
    ({32{ma_dmactrl}}  & {dmac}) |
    ({32{ma_maxlimits}}& {maxl}) |
    ({32{ma_idpartno}} & {idpart}) |
    ({32{ma_idext}}    & {idext}) |
    ({32{ma_idvid}}    & {17'd0,idvid}) |
    ({32{ma_tcclk}}    & {16'd0,opt_tcclk}) |
    ({32{ma_msgmap}}   & {12'd0,pre_reg[11:8],4'd0,pre_reg[7:4],3'd0,pre_reg[12],pre_reg[3:0]}) |
    ({32{ma_rsttim}}   & {8'd0,opt_rsttimes[23:0]}) |
    ({32{ma_vgpio}}    & {16'd0,opt_vgpio_match[15:8],7'd0,opt_vgpio_ccc}) |
    ({32{ma_hdrcmd}}   & {opt_hdrcmd[1:0],22'd0,raw_hdr_cmd[7:0]}) | // raw由标志保护
    ({32{ma_cccmask}}  & {25'd0,cf_CccMask[6:0]}) |
    ({32{ma_errwarnmask}}&{14'd0,6'd0,msk_DataErr,2'd0,msk_GenErr}) |
    ({32{ma_mapctrl0}} & {15'd0,SetDA[0],5'd0, 
                           SetDA[0]?{3'd0,SetDA[7:0]} :
                                    {pre_reg[10:8],{8{pre_reg[0]}} & pre_reg[7:0]}}) |
    ({32{ma_mapctrln}} & mapctrl_reg) |
    ({32{ma_id}}       & {BLOCK_ID})    // 若提供，返回ID和版本号
    );

  // 预读寄存器：在第一个读周期锁存可能亚稳态的位，第二个周期安全返回
  wire [1:0] timec = raw_TimeC[0] ? (|raw_TimeC[2:1] ? 2'd3 : 2'd1) : 
                     |raw_TimeC[2:1] ? 2'd2 : 2'd0;
  always @ (posedge PCLK or negedge PRESETn)
    if (!PRESETn) 
      pre_reg   <= 15'd0;               // 仅初始化需要寄存的位
    else if (is_pre_read)
      if (ma_stat)                      // 锁存活数据
        pre_reg[14:0] <= {timec,raw_ActState[29:28],
                          ~raw_EvState[27:24] & 4'b1011, // 无P2P
                          raw_Request[6:0]};
      else if (ma_dynaddr | ma_mapctrl0)
        pre_reg[10:0]  <= {raw_DynChgCause,raw_DynAddr};
      else if (ma_msgmap)
        pre_reg[12:0] <= raw_match_idx;

  // 可选的读寄存器和寄存器字段
  assign cf_SlvEna    = slv_ena;
  assign cf_SlvNack   = slv_nack;
  assign cf_SlvSA[7:1]= opt_slv_saddr;  // 可选I2C从地址
  assign cf_SlvSA[0]  = |opt_slv_saddr; // 仅当非0时有效
  assign cf_IdInst    = idext[3:0];
  assign cf_i2c_dev_rev= idext[6:4];
  assign cf_IdRand    = opt_idrand;
  assign cf_Offline   = slv_offline;
  assign cf_Partno    = idpart;
  assign cf_IdBcr     = idext[23:16];
  assign cf_IdDcr     = idext[15:8];
  assign cf_IdVid     = idvid;
  assign cf_DdrOK     = opt_ddrok;
  assign cf_TspOK     = opt_tspok;
  assign cf_TslOK     = opt_tslok;
  assign cf_MaxRd     = maxl[11:0];
  assign cf_MaxWr     = maxl[27:16];
  assign cf_RstActTim = opt_rsttimes;
  assign cf_BAMatch   = opt_bamatch;
  assign cf_TCclk     = opt_tcclk;
  assign cf_s0ignore  = s0ignore_r;
  assign cf_matchss   = matchss_r;
  assign cf_vgpio     = {opt_vgpio_ccc,opt_vgpio_match[15:8]};
  // 以下为保持在当前时钟域的活动网络
  assign reg_IntEna   = int_ena;
  assign reg_DmaCtrl  = dmac[5:0];
  assign reg_TbEnd    = tb_end_r;       // 数据控制或写数据高位
  assign reg_TbFlush  = tb_flush_r;     // 单周期脉冲，刷新内部缓冲区
  assign reg_FbFlush  = fb_flush_r;     // 单周期脉冲，刷新内部缓冲区
  assign reg_TxTrig   = opt_txtrig;
  assign reg_RxTrig   = opt_rxtrig;
  assign reg_wdata    = write_data;     // 字节或字节结束
  assign reg_clrGenErr  = (is_write & ma_errwarn) ? PWDATA[5:0]  : 6'd0;
  assign reg_clrDataErr = (is_write & ma_errwarn) ? PWDATA[11:8] : 4'd0;
  assign reg_holdOErr = oread_err | owrite_err;
  assign reg_EvPend   = slv_ena ? opt_ctrl_ev : 3'd0;
  assign reg_EvIbiByte= opt_ctrl_ibidata;
  assign reg_PendInt  = opt_pendint;
  assign reg_ActMode  = opt_actmode;
  assign reg_StatusRes= opt_statusres;

  // 可选寄存器的写和读逻辑
  `ifdef USE_D_RESET
    `define D_RESET d_reset_r
  `else
    `define D_RESET 0
  `endif
  generate 
    // 可选静态从地址配置
    if (ENA_SADDR == `SADDR_CONFIG) begin : saddr_reg
      reg       [7:1] saddr_r;
      assign opt_slv_saddr = saddr_r;
      always @ (posedge PCLK or negedge PRESETn)
        if (!PRESETn)                  saddr_r <= 7'd0;
        else if (`D_RESET)             saddr_r <= 7'd0;
        else if (is_write & ma_config) saddr_r <= PWDATA[31:25];
    end else begin
      assign opt_slv_saddr = 7'd0;
    end

    // 总线空闲计数器匹配值
    if (ENA_IBI_MR_HJ[`EV_BAMATCH_b]) begin : bamatch_reg
      reg [CLK_SLOW_BITS-1:0] bamatch_r;
      assign opt_bamatch = bamatch_r; // 可能不超过8位
      always @ (posedge PCLK or negedge PRESETn)
        if (!PRESETn)                  bamatch_r <= CLK_SLOW_MATCH[CLK_SLOW_BITS-1:0]; 
        else if (`D_RESET)             bamatch_r <= CLK_SLOW_MATCH[CLK_SLOW_BITS-1:0]; 
        else if (is_write & ma_config) bamatch_r <= PWDATA[(16+CLK_SLOW_BITS-1):16];
    end else begin
      assign opt_bamatch = 0;
    end

    // 时间控制时钟信息（若基于寄存器）
    if (ENA_TIMEC[`TC_FREQ_REG]) begin : tcclk_reg
      reg [15:0] tcclk_r;
      assign opt_tcclk = tcclk_r;
      always @ (posedge PCLK or negedge PRESETn)
        if (!PRESETn)                 tcclk_r <= TIMEC_FREQ_ACC[15:0]; 
        else if (`D_RESET)            tcclk_r <= TIMEC_FREQ_ACC[15:0]; 
        else if (is_write & ma_tcclk) tcclk_r <= PWDATA[15:0];
    end else begin
      assign opt_tcclk = 16'd0;
    end

    // ID随机部分号作为配置字段
    if (ID_AS_REGS[`IDREGS_RAND_b]) begin : rand_reg
      reg             idrand_r;
      assign opt_idrand = idrand_r;
      always @ (posedge PCLK or negedge PRESETn)
        if (!PRESETn)                  idrand_r <= 1'b0; 
        else if (`D_RESET)             idrand_r <= 1'b0; 
        else if (is_write & ma_config) idrand_r <= PWDATA[8];
    end else begin
      assign opt_idrand = 1'b0;
    end

    // 三种HDR模式的使能寄存器
    if (ENA_HDR[`HDR_DDR_b]) begin : ddr_reg
      reg               ddr_ok_r;
      assign opt_ddrok = ddr_ok_r;
      always @ (posedge PCLK or negedge PRESETn)
        if (!PRESETn)                  ddr_ok_r <= 1'b0; 
        else if (`D_RESET)             ddr_ok_r <= 1'b0; 
        else if (is_write & ma_config) ddr_ok_r <= PWDATA[4];
    end else begin
      assign opt_ddrok = 1'b0;
    end
    if (0 /*ENA_HDR[`HDR_TSP_b]*/) begin : tsp_reg
      reg               tsp_ok_r;
      assign opt_tspok = tsp_ok_r;
      always @ (posedge PCLK or negedge PRESETn)
        if (!PRESETn)                  tsp_ok_r <= 1'b0; 
        else if (`D_RESET)             tsp_ok_r <= 1'b0; 
        else if (is_write & ma_config) tsp_ok_r <= PWDATA[5];
    end else begin
      assign opt_tspok = 1'b0;
    end
    if (0 /*ENA_HDR[`HDR_TSL_b]*/) begin : tsl_reg
      reg               tsl_ok_r;
      assign opt_tslok = tsl_ok_r;
      always @ (posedge PCLK or negedge PRESETn)
        if (!PRESETn)                  tsl_ok_r <= 1'b0; 
        else if (`D_RESET)             tsl_ok_r <= 1'b0; 
        else if (is_write & ma_config) tsl_ok_r <= PWDATA[6];
    end else begin
      assign opt_tslok = 1'b0;
    end

    // 事件待处理及可选的IBI字节
    if (ENA_IBI_MR_HJ != 0) begin : ctrl_reg
      reg         [3:0] ctrl_ev_r; // [3:2]允许延迟标志
      reg               is_da;

      // 仅当允许且正确变更时允许事件（0->事件或事件->0）
      wire ev_chg_ok = ~|PWDATA[1:0] |
                       (~|ctrl_ev_r[1:0] &
                          (((PWDATA[1:0]==2'd1)&ENA_IBI_MR_HJ[`EV_IBI_b]) |
                           ((PWDATA[1:0]==2'd2)&ENA_IBI_MR_HJ[`EV_MR_b])  |
                           ((PWDATA[1:0]==2'd3)&ENA_IBI_MR_HJ[`EV_HJ_b])));
      // HJ仅当无动态地址时，IBI和MR仅当有动态地址时
      wire ev_allowed = ~|PWDATA[1:0] | (&PWDATA[1:0] & ~is_da) | (^PWDATA[1:0] & is_da);

      always @ (posedge PCLK or negedge PRESETn)
        if (!PRESETn)
          ctrl_ev_r        <= 4'd0;  
        else if (`D_RESET)
          ctrl_ev_r        <= 4'd0;  
        else if (inp_EvDet[22:20] == 3'd3) // 一个时钟为3，然后为7
          ctrl_ev_r        <= 4'd0;        // EvDet也会清除
        else if (is_write & ma_ctrl) begin // 优先级低于上述测试
          if (ev_chg_ok & ~inp_EvNoCancel & ev_allowed)
            ctrl_ev_r[3:0] <= {2'b10,PWDATA[1:0]}; // 0取消活动事件
        end else if (ctrl_ev_r[3])
          ctrl_ev_r[3:2]   <= 2'b01;    // 现在通知SCL域
      assign opt_ctrl_ev      = ctrl_ev_r[2:0];

      // 记录动态地址状态，以防变更
      always @ (posedge PCLK or negedge PRESETn)
        if (!PRESETn)
          is_da <= 1'b0;  
        else if (`D_RESET)
          is_da <= 1'b0;  
        else if (PSEL & ~PENA & PWRITE)
          is_da <= raw_DynAddr[0];

      if (ENA_MAPPED[`MAP_ENA_b]) begin : ibi_map_idx
        reg       [7:0] midx_r; // 映射设备索引（若使能）
        wire [((MAP_CNT+1)*8)-1:0] addrs = {SetMappedDASA[(MAP_CNT*10)-1:(MAP_CNT*2)],raw_DynAddr};
          // 确保索引有效。若动态地址未使能，则使用基础地址
        wire      [3:0] idx = (PWDATA[7:4]>MAP_CNT) ? 4'd0 : PWDATA[7:4];
        wire      [3:0] vidx = addrs[idx<<3] ? idx : 4'd0;
        always @ (posedge PCLK or negedge PRESETn)
          if (!PRESETn)
            midx_r <= 4'd0;  
          else if (`D_RESET)
            midx_r <= 4'd0;  
          else if (is_write & ma_ctrl) 
            midx_r <= vidx;
        assign cf_IbiMapIdx = midx_r;
      end else begin
        assign cf_IbiMapIdx = 4'd0;
      end
    end else begin
      assign opt_ctrl_ev   = 3'd0;
      assign cf_IbiMapIdx  = 4'd0;
    end
    // IBI数据仅当使能时
    if (ENA_IBI_MR_HJ[`EV_IBI_DAT_b]) begin : ibidata_reg
      reg        [15:8] ctrl_ibidata_r;
      reg               ctrl_extd_r;
      assign opt_ctrl_ibidata = ctrl_ibidata_r;
      assign cf_IbiExtData    = ctrl_extd_r;

      always @ (posedge PCLK or negedge PRESETn)
        if (!PRESETn) begin
          ctrl_ibidata_r   <= 8'd0;
          ctrl_extd_r      <= 1'b0;
        end else if (`D_RESET) begin
          ctrl_ibidata_r   <= 8'd0;
          ctrl_extd_r      <= 1'b0;
        end else if (is_write & ma_ctrl) begin
          if (~inp_EvNoCancel)
            ctrl_ibidata_r <= PWDATA[15:8];
          ctrl_extd_r      <= PWDATA[3];
        end
    end else begin
      assign opt_ctrl_ibidata = 8'd0;
      assign cf_IbiExtData    = 1'b0;
    end

    // 控制字段：待处理中断和活动状态
    if (ENA_CCC_HANDLING[`ENCCC_STATINF_b]) begin : status_fields
      reg       [19:16] ctrl_pendint_r;
      reg       [21:20] ctrl_actmode_r;
      assign opt_pendint = ctrl_pendint_r;
      assign opt_actmode = ctrl_actmode_r;

      // GETSTATUS字段
      always @ (posedge PCLK or negedge PRESETn)
        if (!PRESETn) begin
          ctrl_pendint_r <= 4'd0;
          ctrl_actmode_r <= 2'd0;
        end else if (`D_RESET) begin
          ctrl_pendint_r <= 4'd0;
          ctrl_actmode_r <= 2'd0;
        end else if (is_write & ma_ctrl) begin
          ctrl_pendint_r <= PWDATA[19:16];
          ctrl_actmode_r <= PWDATA[21:20];
        end
    end else begin
      assign opt_pendint = 4'd0;
      assign opt_actmode = 2'd0;
    end

    // 控制字段：厂商信息
    if (ENA_CCC_HANDLING[`ENCCC_STATVEND_b]) begin : status_vend
      reg       [31:24] ctrl_statusres_r;
      assign opt_statusres = ctrl_statusres_r;

      // GETSTATUS字段
      always @ (posedge PCLK or negedge PRESETn)
        if (!PRESETn) 
          ctrl_statusres_r <= 8'd0;
        else if (`D_RESET) 
          ctrl_statusres_r <= 8'd0;
        else if (is_write & ma_ctrl) 
          ctrl_statusres_r <= PWDATA[31:24];
    end else begin
      assign opt_statusres = 8'd0;
    end

    // 可选DMA控制（若支持）
    if (SEL_BUS_IF[`SBIF_DMA_b]) begin : dma_reg
      reg         [5:0] dma_r;
       // 注意：应设置MATCHSS以使用此功能
      wire              is_slv_done = is_slave & inp_IntStates[`IS_MATCHED] & 
                                      (inp_IntStates[`IS_STOP] | inp_IntStates[`IS_START]);
      wire              is_mst_done = ~is_slave & master_comp;
      assign dmac = {26'd0, dma_r};
      always @ (posedge PCLK or negedge PRESETn)
        if (!PRESETn)
          dma_r        <= 6'h10;
        else if (`D_RESET)
          dma_r        <= 6'h10;
        else if (is_write & ma_dmactrl) begin
          dma_r[3:0]   <= PWDATA[3:0];  // DMA设置
          if (SEL_BUS_IF[`SBIF_HALF_b])
            dma_r[5:4] <= &PWDATA[5:4] ? 2'd0 : PWDATA[5:4];
        end else if (is_slv_done | is_mst_done | inp_err_loc) begin
          // 从机：在START/STOP时自动清除（应仅与matchss配合使用）
          // 主机：在完成时自动清除
          dma_r[3:2] <= (dma_r[3:2]==2'd1) ? 2'd0 : dma_r[3:2];
          dma_r[1:0] <= (dma_r[1:0]==2'd1) ? 2'd0 : dma_r[1:0];
        end
    end else begin
      assign dmac = 32'd0;                // 不支持
    end

    // 最大读写限制
    // 注意：MXDS由常量处理（基于BCR[0]）
    if (ENA_CCC_HANDLING[`ENCCC_MAXES_b]) begin : ccc_max
      reg        [11:0] maxrd_r, maxwr_r;
      assign maxl = {4'd0,maxwr_r,4'd0,maxrd_r};
      always @ (posedge PCLK or negedge PRESETn)
        if (!PRESETn) begin
          maxrd_r <= MAX_RDLEN;
          maxwr_r <= MAX_WRLEN;
        end else if (`D_RESET) begin
          maxrd_r <= MAX_RDLEN;
          maxwr_r <= MAX_WRLEN;
        end else if (inpflg_MaxRd)      // 主机写入（已同步）
          maxrd_r <= inp_MaxRW;
        else if (inpflg_MaxWr)          // 主机写入（已同步）
          maxwr_r <= inp_MaxRW;
        else if (is_write & ma_maxlimits) begin
          // 注意：若应用在主机写入时同时写入，应用将丢失
          // 因此有风险时应读取回
          maxrd_r <= PWDATA[11:0];
          maxwr_r <= PWDATA[27:16];
        end
    end else begin
      assign maxl = 32'd0;
    end

    wire [7:0] custim = RSTACT_CONFIG[`RSTA_CUS_b] ? 
                        RSTACT_CONFIG[`RSTA_TIM_PER_b] : // 使用外设定时
                        8'd0;
    if (RSTACT_CONFIG[`RSTA_MMR_b]) begin : rstact_tim
      reg   [23:0] rsttimes_r;
      assign opt_rsttimes = rsttimes_r;
      always @ (posedge PCLK or negedge PRESETn)
        if (!PRESETn)                   // 从参数初始化
          rsttimes_r <= {custim, 
                         RSTACT_CONFIG[`RSTA_TIM_SYS_b], 
                         RSTACT_CONFIG[`RSTA_TIM_PER_b]};
        else if (`D_RESET)             
          rsttimes_r <= {custim, 
                         RSTACT_CONFIG[`RSTA_TIM_SYS_b], 
                         RSTACT_CONFIG[`RSTA_TIM_PER_b]};
        else if (is_write & ma_rsttim) 
          rsttimes_r <= PWDATA[23:0];
    end else begin
      assign opt_rsttimes = {custim, 
                             RSTACT_CONFIG[`RSTA_TIM_SYS_b], 
                             RSTACT_CONFIG[`RSTA_TIM_PER_b]};
    end

    // VGPIO可作为CCC或映射使用
    if (ID_AS_REGS[`IDREGS_VGPIO_b] & ENA_MAPPED[`MAP_VGPIO_b]) begin : vpgio_reg
      reg [7:0] vgpio_match_r;          // 匹配的CCC
      reg       vgpio_ccc_r;            // 1表示使用CCC，否则使用映射
      assign opt_vgpio_match = vgpio_match_r;
      assign opt_vgpio_ccc   = vgpio_ccc_r;
      always @ (posedge PCLK or negedge PRESETn)
        if (!PRESETn) begin
          vgpio_match_r <= 8'd0;
          vgpio_ccc_r   <= 1'b0;
        end else if (`D_RESET) begin
          vgpio_match_r <= 8'd0;
          vgpio_ccc_r   <= 1'b0;
        end else if (is_write & ma_vgpio) begin
          vgpio_match_r <= PWDATA[15:8];
          vgpio_ccc_r   <= PWDATA[0];
        end
    end else begin
      assign opt_vgpio_match[15:8] = 8'd0;
      assign opt_vgpio_ccc         = 1'b0; // 仅通过映射
    end

    // HDR命令MMR（若使能参数和配置位）
    if (ID_AS_REGS[`IDREGS_HDRCMD_b]) begin : hdrcmd_reg
      reg  [1:0] hdr_cmd_r;
      reg   hdr_new_r, hdr_oflow_r;

      assign hdr_cmd   = hdr_cmd_r;
      assign cf_HdrCmd = ~^hdr_cmd_r  ? 2'b00 : // 3不允许
                         hdr_cmd_r[0] ? 2'b11 : // 读写均到MMR
                                        2'b10;  // 仅读到MMR
      assign opt_hdrcmd = {hdr_new_r, hdr_oflow_r};

      // 新命令脉冲（已同步）
      always @ (posedge PCLK or negedge PRESETn)
        if (!PRESETn) begin
          hdr_new_r     <= 1'b0;
          hdr_oflow_r   <= 1'b0;
        end else if (`D_RESET) begin
          hdr_new_r     <= 1'b0;
          hdr_oflow_r   <= 1'b0;
        end else if (hdr_new_cmd) begin
          hdr_new_r     <= 1'b1;
          if (hdr_new_r)
            hdr_oflow_r <= 1'b1;
        end else if (is_read & ma_hdrcmd) begin
          // 读时清除
          hdr_new_r     <= 1'b0;
          hdr_oflow_r   <= 1'b0;
        end

      // 配置寄存器中的HDR命令使能
      always @ (posedge PCLK or negedge PRESETn)
        if (!PRESETn)
          hdr_cmd_r <= 2'b00;
        else if (`D_RESET)
          hdr_cmd_r <= 2'b00;
        else if (is_write & ma_config) 
          hdr_cmd_r <= ~&PWDATA[11:10] ? PWDATA[11:10] : 2'b00;

    end else begin
      assign cf_HdrCmd  = 1'b0;         // 永不向上传递
      assign opt_hdrcmd = 2'd0;
      assign hdr_cmd    = 2'd0;
    end

    // 未处理CCC的掩码（若使用）
    if (ID_AS_REGS[`IDREGS_CCCMSK_b]) begin : cccmask_reg
      reg [6:0] ccc_mask_r;
      assign cf_CccMask = ccc_mask_r;
      always @ (posedge PCLK or negedge PRESETn)
        if (!PRESETn) 
          ccc_mask_r <= 7'h7F;          // 默认全部使能
        else if (`D_RESET) 
          ccc_mask_r <= 7'h7F;
        else if (is_write & ma_cccmask) 
          ccc_mask_r <= PWDATA[6:0];
    end else begin
      assign cf_CccMask = 7'h7F;        // 默认全部使能
    end

    if (ID_AS_REGS[`IDREGS_MSK_EW_b]) begin : ewmask_reg
      reg [5:0]  msk_gen_r;
      reg [11:8] msk_data_r;
      assign msk_GenErr  = msk_gen_r;
      assign msk_DataErr = msk_data_r;
      always @ (posedge PCLK or negedge PRESETn)
        if (!PRESETn) begin
          msk_gen_r  <= 6'h3F;          // 默认全部使能
          msk_data_r <= 4'hF;           // 默认全部使能
        end else if (`D_RESET) begin
          msk_gen_r  <= 6'h3F;
          msk_data_r <= 4'hF;
        end else if (is_write & ma_errwarnmask) begin
          msk_gen_r  <= PWDATA[5:0];
          msk_data_r <= PWDATA[11:8];
        end
    end else begin
      assign msk_GenErr  = 6'h3F;
      assign msk_DataErr = 4'hF;
    end

    // 部件号，无论真实或随机
    if ((ENA_ID48B == `ID48B_CONST_PARTNO) |
        (ENA_ID48B == `ID48B_CONST_NONE)) begin : partno_reg
      reg        [31:0] partno_r;
      assign idpart = partno_r;
      always @ (posedge PCLK or negedge PRESETn)
        if (!PRESETn)                    partno_r   <= 32'd0;
        else if (`D_RESET)               partno_r   <= 32'd0;
        else if (is_write & ma_idpartno) partno_r <= PWDATA;
    end else begin
      assign idpart = 32'd0;
    end

    // 扩展ID，包含BCR、DCR和实例号的组合
    assign idext[31:24] = 8'd0;
    if (ID_AS_REGS[`IDREGS_BCR_b]) begin : bcr_reg
      reg         [7:0] bcr_r;
      assign idext[23:16] = bcr_r;
      always @ (posedge PCLK or negedge PRESETn)
        if (!PRESETn)                  bcr_r <= 8'd0;
        else if (`D_RESET)             bcr_r <= 8'd0;
        else if (is_write & ma_idext)  bcr_r <= PWDATA[23:16];
    end else begin
      assign idext[23:16] = 8'h0;
    end
    if (ID_AS_REGS[`IDREGS_DCR_b]) begin : dcr_reg
      reg         [7:0] dcr_r;
      assign idext[15:8] = dcr_r;
      always @ (posedge PCLK or negedge PRESETn)
        if (!PRESETn)                  dcr_r <= 8'd0;
        else if (`D_RESET)             dcr_r <= 8'd0;
        else if (is_write & ma_idext)  dcr_r <= PWDATA[15:8];
    end else begin
      assign idext[15:8] = 8'h0;
    end
    if (ENA_ID48B == `ID48B_CONST_INST) begin : inst_reg
      reg         [3:0] inst_r;
      assign idext[3:0] = inst_r;
      always @ (posedge PCLK or negedge PRESETn)
        if (!PRESETn)                  inst_r <= 8'd0;
        else if (`D_RESET)             inst_r <= 8'd0;
        else if (is_write & ma_idext)  inst_r <= PWDATA[3:0];
    end else begin
      assign idext[3:0] = 4'd0;
    end
    if (|MAP_I2CID) begin : i2c_revision
      reg [2:0] i2c_rev_r;
      assign idext[7:4] = {1'b0,i2c_rev_r};
      always @ (posedge PCLK or negedge PRESETn)
        if (!PRESETn)                  i2c_rev_r <= 3'd0;
        else if (`D_RESET)             i2c_rev_r <= 3'd0;
        else if (is_write & ma_idext)  i2c_rev_r <= PWDATA[6:4];
    end else begin
      assign idext[7:4] = 4'd0;
    end

    if (ID_AS_REGS[`IDREGS_VID_b]) begin : vid_reg
      reg          [14:0] vid_r;
      assign idvid [14:0] = vid_r;
      always @ (posedge PCLK or negedge PRESETn)
        if (!PRESETn)                  vid_r <= ID_48B[47:33]; // 复位为默认值
        else if (`D_RESET)             vid_r <= ID_48B[47:33];
        else if (is_write & ma_idvid)  vid_r <= PWDATA[14:0];
    end else begin
      assign idvid = 15'd0;
    end

    // 动态地址覆盖，用于掉电时保持
    // 可在从机禁用时写入。但若主机更改动态地址，则清除。
    // 也允许映射动态地址和静态地址。两种机制独立，可单独或同时使用。
    if (ID_AS_REGS[`IDREGS_DAWR_b]) begin : wrda_reg
      reg [7:1] wr_da;
      reg       hold_wr_da;             // 动态地址变更时清除
      reg       force_rst;
      assign SetDA = force_rst ? 8'hFF : {wr_da, hold_wr_da};
      always @ (posedge PCLK or negedge PRESETn)
        if (!PRESETn) begin
          wr_da      <= 7'd0;
          hold_wr_da <= 1'b0;
          force_rst  <= 1'b0;
        end else if (`D_RESET) begin
          wr_da      <= 7'd0;
          hold_wr_da <= 1'b0;
          force_rst  <= 1'b0;
        end else if (is_write & ma_dynaddr & ~|PWDATA[11:8] & 
                     (~slv_ena | ENA_MAPPED[`MAP_ENA_b]) & 
                     PWDATA[0] & (PWDATA[31:16]==16'hA4D9)) begin
          // 仅当从机未使能且使用密钥时才能写入
          wr_da      <= PWDATA[7:1];
          hold_wr_da <= 1'b1;
        end else if (is_write & ma_dynaddr & ENA_MAPPED[`MAP_ENA_b] & 
                     ~|PWDATA[15:0] & (PWDATA[31:16]==16'hCB19))
          force_rst  <= 1'b1;
        else if (inp_IntStates[`IS_DACHG])
          hold_wr_da <= 1'b0;
        else if (force_rst)
          force_rst  <= 1'b0;
    end else begin
      assign SetDA = 8'd0;
    end
    if (ENA_MAPPED[`MAP_ENA_b]) begin : wrda_map
      // 映射寄存器（位于PCLK域）
      reg [(MAP_CNT*8)-1:0] map_regs_ad;
      reg [(MAP_CNT*2)-1:0] map_regs_det;
      reg             [3:0] map_last_idx; // 用于回读
      assign SetMappedDASA = {map_regs_ad,map_regs_det};
      genvar i;
      for (i = 1; i <= MAP_CNT; i = i + 1) begin : map_writes
        always @ (posedge PCLK or negedge PRESETn)
          if (!PRESETn) begin
            map_regs_ad[((i*8)-1) -: 8]  <= 0;
            map_regs_det[((i*2)-1) -: 2] <= 0;
          end else if (`D_RESET) begin
            map_regs_ad[((i*8)-1) -: 8]  <= 0;
            map_regs_det[((i*2)-1) -: 2] <= 0;
          end else if (map_rstdaa) begin // 若未使能，始终为0
            // 注意：若重置且为静态地址，则无法正确切换回静态；是否应保留额外位？
            if (~map_regs_det[((i*2)-2)])
              map_regs_ad[((i*8)-8)]<= 1'b0; // 重置为关闭
          end else if (map_setaasa) begin // 若未使能，始终为0
            map_regs_det[((i*2)-2)] <= 1'b0; // 若为静态，现变为动态
          end else if (map_daa_ena & (map_sa_idx==i)) begin // 若未使能，始终为0
            map_regs_det[((i*2)-2)] <= 1'b0; // 动态地址 vs 静态地址
            map_regs_ad[((i*8)-1) -: 8]  <= {map_daa_da,1'b1};
          end else if (is_write & ma_dynaddr & (PWDATA[11:8]==i) &
                     (PWDATA[31:16]==16'hA4D9)) begin
            // 警告：若上述DASA或AASA在同一周期激活，则跳过下两条
            // 仅当映射索引非0且使用密钥时才能写入
            map_regs_ad[((i*8)-1) -: 8]  <= PWDATA[7:0];
            map_regs_det[((i*2)-1) -: 2] <= {1'b0,PWDATA[12]};
          end else if (is_write & ma_dynaddr & (PWDATA[11:8]==i) &
                      (PWDATA[31:16]==16'hA731)) begin
            map_regs_det[((i*2)-1) -: 1] <= ~PWDATA[0]; // [0]=1表示ACK，0表示NACK
          end else if (is_write & ma_mapctrln & (PADDR[4:2]==(i-1))) begin
            // 警告：若上述DASA或AASA在同一周期激活，则跳过下两条
            map_regs_ad[((i*8)-1) -: 8]  <= PWDATA[7:0];
            map_regs_det[((i*2)-1) -: 2] <= {PWDATA[12],PWDATA[8]};
          end
      end
      // I2C 10位静态地址
      if (ENA_MAPPED[`MAP_I2C_SA10_b]) begin : static_10b
        // 免费版本已移除扩展映射功能
      end else begin
        assign SetSA10b = 3'd0;
      end

      always @ (posedge PCLK or negedge PRESETn)
        if (!PRESETn) 
          map_last_idx <= 4'd0;
        else if (`D_RESET) 
          map_last_idx <= 4'd0;
        else if (is_write & ma_dynaddr) // 选择索引用于回读
          map_last_idx <= {4{|PWDATA[11:8] & ~|PWDATA[31:16]}} & PWDATA[11:8];
      wire [1:0] last_ns= map_regs_det[(map_last_idx<<1)-1 -: 2];
      assign MapLastIdx = map_last_idx;
      assign MapLastDet = {last_ns[1],{3{map_last_idx==4'd1}}&SetSA10b,last_ns[0]};
      assign MapLastDA  = map_regs_ad[(map_last_idx<<3)-1 -: 8];
      wire [5:0] mapr   = PADDR[4:2]; // 通过地址选择索引
      wire [7:0] mapp   = (mapr * PID_CNT);
      wire [1:0] mapns  = map_regs_det[(mapr<<1) +: 2];
      wire [7:0] maddr  =  map_regs_ad[(mapr<<3) +: 8];
      wire [31:14] mpid;
      wire [31:24] mdcr;
      wire       mdause;
      if (MAP_DA_AUTO[`MAPDA_DAA_MMR_b]) begin : daa_mrr_rd
        // 免费版本已移除扩展映射功能
      end else begin
        assign mpid     = 0; 
        assign mdcr     = 0;
        assign mdause   = 1'b0;
      end
      // 构建多个MAPCTRL寄存器的返回字符串
      assign mapctrl_reg= {mdcr,mpid[23:14],mdause,mapns[1],
                           {3{mapr==4'd0}}&SetSA10b,mapns[0],maddr};
      if (MAP_DA_AUTO[`MAPDA_DAA_MMR_b]) begin : map_daa_mmr
        // 免费版本已移除扩展映射功能
      end else begin
        assign map_daa_use = 0;
        assign map_daa_dcr = 0;
        assign map_daa_pid = 0;
      end
    end else begin
      assign SetMappedDASA = 0;
      assign SetSA10b      = 0;
      assign MapLastIdx    = 0;
      assign MapLastDet    = 0;
      assign MapLastDA     = 0;
      assign mapctrl_reg   = 0;
      assign map_daa_use   = 0;
      assign map_daa_dcr   = 0;
      assign map_daa_pid   = 0;
    end

    // 中断使能寄存器（通过intset、intclr配置）
    if (SEL_BUS_IF[`SBIF_IRQ_b]) begin : int_reg
      reg        [19:8] int_ena_r;
      assign int_ena          = int_ena_r;
      always @ (posedge PCLK or negedge PRESETn)
        if (!PRESETn) begin
          int_ena_r[18:8]   <= 11'd0;
          int_ena_r[19]     <= RSTACT_CONFIG[`RSTA_ENA_b]; // SLVRST默认使能
        end else if (`D_RESET) begin
          int_ena_r[18:8]   <= 11'd0;
          int_ena_r[19]     <= RSTACT_CONFIG[`RSTA_ENA_b];
        end else if (is_write)
          if (ma_intset) begin
            int_ena_r[18:8] <= int_ena_r[18:8] | PWDATA[18:8];
            int_ena_r[19]   <= int_ena_r[19] | (PWDATA[19]&RSTACT_CONFIG[`RSTA_ENA_b]);
          end else if (ma_intclr) 
            int_ena_r       <= int_ena_r & ~PWDATA[19:8];
    end else begin
      assign int_ena          = 12'd0;
    end
    // 状态位可通过两种方式清除
    assign reg_clrIntStates = (is_write & ma_stat) ? PWDATA[19:8] : 12'd0;

    // 发送FIFO或非FIFO控制
    // 始终使用，因为乒乓缓冲仍使用0与非0
    reg       [5:4] txtrig_r;
    assign opt_txtrig = txtrig_r;
    always @ (posedge PCLK or negedge PRESETn)
      if (!PRESETn)
        txtrig_r <= 2'd3;             // 默认值为3（任意）
      else if (`D_RESET)
        txtrig_r <= 2'd3;
      else if (is_write & ma_datactrl & PWDATA[3]) // 注意解锁位
        txtrig_r <= PWDATA[5:4];

    // 接收FIFO或非FIFO控制
    // 始终使用，因为乒乓缓冲仍使用0与非0
    reg       [7:6] rxtrig_r;
    assign opt_rxtrig = rxtrig_r;
    always @ (posedge PCLK or negedge PRESETn)
      if (!PRESETn) 
        rxtrig_r <= 2'd0;
      else if (`D_RESET) 
        rxtrig_r <= 2'd0;
      else if (is_write & ma_datactrl & PWDATA[3]) // 注意解锁位
        rxtrig_r <= PWDATA[7:6];

  endgenerate

  // 非可选寄存器写入逻辑
  // 配置寄存器 @0x004
  always @ (posedge PCLK or negedge PRESETn)
    if (!PRESETn) begin
      slv_ena    <= ID_AS_REGS[`IDREGS_SLVENA_b]; // 默认可能配置为使能
      slv_nack   <= 1'b0;
      s0ignore_r <= 1'b0;
      matchss_r  <= 1'b0;
      slv_offline<= 1'b0;
    end else if (`D_RESET) begin
      slv_ena    <= ID_AS_REGS[`IDREGS_SLVENA_b]; 
      slv_nack   <= 1'b0;
      s0ignore_r <= 1'b0;
      matchss_r  <= 1'b0;
      slv_offline<= 1'b0;
    end else if (is_write & ma_config) begin
      slv_ena    <= PWDATA[0];
      slv_nack   <= PWDATA[1];
      matchss_r  <= PWDATA[2];
      s0ignore_r <= PWDATA[3];
      slv_offline<= PWDATA[9];
    end 

  // 写数据字节和结束标志处理
  reg           [7:0] write_data_r;
  reg           [7:0] write_data2_r;
  reg           [1:0] rd_cnt;
  reg           [1:0] wr_cnt;
  reg           [1:0] wr_again;

  assign write_data = wr_again[0] ? write_data2_r : write_data_r;
  always @ (posedge PCLK or negedge PRESETn)
    if (!PRESETn) begin
      write_data_r   <= 0;
      write_data2_r  <= 0;
      wr_again       <= 2'b00;
      wr_cnt         <= 2'b00;
      owrite_err     <= 1'b0;
    end else if (`D_RESET) begin
      write_data_r   <= 0;
      write_data2_r  <= 0;
      wr_again       <= 2'b00;
      wr_cnt         <= 2'b00;
      owrite_err     <= 1'b0;
    end else if (is_write & (ma_wdatab | ma_wdatabe | ma_wdatab1 | &ign_mwrite)) begin
      // 注意：写入DDR消息时仅移动CMD（非半字）
      if (inp_TxFull)
        owrite_err   <= 1'b1;
      else begin 
        write_data_r <= PWDATA[7:0];
        wr_cnt       <= 2'b01;
      end
    end else if (is_write & ~ign_mwrite[0] & (ma_wdatah | ma_wdatahe)) begin
      // 若未使能，此逻辑将被优化
      if (inp_TxFull)
        owrite_err   <= 1'b1;
      else begin
        write_data2_r<= PWDATA[7:0];
        write_data_r <= PWDATA[15:8];
        wr_cnt       <= 2'b01;
        wr_again     <= {ma_wdatahe|PWDATA[16]|inp_dma_last_tb,1'b1};
      end
    end else if (is_write & (ma_errwarn|ma_merrwarn) & PWDATA[17])
      owrite_err     <= 1'b0;           // 写1清除
    else if (wr_again[0]) begin
      // 半字写入的第二个字节
      if (inp_TxFull)                   // 注意：第二个字节可能出错
        owrite_err   <= 1'b1;
      wr_again[0]    <= 1'b0;
    end else begin
      wr_cnt         <= 2'b00;          // 单周期脉冲
      if (is_pre_read & ma_rdatah & ~inp_RxEmpty)
        write_data2_r<= read_buff;      // 第一个半字（低字节）
    end

  // 写计数仅在寄存器后有效一个周期
  assign regflg_wr_cnt = wr_cnt;
  // 仅当非空时读取（本地处理错误）
  assign regflg_rd_cnt = (((is_read | is_pre_read) & ma_rdatah & ~inp_RxEmpty) |
                          (is_read & ma_rdatab & ~inp_RxEmpty)) ? 2'd1 : 2'd0;
  assign read_buff     = inp_RxEmpty ? 8'd0 : inp_fb_data[7:0];
  assign read_buff2    = write_data2_r;   // 保存低半字
  assign exp_owrite_err= owrite_err;
  assign exp_oread_err = oread_err;

  always @ (posedge PCLK or negedge PRESETn)
    if (!PRESETn) begin
      rd_cnt      <= 2'b00;
      oread_err   <= 1'b0;
    end else if (`D_RESET) begin
      rd_cnt      <= 2'b00;
      oread_err   <= 1'b0;
    end else if (is_read & ma_rdatab) begin
      if (inp_RxEmpty)
        oread_err <= 1'b1;
      else
        rd_cnt    <= 2'b01;
    end else if ((is_read | is_pre_read) & ma_rdatah) begin
      // 见write_data2_r获取额外字节
      if (inp_RxEmpty)
        oread_err <= 1'b1;
      else
        rd_cnt    <= 2'b01;
    end else if (is_write & (ma_errwarn|ma_merrwarn) & PWDATA[16])
      oread_err   <= 1'b0;              // 写1清除
    else
      rd_cnt      <= 2'b00;

  // 结束位单独处理
  // 发送结束可由专用位或WDATA设置
  always @ (posedge PCLK or negedge PRESETn)
    if (!PRESETn) 
      tb_end_r     <= 1'b0;
    else if (`D_RESET) 
      tb_end_r     <= 1'b0;
    else if (is_write) begin
      if (ma_wdatabe)                   // 结束数据寄存器
        tb_end_r   <= 1'b1;
      else if (ma_wdatab & (PWDATA[8] | PWDATA[16] | inp_dma_last_tb))
        tb_end_r   <= 1'b1;             // 结束标记位
      else if (ma_wdatab1 & inp_dma_last_tb)
        tb_end_r   <= 1'b1;             // DMA专用
      else
        tb_end_r   <= 1'b0;
    end else if (wr_again == 2'b11)     // wdatahe
      tb_end_r     <= 1'b1;             // 结束应用于第二个字节
    else
      tb_end_r     <= 1'b0; 
      
  // 发送刷新位：刷新内部发送状态
  always @ (posedge PCLK or negedge PRESETn)
    if (!PRESETn) 
      tb_flush_r <= 1'b0;
    else if (`D_RESET) 
      tb_flush_r <= 1'b0;
    else if (is_write & ma_datactrl & PWDATA[0])
      tb_flush_r <= 1'b1;
    else if (tb_flush_r)
      tb_flush_r <= 1'b0;

  // 接收刷新位：刷新内部接收状态
  always @ (posedge PCLK or negedge PRESETn)
    if (!PRESETn) 
      fb_flush_r <= 1'b0;
    else if (`D_RESET) 
      fb_flush_r <= 1'b0;
    else if (is_write & ma_datactrl & PWDATA[1])
      fb_flush_r <= 1'b1;
    else if (fb_flush_r)
      fb_flush_r <= 1'b0;

  generate if (ENA_IBI_MR_HJ[`EV_EXTFIFO_b]) begin : ibi_fifo_mux
    reg       rdy_r, valid_r, end_r, flush_r;
    reg [7:0] data_r;

    assign ibi_wr_fifo      = {flush_r, end_r, valid_r, data_r};
    assign opt_ibi_wr_empty = ~rdy_r;

    always @ (posedge PCLK or negedge PRESETn)
      if (!PRESETn)  begin
        rdy_r     <= 1'b0;              // 准备应用写入新数据
        end_r     <= 1'b0;
        flush_r   <= 1'b0;
        data_r    <= 8'd0;
      end else if (`D_RESET)  begin
        rdy_r     <= 1'b0;
        end_r     <= 1'b0;
        flush_r   <= 1'b0;
        data_r    <= 8'd0;
      end else if (is_write & ma_wibidata) begin
        if (PWDATA[31])
          flush_r <= 1'b1;
        else begin
          rdy_r   <= 1'b1;
          end_r   <= PWDATA[8] | PWDATA[16];
          data_r  <= PWDATA[7:0];
          flush_r <= 1'b0;
        end
      end else if (rdy_r & ibi_wr_ack)  // 数据已复制
        rdy_r     <= 1'b0;
      else if (flush_r)
        flush_r   <= 1'b0;

    always @ (posedge PCLK or negedge PRESETn)
      if (!PRESETn)
        valid_r   <= 1'b0;              // 有效，可推入FIFO
      else if (`D_RESET)
        valid_r   <= 1'b0;
      else if (is_write & ma_wibidata) 
        valid_r   <= 1'b1;
      else
        valid_r   <= 1'b0;
  end else begin
    assign ibi_wr_fifo      = 11'd0;
    assign opt_ibi_wr_empty = 1'b0;
  end endgenerate

  // 根据参数构建能力寄存器
  assign capable[1:0]   = ENA_ID48B[1:0]; // 0表示4（全部来自寄存器）
  assign capable[5:2]   = ID_AS_REGS[3:0];
  assign capable[8:6]   = ENA_HDR[2:0]; // DDR, BT, 保留
  assign capable[9]     = ENA_MASTER[0];
  assign capable[11:10] = ENA_SADDR[1:0];
  assign capable[15:12] = ENA_CCC_HANDLING[3:0];
  assign capable[20:16] = ENA_IBI_MR_HJ[4:0];
  assign capable[21]    = |ENA_TIMEC;
  assign capable[22]    = 1'b0;
  assign capable[25:23] = EXT_FIFO[2:0];
  wire [2:0] tmptb, tmpfb;
  assign tmptb          = ENA_TOBUS_FIFO[2:0]-2'd1;
  assign capable[27:26] = |ENA_TOBUS_FIFO[2:0] ? tmptb[1:0] : 2'd0;
  assign tmpfb          = ENA_FROMBUS_FIFO[2:0]-2'd1;
  assign capable[29:28] = |ENA_FROMBUS_FIFO[2:0] ? tmpfb[1:0] : 2'd0;
  assign capable[30]    = SEL_BUS_IF[`SBIF_IRQ_b];
  assign capable[31]    = SEL_BUS_IF[`SBIF_DMA_b];
  assign capable2[3:0]  = ENA_MAPPED ? MAP_CNT[3:0] : 4'd0;
  assign capable2[4]    = ENA_MAPPED[`MAP_I2C_SA10_b];
  assign capable2[5]    = ENA_MAPPED[`MAP_I2C_SLVRST_b];
  assign capable2[6]    = |MAP_I2CID; 
  assign capable2[7]    = 1'b0;
  assign capable2[8]    = 1'b1; // 是否应作为参数？
  assign capable2[9]    = ENA_IBI_MR_HJ[`EV_EXTFIFO_b];
  assign capable2[15:10]= 0;
  assign capable2[16]   = ENA_CCC_HANDLING[`ENCCC_V11MIN];
  assign capable2[17]   = RSTACT_CONFIG[`RSTA_ENA_b];
  assign capable2[19:18]= 2'd0; // 暂无组
  assign capable2[20]   = 1'b0; 
  assign capable2[21]   = ENA_CCC_HANDLING[`ENCCC_AASA];
  assign capable2[22]   = 1'b0;
  assign capable2[23]   = 1'b0;
  assign capable2[31:24]= 0;

endmodule