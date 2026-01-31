/*--------------------------------------------------------------------
此模块为 MIPI I3C 从设备包装器，纯粹在 SCL 时钟域中操作。
它实例化了从机引擎、DAA处理、CCC处理和退出模式检测等模块。
--------------------------------------------------------------------*/

`include "i3c_params.v"                 // 本地参数/常量

module i3c_slave_wrapper #(
    // 参数由上层驱动，以控制此块的构建和操作方式
    parameter ENA_ID48B       = `ID48B_CONST, // 常量vs寄存器
    parameter  ID_48B         = 48'h0,  // 必须从上面填入48位ID的部分或全部
    parameter ID_AS_REGS      = 12'd0,  // [0]=IDINST,[1]=ISRAND,[2]=IDDCR,[3]=IDBCR,[4]=VID,[5]=DAwr
    parameter  ID_BCR         = 8'd0,   // 从上面填入BCR（如果不是来自寄存器）
    parameter  ID_DCR         = 8'd0,   // 从上面填入DCR（如果不是来自寄存器）
    parameter ENA_SADDR       = `SADDR_NONE, // 无、常量、寄存器/网络
    parameter  SADDR_P        = 0,      // 7位，作为7:1
    parameter ENA_MAPPED      = 5'd0,   // 如果允许额外的DA/SA及相关
    parameter  MAP_CNT        = 4'd1,   // 允许的额外DA/SA数量
    parameter  MAP_I2CID      = 24'd0,// !=0 如果I2C扩展了DevID
    parameter  MAP_DA_AUTO    = {5'd0,1'b0,1'b0,3'd0,1'b0,1'b0,1'b0},
    parameter  MAP_DA_DAA     = 0,      // 如果不是MMR且PID/DCR !=0，则是位数组      
    parameter ENA_IBI_MR_HJ   = 0,      // 0表示无事件，否则事件作为掩码
    parameter ENA_CCC_HANDLING= 6'd0,   // 传递下来支持的内容
    parameter RSTACT_CONFIG   = 26'd0,  // 从机复位RSTACT CCC配置（参见RSTA_xxx_b字段）
    parameter MAX_RDLEN       = 0,      // 默认S->M长度最大值
    parameter MAX_WRLEN       = 0,      // 默认M->S长度最大值
    parameter MAX_DS_WR       = 0,      // M->S的数据速度限制
    parameter MAX_DS_RD       = 0,      // S->M的数据速度限制
    parameter MAX_DS_RDTURN   = 0,      // S->M读取请求的延迟需求
    parameter ENA_HDR         = 0,      // HDR模式支持
    parameter ENA_MASTER      = 0,      // 如果是主控 - 用于CCC支持
    parameter PIN_MODEL       = `PINM_COMBO,  // 组合引脚使用
    parameter ENA_TIMEC       = 6'b000010,    // 如果寄存器，res, res, 模式1, 模式0, 同步
    parameter TIMEC_FREQ_ACC  = {8'd24,8'd10},// 频率为12MHz（12.0=24）精度为1.0%
    parameter priv_sz         = PIN_MODEL[1] ? (ENA_HDR[0] ? 3 : 1) : 0,// 如果ext-pad+ddr则更宽
    parameter [7:0] PID_CNT   = MAP_DA_AUTO[`MAPDA_DAA_PID_lb+4:`MAPDA_DAA_PID_lb], // 计算得出
    parameter SLV_DBG_MX      = 0       // 调试观察器
  )
  (
  input               RSTn,             // 主复位
  // 需要系统慢时钟和可选的高速时钟
  // 如果是总线，则还需要总线接口
  // -- 否则输出SCL作为时钟

  // 定义引脚 - 从机将SCL视为仅输入，除了三态
  input               pin_SCL_in,       // SCL: 通常是时钟输入，如果是三态则为数据
  output              pin_SCL_out,      // SCL: 仅在三态时驱动
  output              pin_SCL_oena,     // SCL: 仅在三态时驱动
  input               pin_SDA_in,       // SDA: M->S
   // 下面3个在PIN_MODEL==`PINM_EXT_REG时使用特殊
  output              pin_SDA_out,      // SDA: S->M读取时
  output              pin_SDA_oena,     // SDA: S->M读取时
  output  [priv_sz:0] pin_SDA_oena_rise,// SCL上升沿时特殊如果EXT_REG，否则为0
    // 下一个是只有当SDA和SCL引脚有i2c 50ns尖峰
    // 滤波器并且可以通过网络打开/关闭时才使用
  output              i2c_spike_ok,     // 为允许i2c尖峰滤波器设为1
  output              i2c_hs_enabled,   // 如果启用，表示处于I2C HS模式
  // 主从机使能是来自寄存器的释放（如果有）
  input               slv_enable,       // 持续为1如果启用
  // 现在一些信号指示状态，以便上层或系统
  // 对它们采取行动或中断。顺序是int_start_seen，然后可能是一个匹配的
  // 地址（和fb_data_done），然后是数据和可能的state_in_CCC，最后是
  // in_STOP。请注意，上层中的DA更改中断
  output              int_start_seen,   // 看到START或重复START时的1周期脉冲
  output              int_start_err,    // SDA=1后STOP时保持 - 在START时清除
  output              int_da_matched,   // 匹配我们的动态地址时的1周期脉冲
  output              int_sa_matched,   // 匹配我们的静态地址时的1周期脉冲（如果有）
  output              int_7e_matched,   // 匹配i3c广播地址时的1周期脉冲
  output              int_ddr_matched,  // 匹配DDR命令到DA时的1周期脉冲
  output              int_in_STOP,      // STOP状态（直到START为止保持）
  output              int_event_sent,   // 事件（event_pending）发出时的1周期脉冲
  output              int_event_ack,    // int_event_sent时的1周期脉冲，如果是ACK vs NACK
  output              int_in_evproc,    // 1表示现在正在处理IBI
  output              int_ccc_handled,  // 如果CCC由我们处理则保持
  output              int_ccc,          // CCC未由我们处理时的1周期脉冲
  output              state_in_ccc,     // 持续为真 - 上下文
  output              state_in_daa,     // 持续为真 - 信息性
  output              state_in_hdr,     // 持续为真 - 信息性
  output        [6:0] state_req,        // 状态: [6]=HDR, [5]=ENTDAA, [4]=写, [3]=读, [2]=CCChandle, [1]=W/R, [0]=忙
  output              state_dir,        // 持有1如果头是RnW，否则为0
  output        [1:0] opt_state_AS,     // 已知的活动状态
  output        [3:0] opt_ev_mask,      // 已知的事件掩码
  output        [2:0] opt_TimeC,        // 启用的时间控制（如果有）- 单热
  output       [12:0] opt_timec_sync,   // 用于SYNC时间控制
  output        [3:0] opt_slvr_reset,   // 用于从机复位控制
  output       [12:0] opt_match_idx,    // 最后3个匹配的Map地址（如果使用）
  input               i_rst_slvr_reset, // 在SCL域中
  input               clr_slvr_cause,   // 清除原因 - 同步到SCL
      // 注意: state_bus_act是~int_in_STOP
  // 现在与寻址相关
  output        [7:0] dyn_addr,         // 我们从主设备获得的动态地址
  input         [7:0] opt_static_addr,  // 静态地址匹配在[7:1]中如果[0]=1（SADDR控制）
  output              dyn_addr_chg,     // 来自ENTDAA、SETDASA、RSTDAA、SETNEWDA的变化
  output        [2:0] dyn_chg_cause,    // 最后变化的原因
  // 现在与事件相关，如IBI、MR和HJ（如果使用） 
  input         [2:0] event_pending,    // !=0如果有事件（MR=2, IBI=1, HJ=3）待处理
  input               force_sda,        // 强制SDA只要SCL=1。这是看时间的
  input               ibi_has_byte,     // 1如果有附加到IBI的额外字节
  input         [7:0] opt_ibi_byte,     // 如果我们有一个字节发送
  input               ibi_timec,        // 1如果IBI的时间控制开启
  output        [1:0] ibi_timec_marks,  // SC1和SC2停止 - 脉冲
  output        [2:0] ibi_timec_sel,    // 多路选择器
  input               ibi_extdata,      // 如果在IBI字节后读取数据
  input         [3:0] ibi_mapidx,       // IBI的映射索引
  output              ibi_in_extdata,   // 表示正在读取IBI扩展数据（SCL域）
  input               timec_oflow,
  output              timec_oflow_clr,
  // 我们导出SCL和SCL_n向上以允许到总线和从总线
  output              oclk_SCL,         // SCL作为时钟
  output              oclk_SCL_n,       // SCL作为时钟
  // 现在通用缓冲区到总线。我们使用clk_SCL_n作为操作
  // 时钟进行确认。有效模型是01（下部），10（上部），或00（无）。
  // 确认将允许其前进（可以使用SCL读取）。
  input               tb_data_valid,    // b1如果满，b0如果空
  input         [7:0] tb_datab,         // 来自系统的输入字节如果valid==1
  input               tb_end,           // 没有更多数据
  output              tb_datab_ack,     // 在消耗时在1 SCL_n时钟上的确认
  output              tb_urun_nack,     // 如果由于没有数据而NACKed则脉冲1 SCL_n
  output              tb_urun,          // 下溢且未结束
  output              tb_term,          // 由主设备终止
  // 现在通用缓冲区从总线。此块上方的层使用clk_SCL作为操作时钟
  // 以指示要填充的缓冲区。使用模型是01（构建下部），10（构建上部），
  // 或0（仍然满，因此如果数据进入，将溢出）。
  // 此块在clk_SCL_n上脉冲完成。数据使用不应在完成之前改变。
  input               fb_data_use,      // b1如果OK，b0如果满
  output        [7:0] fb_datab,         // 输出字节到系统如果done=1且use=1
  output              fb_datab_done,    // 在写入时在clk_SCL_n上脉冲完成
  output              fb_datab_err,     // 在i3c奇偶校验错误时与done一起设置
  output              fb_orun,          // 如果溢出（满时数据进入）则在clk_SCL_n上脉冲
  output        [2:0] fb_ddr_errs,      // DDR帧、奇偶校验、CRC
  output              fb_s0s1_err,      // 持续为1如果在S0或S1错误保持中
  input               brk_s0s1_err,     // 在scl域中脉冲1以打破S0/S1错误保持
  input               rd_abort,         // 中止读取由于时钟停滞（>60us）
  output              fb_hdr_exit,      // 看到HDR退出模式（在SDA域中的脉冲！）
  // 下一个是仅导出位索引和ccc字节和willbe_ccc - 用于专用用途
  output        [2:0] obit_cnt,         // 只有在需要且了解含义时使用
  output        [7:0] ccc_byte,         // CCC字节，如果有特殊含义如果是DDR
  output              willbe_ccc,       // 下一周期的CCC（除了奇偶校验错误）
  // 现在来自寄存器或外部系统的网络，取决于ID_AS_REGS
  input         [3:0] cf_IdInst,        // 选择时的ID实例（如果不是partno使用）
  input               cf_IdRand,        // ID的随机partno（如果使用）
  input        [31:0] cf_Partno,        // ID的partno（如果使用）
  input         [7:0] cf_IdBcr,         // DAA的BCR（如果不是常量）
  input         [7:0] cf_IdDcr,         // DAA的DCR（如果不是常量）
  input        [14:0] cf_IdVid,         // 供应商ID（如果不是常量）
  // 更多这样的网络，其使用取决于参数（CCC、HDR等）
  input        [11:0] cf_MaxRd,         // 最大读取返回
  input        [11:0] cf_MaxWr,         // 主设备的最大写入
  input        [23:0] cf_RstActTim,     // 从机复位恢复时间
  input               cf_SlvNack,       // Nack消息（临时）
  input               cf_SdrOK,         // 允许I3C SDR，否则仅i2c
  input               cf_DdrOK,         // 允许DDR消息
  input               cf_TspOK,         // 允许TSP消息
  input               cf_TslOK,         // 允许TSL消息
  input        [15:0] cf_TCclk,         // 时间控制时钟信息
  input               cf_s0ignore,      // 忽略S0错误
  input         [7:0] cf_SetDA,         // 主设备设置自己的DA
  input [(MAP_CNT*10)-1:0] cf_SetMappedDASA, // 支持的映射SA/DA
  input         [2:0] cf_SetSA10b,      // 如果[1]是SA 10位
  input         [1:0] cf_HdrCmd,        // 作为MMR启用HDR Cmd
  input         [6:0] cf_CccMask,       // 未处理CCC的掩码启用
  input         [8:0] cf_vgpio,         // VGPIO控制
  input               cf_MasterAcc,     // 仅用于M+S，从机可以接受主设备权
  input   [MAP_CNT-1:0] map_daa_use,      // 哪个MAP是自动DAA
  input [(MAP_CNT*8)-1:0] map_daa_dcr,      // 如果MAP自动DAA的DCR
  input [(MAP_CNT*PID_CNT)-1:0] map_daa_pid, // 如果MAP自动DAA的PID部分
  output              opt_MasterAcc,    // 在SCL中脉冲1如果主设备被接受
  output              raw_vgpio_done,   // 如果VGPIO启用则完成脉冲
  output        [7:0] raw_vgpio_byte,   // 字节值如果"
  output              raw_hdr_newcmd,   // 在SCL域中脉冲如果有新CMD
  output        [7:0] raw_hdr_cmd,      // 新CMD的原始但新_cmd - 如果cf_HdrCmd使用
  output              raw_setaasa,      // 如果MAP自动
  output              raw_rstdaa,       // RSTDAA用于map使用（如果是自动DAA）
  output              raw_daa_ena,      // 如果map自动-DASA匹配或自动-DAA匹配则脉冲
  output        [3:0] map_sa_idx,       // 如果raw_daa_ena的索引
  output        [7:1] map_daa_da,       // 新DA
    // 如果使用i2c扩展的东西
  input         [2:0] i2c_dev_rev,      // 如果DeviceID使用则修订（或入const）
  output              i2c_sw_rst,       // SW复位
  // 下面这些是安全-原始的。这意味着要么在
  // 总线停止时更改，要么在GETSTATUS期间或同步时更改。
  input         [7:6] sraw_ActMode,     // 系统活动模式（如果不使用则为0）
  input         [3:0] sraw_PendInt,     // 待处理中断（如果不使用则为0） 
  input        [15:8] sraw_StatusRes,   // 状态的保留位（如果不使用则为0） 
  // 现在返回处理的CCC的状态
  output              opt_ChgMaxRd,
  output              opt_ChgMaxWr,
  output       [11:0] opt_MaxRdWr,      // 一个上述变化时的值
  // 调试观察器
  output[SLV_DBG_MX:0]slv_debug_observ, // 可选观察器
  // 现在扫描相关
  input               scan_single_clock,// 来自扫描的单一时钟域
  input               scan_clk,         // 如果在扫描中使用的时钟
  input               scan_no_rst,      // 防止分层复位
  input               scan_no_gates     // 防止时钟门控
  );

  //
  // 目录
  //
  // 1. 我们定义连接块的线和时钟生成的线
  // 2. 我们从SCL和SDA引脚生成时钟。
  // 3. 我们处理一些特殊网络如SDR_hold
  // 4. 我们实例化引擎、daa、ccc和退出模式检测块
  // 5. 我们可选地实例化HDR块
  //

  // 1. 我们定义连接块的线和时钟生成的线
  wire                HDR_exit;
  wire                in_HDR_mode;      // 看到CCC ENTHDRn
  wire                HDR_restart;
  wire                HDR_restart_ack;
  wire                SDR_hold;
  reg                 sdr_hold_r;
  wire                daa_inp_drv;
  wire                daa_inp_bit;
  wire                daa_done;
  wire                daa_acknack;
  wire          [7:0] state_in_CCC_v;
  wire                state_accslv;
  wire                state_req_rd;
  wire                state_req_wr;
  wire                state_daa;
  wire                state_is_read;
  wire          [1:0] next_ccc;
  wire          [1:0] next_hdr;         // [1]=ddr, [0]=hdr
  wire                in_ddr;
  wire                ccc_handled;
  wire                ccc_handling;
  wire                ccc_uh_mask;
  wire                ccc_get;
  wire                ccc_is_read;
  wire                ccc_real_START;
  wire          [7:0] done_buff;
  wire                done_rdy;
  wire                ccc_tb_now;
  wire                ccc_tb_data;
  wire                ccc_tb_continue;
  wire          [1:0] ccc_vgpio_done;
  wire          [7:0] ccc_vgpio;
  wire                is_9th;
  wire                daa_active;
  wire                daa_act_ok;
  wire         [63:0] daa_id;
  wire                set_da;
  wire          [7:1] new_da;
  wire          [7:1] map_da;
  wire                raw_dasa_ena;
  wire                raw_map_daa;
  wire          [3:0] map_dasa_idx;
  wire          [3:0] daa_map_idx;
  wire                opt_setaasa;
  wire                match_da, match_sa;
  wire          [6:0] ccc_counter;
  wire                stb_urun_nack;
  wire                stb_urun;
  wire                stb_term;
  wire                fb_s0s1;
  reg                 s0s1_err;
    // HDR-DDR内容如下。仅在启用DDR时使用
  wire          [1:0] ddr_fb_ctrl;
  wire          [1:0] ddr_fb;
  wire          [1:0] ddr_tb_ctrl;
  wire          [1:0] ddr_tb_bits; 
  wire          [1:0] ddr_tb_oe;
  wire          [7:0] byte_1st;
  wire          [7:0] byte_2nd;
  wire                dtb_urun_nack;
  wire                dtb_urun;
  wire                dtb_term;
  wire                ddr_read;

  wire                clk_SCL_n;
  wire                clk_SCL;
  wire                clk_SDA;
  wire                clk_SDA_n;

  //
  // 允许导出调试观察器数据
  // -- 默认存根分配0，但这允许添加网络
  // -- 注意ENG_DBG_MX由包含文件定义为
  //    eng_debug_observ
  //
  `ifdef ENA_DBG_OBSERVE
   `include "dbg_observ_slv.v"   // 集成商替换包含文件内容
  `else
   localparam ENG_DBG_MX = 0;
   wire [ENG_DBG_MX:0] eng_debug_observ;
   assign slv_debug_observ = 0;
  `endif
  //


  // 2. 我们从SCL和SDA引脚生成时钟。
  // 四个时钟域的时钟：来自引脚除了在扫描中
  // 以下是值得的时钟：无毛刺且名称用于时钟源
  // 注意SCL与SCL_n的关系是定义的，SCL与SDA的关系也是如此
  CLOCK_SOURCE_MUX source_scl  (.use_scan_clock(scan_single_clock), .scan_clock(scan_clk), 
                                .pin_clock(pin_SCL_in), .clock(clk_SCL));
  CLOCK_SOURCE_INV source_scl_n(.scan_no_inv(scan_single_clock), .good_clock(clk_SCL), 
                                .clock(clk_SCL_n));
  CLOCK_SOURCE_MUX source_sda  (.use_scan_clock(scan_single_clock), .scan_clock(scan_clk), 
                                .pin_clock(pin_SDA_in), .clock(clk_SDA));
  CLOCK_SOURCE_INV source_sda_n(.scan_no_inv(scan_single_clock), .good_clock(clk_SDA), 
                                .clock(clk_SDA_n));
    // 导出SCL时钟用于数据CDC
  assign oclk_SCL   = clk_SCL; 
  assign oclk_SCL_n = clk_SCL_n; 

  generate if (1 /*~|ENA_HDR[`HDR_TSL_b:`HDR_TSP_b]*/) begin : no_hdr_ternary
    assign pin_SCL_out     = 0;
    assign pin_SCL_oena    = 0;         // 我们从不驱动SCL除了三态
  end else begin
    // 未来：如果决定支持HDR三态，则添加逻辑
  end endgenerate

  // 3. 我们处理一些特殊网络如SDR_hold
  // SDR_hold用于在HDR中（等待退出模式，
  //   无论我们是否支持HDR）以及启用和
  //   S0/S1错误，等待HDR退出模式。
  //   注意slv_enable既重置注册的保持
  //   如果关闭，也防止SDR使用（如果关闭则任何使用）
  wire hold_RSTn = RSTn & ((~HDR_exit & slv_enable) | scan_no_rst);
  wire s0s1_RSTn = hold_RSTn & (~(brk_s0s1_err & s0s1_err) | scan_no_rst);
  always @ (posedge clk_SCL_n or negedge s0s1_RSTn)
    if (!s0s1_RSTn)
      sdr_hold_r  <= 1'b0;              // i2c/SDR是默认值
    else if (in_HDR_mode | next_hdr[0] | s0s1_err)
      sdr_hold_r  <= 1'b1;              // 等待退出模式（这会重置）
  assign SDR_hold = sdr_hold_r | &next_hdr[1:0] | // DDR是早期
                    ~slv_enable;        // 如果禁用则忽略

  // S0和S1错误锁定等待HDR退出。我们允许应用程序也中断
  always @ (posedge clk_SCL_n or negedge s0s1_RSTn)
    if (!s0s1_RSTn)
      s0s1_err <= 1'b0;
    else if (fb_s0s1)
      s0s1_err <= 1'b1;
  assign fb_s0s1_err = s0s1_err;

  // 4. 我们实例化引擎、daa、ccc和退出模式检测块
  //
  // 从机引擎是从机的工作马 - SDR协议
  // -- 也知道一些CCC和模式和状态，如需要
  // -- 由HDR模式抑制，使用退出模式检测器唤醒它
  // -- 有一些HDR DDR知识
  //
  i3c_sdr_slave_engine #(.ENA_SADDR(ENA_SADDR), .SADDR_P(SADDR_P),
                         .ENA_MAPPED(ENA_MAPPED),.MAP_CNT(MAP_CNT),
                         .MAP_DA_AUTO(MAP_DA_AUTO), .MAP_DA_DAA(MAP_DA_DAA),
                         .MAP_I2CID(MAP_I2CID),
                         .ENA_IBI_MR_HJ(ENA_IBI_MR_HJ), .ENA_HDR(ENA_HDR),
                         .PIN_MODEL(PIN_MODEL),.ENG_DBG_MX(ENG_DBG_MX))
    engine
    (
    .clk_SCL_n        (clk_SCL_n), 
    .clk_SCL          (clk_SCL), 
    .clk_SDA          (clk_SDA), 
    .RSTn             (RSTn), 
    .SDR_hold         (SDR_hold), 
    .cf_SdrOK         (cf_SdrOK),
    .pin_SCL_in       (pin_SCL_in), 
    .pin_SDA_in       (pin_SDA_in), 
    .pin_SDA_out      (pin_SDA_out), 
    .pin_SDA_oena     (pin_SDA_oena), 
    .pin_SDA_oena_rise(pin_SDA_oena_rise), 
    .i2c_spike_ok     (i2c_spike_ok),
    .i2c_hs_enabled   (i2c_hs_enabled),
    .int_start_seen   (int_start_seen), 
    .int_start_err    (int_start_err), 
    .int_da_matched   (match_da), // 在下面如果处理则屏蔽 
    .int_sa_matched   (match_sa), // "
    .int_7e_matched   (int_7e_matched), 
    .int_in_STOP      (int_in_STOP), 
    .int_event_sent   (int_event_sent), 
    .int_event_ack    (int_event_ack), 
    .int_in_evproc    (int_in_evproc),
    .cf_SlvNack       (cf_SlvNack),
    .tb_data_valid    (tb_data_valid), 
    .tb_datab         (tb_datab), 
    .tb_end           (tb_end), 
    .tb_datab_ack     (tb_datab_ack), 
    .tb_urun_nack     (stb_urun_nack), 
    .tb_urun          (stb_urun), 
    .tb_term          (stb_term),
    .fb_data_use      (fb_data_use), 
    .fb_datab         (fb_datab), 
    .fb_datab_done    (fb_datab_done), 
    .fb_datab_err     (fb_datab_err), 
    .fb_orun          (fb_orun), 
    .fb_s0s1_err      (fb_s0s1),
    .rd_abort         (rd_abort),
    .state_in_CCC     (state_in_CCC_v), 
    .next_ccc         (next_ccc),
    .next_hdr         (next_hdr),
    .in_ddr           (in_ddr),
    .ccc_handling     (ccc_handling),
    .int_ccc_handled  (int_ccc_handled), // 为我们具体
    .ccc_get          (ccc_get),
    .ccc_is_read      (ccc_is_read),
    .ccc_real_START   (ccc_real_START),
    .ccc_handled      (ccc_handled),
    .ccc_uh_mask      (ccc_uh_mask),
    .int_ccc          (int_ccc),
    .state_accslv     (state_accslv),
    .state_rd         (state_req_rd),
    .state_wr         (state_req_wr),
    .state_dir        (state_is_read),
    .in_daa_mode      (state_daa),
    .done_buff        (done_buff), 
    .done_rdy         (done_rdy), 
    .ccc_tb_now       (ccc_tb_now),
    .ccc_tb_data      (ccc_tb_data),
    .ccc_tb_continue  (ccc_tb_continue),
    .is_9th           (is_9th),
    .opt_s0ignore     (cf_s0ignore),
    .daa_active       (daa_active), 
    .daa_act_ok       (daa_act_ok),
    .daa_inp_drv      (daa_inp_drv), 
    .daa_inp_bit      (daa_inp_bit), 
    .daa_acknack      (daa_acknack),
    .daa_done         (daa_done), 
    .dyn_addr         (dyn_addr), 
    .opt_static_addr  (opt_static_addr),// 将是网络、配置或上面的常量0
    .SetMappedDASA    (cf_SetMappedDASA),// 如果映射SA/DA
    .SetSA10b         (cf_SetSA10b),    // 如果[1]是SA 10位
    .cf_vgpio         (cf_vgpio[8]),    // 如果CCC基于（所以不是单独的DA）
    .vgpio_done       (raw_vgpio_done),
    .vgpio_byte       (raw_vgpio_byte),
    .ccc_vgpio_done   (ccc_vgpio_done),
    .ccc_vgpio        (ccc_vgpio),
    .hdr_newcmd       (raw_hdr_newcmd), // 在SCL域中脉冲如果有新CMD
    .hdr_cmd          (raw_hdr_cmd),    // 新CMD的cmd值
    .raw_dasa_ena     (raw_dasa_ena),   // 如果自动-DASA匹配则脉冲
    .map_sa_idx       (map_dasa_idx),   // 如果raw_dasa_ena的索引
    .opt_match_idx    (opt_match_idx),  // 如果使用映射的最后3个匹配
    .i2c_dev_rev      (i2c_dev_rev),    // 如果扩展i2c
    .i2c_sw_rst       (i2c_sw_rst),     // "
    .event_pending    (event_pending), 
    .force_sda        (force_sda), 
    .ibi_has_byte     (ibi_has_byte), 
    .opt_ibi_byte     (opt_ibi_byte),
    .ibi_timec        (ibi_timec),
    .ibi_timec_marks  (ibi_timec_marks),
    .ibi_timec_sel    (ibi_timec_sel),
    .ibi_extdata      (ibi_extdata),
    .ibi_mapidx       (ibi_mapidx),
    .ibi_in_extdata   (ibi_in_extdata),
    .ddr_fb_ctrl      (ddr_fb_ctrl),
    .ddr_fb_MSB       (ddr_fb[0]),
    .ddr_fb_LSB       (ddr_fb[1]),
    .ddr_tb_ctrl      (ddr_tb_ctrl),
    .ddr_tb_bits      (ddr_tb_bits), 
    .ddr_tb_oe        (ddr_tb_oe), 
    .obit_cnt         (obit_cnt), 
    .byte_1st         (byte_1st), 
    .eng_debug_observ (eng_debug_observ),
    .byte_2nd         (byte_2nd),
    .scan_no_rst      (scan_no_rst)
    );
  assign ccc_byte      = byte_1st;
  assign willbe_ccc    = |next_ccc;
  assign state_in_ccc  = |state_in_CCC_v[`CF_DIRECT:`CF_BCAST]; // 广播或直接
  assign state_in_daa  = state_in_CCC_v[`CF_DAA_M]; 
  assign in_HDR_mode   = state_in_CCC_v[`CF_HDR_M];
  assign state_in_hdr  = in_HDR_mode; 
  assign state_dir     = state_is_read | (in_HDR_mode & ddr_read);
  // 状态: [6]=HDR, [5]=ENTDAA, [4]=写, [3]=读/IBI, [2]=CCChandle, [1]=W/R, [0]=忙
  assign state_req = {state_in_hdr, state_daa, state_req_wr, 
                      state_in_hdr?ddr_read:state_req_rd, ccc_handled&~int_in_STOP, 
                      state_accslv, ~int_in_STOP};
    // 只有在未处理时才引起匹配中断
  assign int_da_matched= match_da & ~ccc_handled;
  assign int_sa_matched= match_sa & ~ccc_handled;
  assign raw_daa_ena   = raw_dasa_ena | raw_map_daa;
  assign map_daa_da    = raw_map_daa ? map_da : new_da; // 获取DASA/DAA更改的新DA
  assign map_sa_idx    = raw_map_daa ? daa_map_idx : map_dasa_idx;

  //
  // DAA从机支持 - 支持ENTDAA/RSTDAA处理与引擎的帮助
  // -- 还有来自CCC的SETNEWDA和SETDASA的输入
  // -- 此块拥有DA（动态地址）
  //
  i3c_daa_slave #(.ENA_ID48B(ENA_ID48B), .ID_48B(ID_48B), 
                  .ENA_MAPPED(ENA_MAPPED),.MAP_CNT(MAP_CNT),.MAP_I2CID(MAP_I2CID),
                  .MAP_DA_AUTO(MAP_DA_AUTO), .MAP_DA_DAA(MAP_DA_DAA),
                  .ID_AS_REGS(ID_AS_REGS), .ID_BCR(ID_BCR),
                  .ID_DCR(ID_DCR), .PIN_MODEL(PIN_MODEL))
    daa 
    (
    .clk_SCL_n        (clk_SCL_n), 
    .clk_SCL          (clk_SCL), 
    .RSTn             (RSTn), 
    .pin_SDA_in       (pin_SDA_in), 
    .state_in_CCC     (state_in_CCC_v), 
    .daa_active       (daa_active), 
    .daa_act_ok       (daa_act_ok),
    .daa_inp_drv      (daa_inp_drv), 
    .daa_inp_bit      (daa_inp_bit), 
    .daa_done         (daa_done), 
    .daa_acknack      (daa_acknack),
    .dyn_addr         (dyn_addr), 
    .dyn_addr_chg     (dyn_addr_chg),
    .dyn_chg_cause    (dyn_chg_cause),
    .id64_cnt         (ccc_counter),
    .daa_out          (daa_id),
    .raw_map_daa      (raw_map_daa),
    .daa_map_idx      (daa_map_idx),
    .map_da           (map_da),
    .map_chg          (raw_dasa_ena|raw_setaasa),
    .set_da           (set_da & ~raw_dasa_ena), // 设置除非映射
    .new_da           (new_da),
    .set_aasa         (opt_setaasa),
    .old_sa           (opt_static_addr),
    .cf_IdInst        (cf_IdInst), 
    .cf_IdRand        (cf_IdRand), 
    .cf_Partno        (cf_Partno), 
    .cf_IdBcr         ({cf_IdBcr[7:6],cf_IdBcr[5]|cf_DdrOK,cf_IdBcr[4:0]}), // 没有三态
    .cf_IdDcr         (cf_IdDcr),
    .cf_IdVid         (cf_IdVid),
    .cf_DdrOK         (cf_DdrOK),
    .cf_SetDA         (cf_SetDA),
    .map_daa_use      (map_daa_use),
    .map_daa_dcr      (map_daa_dcr),
    .map_daa_pid      (map_daa_pid),
    .SetMappedDASA    (cf_SetMappedDASA),
    .scan_no_rst      (scan_no_rst)
    );

  //
  // 从机的CCC处理以支持参数请求的尽可能多的CCC
  // -- 最小化处理SETDASA和SETNEWDA
  // -- 可以为其他启用
  //
  i3c_ccc_slave #(.ENA_CCC_HANDLING(ENA_CCC_HANDLING), .ID_AS_REGS(ID_AS_REGS),
                  .RSTACT_CONFIG(RSTACT_CONFIG),.MAX_DS_WR(MAX_DS_WR), 
                  .MAX_DS_RD(MAX_DS_RD), .MAX_DS_RDTURN(MAX_DS_RDTURN),
                  .ENA_MASTER(ENA_MASTER), .ENA_HDR(ENA_HDR), .PIN_MODEL(PIN_MODEL),
                  .ENA_TIMEC(ENA_TIMEC),.TIMEC_FREQ_ACC(TIMEC_FREQ_ACC))
    ccc 
    (
    .clk_SCL_n        (clk_SCL_n), 
    .clk_SCL          (clk_SCL), 
    .RSTn             (RSTn), 
    .pin_SDA_in       (pin_SDA_in), 
    .state_in_CCC     (state_in_CCC_v), 
    .next_ccc         (next_ccc & {1'b1,~fb_datab_err}),// 在奇偶校验错误时掩码广播
    .dyn_addr         (dyn_addr), 
    .int_start_seen   (int_start_seen), 
    .int_da_matched   (match_da), 
    .int_7e_matched   (int_7e_matched), 
    .int_in_STOP      (int_in_STOP), 
    .ccc_tb_now       (ccc_tb_now),
    .ccc_tb_data      (ccc_tb_data),
    .ccc_tb_continue  (ccc_tb_continue),
    .is_9th           (is_9th),
    .idata_byte       (done_buff), 
    .idata_done       (done_rdy),
    .daa_active       (daa_active), 
    .ccc_counter      (ccc_counter),
    .daa_id           (daa_id),
    .set_da           (set_da),
    .new_da           (new_da),
    .ccc_handling     (ccc_handling),   // 将处理CCC
    .ccc_handled      (ccc_handled),    // 正在处理CCC
    .int_ccc_handled  (int_ccc_handled),
    .ccc_uh_mask      (ccc_uh_mask),    // 未处理的掩码
    .ccc_get          (ccc_get),
    .ccc_is_read      (ccc_is_read),
    .ccc_real_START   (ccc_real_START),
    .opt_state_AS     (opt_state_AS),   // 如果处理则活动状态
    .opt_ev_mask      (opt_ev_mask),    // 已知的事件掩码
      // 现在启用时的读写长度
    .cf_MaxRd         (cf_MaxRd),
    .cf_MaxWr         (cf_MaxWr),
    .cf_RstActTim     (cf_RstActTim),
    .cf_vgpio         (cf_vgpio),       // 如果VGPIO是CCC则使用
    .cf_CccMask       (cf_CccMask),     // 选择哪些未处理的CCC传递上去
    .ccc_vgpio_done   (ccc_vgpio_done),
    .ccc_vgpio        (ccc_vgpio),
    .opt_ChgMaxRd     (opt_ChgMaxRd),
    .opt_ChgMaxWr     (opt_ChgMaxWr),
    .opt_MaxRdWr      (opt_MaxRdWr),
    .opt_TimeC        (opt_TimeC),
    .opt_timec_oflow  (timec_oflow),
    .opt_timec_oflow_clr(timec_oflow_clr),
    .cf_TCclk         (cf_TCclk),
    .opt_timec_sync   (opt_timec_sync),
    .cf_MasterAcc     (cf_MasterAcc),   // 仅用于M+S，从机可以接受主设备权
    .opt_MasterAcc    (opt_MasterAcc),
    .opt_slvr_reset   (opt_slvr_reset), // 从机复位控制
    .i_rst_slvr_reset (i_rst_slvr_reset),
    .clr_slvr_cause   (clr_slvr_cause),
    .opt_setaasa      (opt_setaasa),
    .no_setaasa       (&event_pending[1:0]), // 如果HJ则没有SETAASA
    .bus_err          ((fb_datab_done & fb_datab_err) | (|fb_ddr_errs)),
    .event_pending    (event_pending),
    .sraw_ActMode     (sraw_ActMode),
    .sraw_PendInt     (sraw_PendInt),
    .sraw_StatusRes   (sraw_StatusRes),
    .scan_no_rst      (scan_no_rst)
  );
  assign raw_setaasa = MAP_DA_AUTO[`MAPDA_AASA_b] & opt_setaasa; // 仅如果启用
  assign raw_rstdaa  = |MAP_DA_AUTO[2:0] &
                          (state_in_CCC_v[`CF_GRP_b] == `CFG_RSTDAA_US);

  //
  // 退出模式检测器以退出HDR
  // -- 在所有时间工作
  // -- 在CCC状态显示我们在HDR中时至关重要，因为这锁定了SDR模式
  //
  i3c_exit_detector #(.ENA_HDR(ENA_HDR))
    exit_detect
    (
    .clk_SDA_n        (clk_SDA_n), 
    .clk_SDA          (clk_SDA), 
    .clk_SCL          (clk_SCL), 
    .RSTn             (RSTn), 
    .pin_SCL_in       (pin_SCL_in), 
    .in_HDR_mode      (in_HDR_mode), 
    .HDR_restart_ack  (HDR_restart_ack), 
    .oHDR_exit        (HDR_exit), 
    .oHDR_restart     (HDR_restart),
    .scan_no_rst      (scan_no_rst)
  );
  assign fb_hdr_exit = HDR_exit;


  //
  // 5. 我们可选地实例化HDR块
  //

  //
  // 如果启用，则在此处实例化HDR-DDR
  // -- 注意它使用SDR引擎做很多工作
  // 
  generate if (ENA_HDR[`HDR_DDR_b]) begin : ddr_instance
    i3c_hdr_ddr_slave #(.PIN_MODEL(PIN_MODEL))
      hdr_ddr
      (
      .clk_SCL_n        (clk_SCL_n), 
      .clk_SCL          (clk_SCL), 
      .RSTn             (RSTn), 
      .pin_SDA_in       (pin_SDA_in), 
      .in_DDR           (in_ddr&daa_id[8+5]), // daa_id[8+5]以确保HDR允许
      .HDR_restart      (HDR_restart), 
      .restart_ack      (HDR_restart_ack),
      .HDR_exit         (HDR_exit), 
      .rd_abort         (rd_abort),     // 如果时钟停止太久则杀死
      .dyn_addr         (dyn_addr), 
      .ddr_fb_ctrl      (ddr_fb_ctrl), 
      .ddr_fb           (ddr_fb), 
      .ddr_tb_ctrl      (ddr_tb_ctrl), 
      .ddr_tb_bits      (ddr_tb_bits), 
      .ddr_tb_oe        (ddr_tb_oe), 
      .bit_cnt          (obit_cnt[1:0]), // 仅下2位因为以2s递增 
      .byte_1st         (byte_1st), 
      .byte_2nd         (byte_2nd), 
      .tb_datab         (tb_datab),
      .tb_empty         (~tb_data_valid), 
      .tb_end           (tb_end),
      .int_ddr_matched  (int_ddr_matched),
      .ddr_read         (ddr_read),
      .fb_ddr_errs      (fb_ddr_errs),
      .tb_term          (dtb_term),
      .tb_urun_nack     (dtb_urun_nack),
      .tb_urun          (dtb_urun),
      .cf_HdrCmd        (cf_HdrCmd),      // 如果HDR CMD发送到MMR
      .hdr_newcmd       (raw_hdr_newcmd), // 在SCL域中脉冲如果有新CMD
      .scan_no_rst      (scan_no_rst)
      );
    // 下溢是SDR和DDR下溢的OR
    assign tb_urun_nack = stb_urun_nack | dtb_urun_nack;
    assign tb_urun      = stb_urun | dtb_urun;
    assign tb_term      = stb_term | dtb_term;
  end else begin
    assign ddr_fb_ctrl = 0;
    assign ddr_fb      = 0;
    assign ddr_tb_ctrl = 0;
    assign ddr_tb_bits = 0;
    assign ddr_tb_oe   = 0;
    assign HDR_restart_ack = 1'b1;      // 仅在HDR支持时使用
    assign int_ddr_matched = 0;
    assign ddr_read    = 0;
    assign fb_ddr_errs = 0;
    assign tb_urun_nack= stb_urun_nack;
    assign tb_urun     = stb_urun;
    assign tb_term     = stb_term;
    assign raw_hdr_newcmd = 0;
  end endgenerate

endmodule
