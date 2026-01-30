// i3c_apb_wrapper.v
// 模块: i3c_apb_wrapper
// 功能: 带有 APB 总线用于 MMR 的 MIPI I3C 从设备支持
//       该模块包含用于 I3C 与 APB 总线寄存器接口的封装。
//       如果不使用 APB，请参阅其他封装。
//       作为外部封装，该模块支持4个需求：
//       1. 实例化从设备
//       2. 使用 APB 实例化寄存器接口
//       3. 使用系统时钟（本例中为 PCLK）处理时钟域交叉，尽管在完整封装中较低层处理。
//       4. 根据需要管理数据缓冲/FIFO，尽管在完整封装中较低层处理。
//       该模块的具体功能由参数控制，如微架构规范和参数文件所述。

`include "i3c_params.v"                 // 本地参数/常量

module i3c_apb_wrapper #(
    // 参数由上层驱动，以控制该模块的构建和操作
    parameter ENA_ID48B       = `ID48B_CONST, // 常量 vs. 寄存器
    parameter  ID_48B         = 48'h0,  // 必须从上层填充部分或全部48位 ID
    parameter ID_AS_REGS      = 12'd0,  // 用于 ID 和其他用途的寄存器（例如掩码）
    // 注意 BCR 在下面，因此可以使用其他参数
    parameter  ID_DCR         = 8'd0,   // 如果不是来自寄存器，则从上层填充 DCR
    parameter ENA_SADDR       = `SADDR_NONE, // 无、常量、寄存器/网络
    parameter  SADDR_P        = 0,      // 7 位，如 6:0
    parameter ENA_MAPPED      = 5'd0,   // 如果允许额外的 DA/SA 及相关
    parameter  MAP_CNT        = 4'd1,   // 允许的额外 DA/SA 数量
    parameter  MAP_I2CID      = 24'd0,  // 如果 I2C 扩展了 DevID，则 !=0
               // DA_AUTO: PID[pos:0],DCR, MMR, res  DAA, AASA,DASA,
    parameter  MAP_DA_AUTO    = {5'd1,1'b0,1'b0,3'd0,1'b0,1'b0,1'b0},
    parameter  MAP_DA_DAA     = 0,      // 如果不是 MMR 且 PID/DCR !=0，则为位数组
    parameter ENA_IBI_MR_HJ   = 0,      // 0 表示无事件，否则为事件掩码
    parameter  CLK_SLOW_BITS  = 6,      // 总线可用计数所需的位数
    parameter  CLK_SLOW_MATCH = 6'd47,  // 计数：例如，如果 CLK 为 48MHz，则47对应1us（0 rel）
    parameter  CLK_SLOW_HJMUL = 10'd1000,// MATCH 的数量（常量或寄存器）对应1ms（1 rel）
      // 下一个是错误处理。如果启用了 IBI，则默认读取中止
    parameter  ERROR_HANDLING = 3'd0|((|ENA_IBI_MR_HJ)<<`ERR_RDABT_b),
    parameter  ENA_TIMEC      = 6'b000010,  // 时钟寄存器、保留、保留、模式1、模式0、同步
    parameter  TIMEC_FREQ_ACC = {8'd0,8'd0},// 频率 H 为 0.5MHz 的倍数，然后精度为 0.1% 的倍数
    parameter ENA_CCC_HANDLING= 6'd0,   // 传递下来以支持哪些 CCC
    parameter RSTACT_CONFIG   = 26'd0,  // 从设备复位 RSTACT CCC 配置（见 RSTA_xxx_b 字段）
    parameter MAX_RDLEN       = 0,      // 默认 S->M 长度最大值
    parameter MAX_WRLEN       = 0,      // 默认 M->S 长度最大值
    parameter MAX_DS_WR       = 0,      // M->S 的数据速度限制
    parameter MAX_DS_RD       = 0,      // S->M 的数据速度限制
    parameter MAX_DS_RDTURN   = 0,      // S->M 读请求的延迟需求
    parameter SEL_BUS_IF      = 5'h07,  // 默认为完整 APB，但不是 DMA 或半双工（除非 FIFO）
    parameter  DMA_TYPE       = 2'd0,   // 0=req/ack 在前，2=req/ack 在后，3=req/ack 4相
    parameter FIFO_TYPE       = 4'b10_00,// 使用的 FIFO 类型（包括外部）和保持
    parameter  EXT_FIFO       = 3'd0,   // 如果使用外部 FIFO，则为其类型
    parameter  ENA_TOBUS_FIFO = 0,      // 到总线的深度，从2开始以2的幂次方
    parameter  ENA_FROMBUS_FIFO=0,      // 从总线来的深度，从2开始以2的幂次方
    parameter ENA_HDR         = 0,      // HDR 模式的使能
    parameter BLOCK_ID        = 0,      // 如果不为0，则允许 ID 寄存器
    parameter ENA_MASTER      = 0,      // 1 表示使用主设备
    parameter PIN_MODEL       = `PINM_COMBO, // 组合逻辑引脚使用
      // BCR 根据参数自动填充。但是，不能离线或桥接
    parameter  ID_BCR         = (|ENA_MASTER<<6) | (|ENA_HDR<<5) | ((ENA_IBI_MR_HJ&8'h03)<<1) |
                                (|(MAX_DS_RDTURN|MAX_DS_WR|MAX_DS_RD)), // 限制在 [0]
    parameter priv_sz         = PIN_MODEL[1] ? (ENA_HDR[0] ? 3 : 1) : 0,// 如果外部焊盘+ddr 则更宽
    parameter TOP_DBG_MX      = 0       // 用于调试观察器
  )
  (
  // 定义时钟和复位
  input               PRESETn,          // 系统复位
  input               PCLK,             // 寄存器和同步的系统时钟
  input               CLK_SLOW,         // 可能与 PCLK 相同或更慢
  output              slow_gate,        // 1 表示可能门控 CLK_SLOW
  input               CLK_SLOW_TC,      // 时间控制时钟，可能与 CLK_SLOW 相同
  output              tc_slow_gate,     // 1 表示可能门控 CLK_SLOW_TC；如果共用时钟，则与 slow_gate 相与
    // 未来：input  clk_FastTernary,  // 如果使用三态，则为三态时钟
  // 定义 APB 总线
  input               PSEL,             // 选择（与 penable 一起）
  input               PENA,             // 实际读或写周期使能
  input        [11:0] PADDR,            // 地址 - 字对齐
  input               PWRITE,           // 是写（数据阶段和地址相同）
  output       [31:0] PRDATA,           // 读寄存器
  input        [31:0] PWDATA,           // 写寄存器
  output              PREADY,           // 停顿
  output              PSLVERR,          // 如果 ERROR_HANDLING 使能，则可选
  // 定义 IRQ 和 DMA - 如果不使用，则不连接
  output              irq,              // 中断，当 int_pin 时保持
  output              dma_req_tb,       // 请求到总线数据
  output              dma_req_fb,       // 请求获取来自总线的数据
     // DMA 确认的使用取决于系统中 DMA 的工作方式：
     // - 如果 DMA_TYPE=0，则 ACK 为1个时钟（PCLK）并在 MMR 访问之前（例如 WDATAB 和 RDATAB）。
     // - 如果 DMA_TYPE=2，则 ACK 为1个时钟并与 MMR 访问一起/之后。
     // - 如果 DMA_TYPE=3，则 REQ/ACK 为4相，ACK 与 MMR 访问一起/之后。
     // - 如果 DMA_TYPE=1 保留（触发已弃用）。
     // 注意：如果不使用 dma_req_tb_ack_last，则应置0。否则，当最后一个字节（END）时，DMA 必须在 WDATAB/WDATAH 写入期间保持1。
     // 这将视为 WDATABE/WDATAHE。
  input               dma_req_tb_ack,   // 到总线请求的确认
  input               dma_req_fb_ack,   // 来自总线请求的确认
  input               dma_req_tb_ack_last,// 注意：如果不使用，则置0
  output              wakeup,           // 如果使能，则与 irq 一起保持（用于唤醒深度睡眠）
    // 接下来的两个可能在集成中合并 - 作为一个唤醒
  output              raw_wakeup_irq,   // 在 SCL 中 SA/DA 匹配时唤醒：为启用的中断启动 PCLK
  output              raw_wakeup_dma,   // 同上，但如果使能了 dma 请求
  // 定义引脚
  input               pin_SCL_in,       // SCL：通常是时钟输入，如果是三态则为数据
  output              pin_SCL_out,      // SCL：仅当三态时驱动
  output              pin_SCL_oena,     // SCL：仅当三态时驱动
  input               pin_SDA_in,       // SDA：M->S
    // 注意：当 PIN_MODEL==`PINM_EXT_REG 时，接下来的3个特殊使用
    //       它们馈送靠近焊盘的3个触发器
  output              pin_SDA_out,      // SDA：S->M 读取时
  output              pin_SDA_oena,     // SDA：S->M 读取时
  output  [priv_sz:0] pin_SDA_oena_rise,// 特殊：如果 EXT_REG，则在 SCL 上升沿，否则为0
    // 下一个仅当 SDA 和 SCL 焊盘具有 i2c 50ns 尖峰滤波器且可以通过网络打开/关闭时使用
  output              i2c_spike_ok,     // 为1允许 i2c 尖峰滤波器
  output              i2c_hs_enabled,   // 如果使能，表示在 I2C HS 模式
  // 当不是来自寄存器时，可选的网络进入或来自系统
  // 如果不使用，则不连接输出并将输入置0
  input         [7:0] ext_SlvSA,        // 输入从设备地址 [0]=1 如果有效
  output        [7:0] out_SlvDA,        // DA 如果有，[0]=1 如果值有效
    // 注意：以下在 SCL 时钟域中。[11]->1 表示变化；其余在之后稳定
  output       [13:0] raw_timec_sync,   // 系统处理的 SYNC tc。[13]=clk
  // 接下来的3个用于 SlaveReset 如果使能/使用。复位检测器在模块外部
  output        [3:0] raw_slvr_reset,   // SlaveRst RSTACT 动作在 [2:0]，清除在 [3]
  input               iraw_rst_slvr_reset, // 在 SCL 域中清除 SlaveRst 控制
    // 注意：如果由应用程序中断处理，则模块的从设备复位连接到复位检测器，否则置0并由系统控制处理。
  input               iraw_slvr_irq,    // 模块复位请求：设置 IRQ 和原因
  output              raw_i2c_slvrst,   // 如果扩展 i2c - 从设备复位
    // 接下来的两个仅与 ENA_MAPPED[VGPIO] 一起使用
  output              vgpio_done,       // 如果 VGPIO 使能，则完成脉冲
  output        [7:0] raw_vgpio_byte,   // 字节值如果 "
  // 可选的外部 FIFO：
    // 到总线表示从设备到主设备
  input               ixf_tb_avail,     // 1 表示字节可用于到总线
  input               ixf_tb_last,      // 字节是消息的最后一个
  input         [7:0] ixf_tb_data,      // 当 avail=1 时的实际数据
  output               oxf_tb_start,     // 当 ixf_tb_avail==1 且 START 时脉冲
  output              oxf_tb_used,      // 当使用 tb_data 时脉冲
    // 从总线表示从主设备进入从设备
  input               ixf_fb_free,      // 1 表示有空间从总线获取字节
  output              oxf_fb_req,       // 当数据将被获取时脉冲（且 free=1）
  output        [7:0] oxf_fb_data,      // 当 req=1 时来自总线的数据
  output              oxf_fb_eof,       // 在重复 START 或 STOP 中帧结束
  // 现在用于“调试”类型观察的通用端口
  `ifdef ENA_DBG_OBSERVE
  output[TOP_DBG_MX:0]out_debug_observ, // 观察器输出
  `endif
  // 以下特殊用于 D 输入复位
  `ifdef USE_D_RESET
  input               d_reset_r,
  `endif
  // 现在扫描相关
  input               scan_single_clock,// 用于基于引脚的时钟的扫描时钟单时钟域
  input               scan_clk,         // 如果扫描中，用于引脚时钟的时钟
  input               scan_no_rst,      // 防止分层复位
  input               scan_no_gates     // 防止架构时钟门控
  );
  // 以下用于可选的 MAP 自动 DAA
  localparam   [7:0] PID_CNT = MAP_DA_AUTO[`MAPDA_DAA_PID_lb+4:`MAPDA_DAA_PID_lb];

  // 总线特殊
  wire   wr_err;                        // 在寄存器接口中设置
  assign PSLVERR = wr_err & ERROR_HANDLING[`ERR_WR_RO_b];

  //
  // 线网 - 我们定义去往/来自寄存器的线网，并在此或通过实例处理。
  //

  // 配置是来自寄存器的值，保持不变
  wire                cf_SlvEna;
  wire                cf_SlvNack;
  wire          [7:0] cf_SlvSA;
  wire          [3:0] cf_IdInst;
  wire                cf_IdRand;
  wire                cf_Offline;
  wire         [31:0] cf_Partno;
  wire          [7:0] cf_IdBcr;
  wire          [7:0] cf_IdDcr;
  wire         [14:0] cf_IdVid;
  wire                cf_DdrOK;
  wire                cf_TspOK;
  wire                cf_TslOK;
  wire         [11:0] cf_MaxRd;
  wire         [11:0] cf_MaxWr;
  wire         [23:0] cf_RstActTim;
  wire          [7:0] cf_BAMatch;
  wire         [15:0] cf_TCclk;
  wire                cf_s0ignore;
  wire                cf_matchss;
  wire          [1:0] cf_HdrCmd;
  wire          [6:0] cf_CccMask;
  wire          [8:0] cf_vgpio;
  wire    [MAP_CNT-1:0] map_daa_use;      // 哪些 MAP 是自动 DAA
  wire  [(MAP_CNT*8)-1:0] map_daa_dcr;      // 如果 MAP 自动 DAA，则为 DCR
  wire  [(MAP_CNT*PID_CNT)-1:0] map_daa_pid; // 如果 MAP 自动 DAA，则为 PID 部分

    // 下一个是特殊情况：当从设备未使能时更改，以及如果使用的映射寄存器
  wire          [7:0] SetDA;
  wire [(MAP_CNT*10)-1:0] SetMappedDASA;
  wire          [2:0] SetSA10b;
  wire                cf_IbiExtData;
  wire          [3:0] cf_IbiMapIdx;
  wire          [2:0] cf_i2c_dev_rev;
  // 现在来自 SCL 域原始的特殊状态（同步由寄存器块完成）
  wire        [21:20] raw_ActState;
  wire        [19:16] raw_EvState;
  wire          [2:0] raw_TimeC;
  wire          [6:0] raw_Request;
  wire          [7:0] raw_DynAddr;
  wire          [2:0] raw_DynChgCause;
  wire         [12:0] raw_match_idx;
  wire                raw_matched;
  wire                hdr_new_cmd;
  wire          [7:0] raw_hdr_cmd;
  wire                map_rstdaa;
  wire                map_setaasa;
  wire                map_daa_ena;
  wire          [3:0] map_sa_idx;
  wire          [7:1] map_daa_da;
  // 现在来自相同 PCLK 域的输入和输出寄存器
  wire         [19:8] reg_IntEna;
  wire          [5:0] reg_DmaCtrl;
  wire         [19:8] inp_IntStates;
  wire          [2:0] reg_EvPend;
  wire          [7:0] reg_EvIbiByte;
  wire                inp_EvNoCancel;
  wire        [22:20] inp_EvDet;
  wire          [5:0] inp_GenErr;
  wire         [11:8] inp_DataErr;
  wire          [5:0] msk_GenErr;
  wire         [11:8] msk_DataErr;
  wire         [19:8] reg_clrIntStates;
  wire          [5:0] reg_clrGenErr;
  wire         [11:8] reg_clrDataErr;
  wire                reg_holdOErr;
  wire                inpflg_MaxRd;
  wire                inpflg_MaxWr;
  wire         [11:0] inp_MaxRW;
  wire                reg_TbEnd;
  wire                reg_TbFlush;
  wire                reg_FbFlush;
    // 接下来的4个仅在 FIFO 使能时使用
  wire          [5:4] reg_TxTrig;
  wire          [7:6] reg_RxTrig;
  wire        [20:16] inp_TxCnt;
  wire        [28:24] inp_RxCnt;
  wire                inp_TxFull;
  wire                inp_RxEmpty;
    // 接下来的3个将仅使用参数允许的大小
  wire          [1:0] regflg_wr_cnt;
  wire          [7:0] reg_wdata;
  wire          [1:0] regflg_rd_cnt;
  // 接下来的两个用于寄存器接口
  wire          [7:0] notify_fb_data;
  wire          [8:7] reg_ActMode;
  wire          [3:0] reg_PendInt;
  wire         [15:8] reg_StatusRes;
  // 现在来自寄存器的 IBI FIFO
  wire         [10:0] ibi_wr_fifo;
  wire                ibi_wr_ack;

  //
  // 允许导出调试观察器数据
  // -- 默认存根赋值0，但允许添加线网
  // -- 注意 FULL_DBG_MX 由包含文件定义，与 full_debug_observ 相同
  //
  `ifdef ENA_DBG_OBSERVE
   `include "dbg_observ_top.v"   // 集成器替换 inc 文件内容
  `else
   localparam FULL_DBG_MX = 0;
   wire [FULL_DBG_MX:0] full_debug_observ;
  `endif
  //

  // IRQ 和 DMA（如果使用）
  assign irq        = |(reg_IntEna & inp_IntStates); // 如果有任何被屏蔽且挂起，则发出信号
  assign wakeup     = |(reg_IntEna[12:11] & inp_IntStates[12:11]); // 数据/FIFO 唤醒
  assign raw_wakeup_irq = raw_matched & reg_IntEna[9]; // 如果使能了 irq，则唤醒
  assign raw_wakeup_dma = raw_matched & |reg_DmaCtrl[3:0];// 如果使能了 DMA，则唤醒

  generate if (SEL_BUS_IF[`SBIF_DMA_b]) begin : DMA_supp
    wire [1:0] rx_fullness, tx_avail;
    i3c_dma_control #(.DMA_TYPE(DMA_TYPE)) dma_ctrl(
       .PRESETn         (PRESETn),
       .PCLK            (PCLK),
       .dma_ctrl        (reg_DmaCtrl),
       .rx_trig         (inp_IntStates[`IS_RXPEND]),
       .tx_trig         (inp_IntStates[`IS_TXPEND]),
       .rx_fullness     (rx_fullness),
       .tx_avail        (tx_avail),
       .pread           (PSEL&PENA&~PWRITE),// 特殊用于重新武装 RX
       .dma_req_tb      (dma_req_tb),
       .dma_req_fb      (dma_req_fb),
       .dma_req_tb_ack  (dma_req_tb_ack),
       .dma_req_fb_ack  (dma_req_fb_ack)
       );
    // 下一个是当此生成块未使用时防止错误（仍然被解析）
    localparam FAKE_TB_WID = |ENA_TOBUS_FIFO ? ENA_TOBUS_FIFO : 2; // 如果0则停止错误
    // 计算满度和可用计数
    // RX 的计数表示如果0、1、2或超过2个字节在那里
    assign rx_fullness = |inp_RxCnt[28:26] ? 2'd3 : inp_RxCnt[25:24];
    // TX 的计数更复杂，因为它是可用字节数
    // 允许0、1、2或超过2个可用
    // 对于乒乓情况（如果使用 DMA 则不常见），如果为空则使用2，如果满则为0，如果不满则为1。3不可能。
    assign tx_avail    = ~|ENA_TOBUS_FIFO ?
                         (~|inp_TxCnt[17:16]?2'd2:inp_TxCnt[17]?2'd0:2'd1) :
                         (inp_TxCnt[16+FAKE_TB_WID] ? 2'd0 :
                          &inp_TxCnt[(16+FAKE_TB_WID-1):16] ? 2'd1 :
                          reg_DmaCtrl[5] ? // 需要将3B视为与2B相同（半字）
                            ((inp_TxCnt[(16+FAKE_TB_WID-1):16]>=((1<<FAKE_TB_WID)-3)) ?
                             2'd2 : 2'd3) :
                            ((inp_TxCnt[(16+FAKE_TB_WID-1):16]==((1<<FAKE_TB_WID)-2)) ?
                             2'd2 : 2'd3));
  end else begin
    assign dma_req_tb  = 1'b0;
    assign dma_req_fb  = 1'b0;
  end
  endgenerate

  // 下一个是特殊 D 输入复位方案，由一个项目使用
  wire RSTn;
 `ifdef USE_D_RESET
    assign RSTn = PRESETn & (~d_reset_r | scan_no_rst); // 对于除 PCLK 域外的其他域是异步复位
 `else
    assign RSTn = PRESETn;
 `endif

  //
  // 完整封装处理从设备、CDC 和 FIFO 的数据缓冲
  //
  i3c_full_wrapper #(.ENA_ID48B(ENA_ID48B),.ID_48B(ID_48B),
                     .ID_AS_REGS(ID_AS_REGS),.ID_BCR(ID_BCR),.ID_DCR(ID_DCR),
                     .ENA_SADDR(ENA_SADDR),.SADDR_P(SADDR_P),
                     .ENA_MAPPED(ENA_MAPPED),.MAP_CNT(MAP_CNT),.MAP_I2CID(MAP_I2CID),
                       .MAP_DA_AUTO(MAP_DA_AUTO), .MAP_DA_DAA(MAP_DA_DAA),
                     .ENA_IBI_MR_HJ(ENA_IBI_MR_HJ),
                       .CLK_SLOW_BITS(CLK_SLOW_BITS),.CLK_SLOW_MATCH(CLK_SLOW_MATCH),
                       .CLK_SLOW_HJMUL(CLK_SLOW_HJMUL), .ERROR_HANDLING(ERROR_HANDLING),
                     .ENA_CCC_HANDLING(ENA_CCC_HANDLING), .RSTACT_CONFIG(RSTACT_CONFIG),
                       .MAX_RDLEN(MAX_RDLEN),
                       .MAX_WRLEN(MAX_WRLEN), .MAX_DS_WR(MAX_DS_WR), .MAX_DS_RD(MAX_DS_RD),
                       .MAX_DS_RDTURN(MAX_DS_RDTURN),
                     .SEL_BUS_IF(SEL_BUS_IF),
                     .FIFO_TYPE(FIFO_TYPE),
                       .EXT_FIFO(EXT_FIFO),
                       .ENA_TOBUS_FIFO(ENA_TOBUS_FIFO),.ENA_FROMBUS_FIFO(ENA_FROMBUS_FIFO),
                     .ENA_HDR(ENA_HDR),
                     .BLOCK_ID(BLOCK_ID),
                     .PIN_MODEL(PIN_MODEL),
                     .ENA_TIMEC(ENA_TIMEC),.TIMEC_FREQ_ACC(TIMEC_FREQ_ACC),
                     .FULL_DBG_MX(FULL_DBG_MX))
  full_wrap
  (
    .RSTn             (RSTn),           // 通常就是 PRESETn
    .CLK              (PCLK),
    .CLK_SLOW         (CLK_SLOW),
    .slow_gate        (slow_gate),
    .CLK_SLOW_TC      (CLK_SLOW_TC),
    .tc_slow_gate     (tc_slow_gate), // 如果共用时钟，它们将与 slow_gate 相与
    .clk_FastTernary  (1'b0), // 未来：clk_FastTernary),
    .pin_SCL_in       (pin_SCL_in),
    .pin_SCL_out      (pin_SCL_out),
    .pin_SCL_oena     (pin_SCL_oena),
    .pin_SDA_in       (pin_SDA_in),
    .pin_SDA_out      (pin_SDA_out),
    .pin_SDA_oena     (pin_SDA_oena),
    .pin_SDA_oena_rise(pin_SDA_oena_rise),
    .i2c_spike_ok     (i2c_spike_ok),
    .i2c_hs_enabled   (i2c_hs_enabled),
    .cf_SlvEna        (cf_SlvEna),
    .cf_SlvNack       (cf_SlvNack),
     // 静态地址也将映射到下面，因此常量并不真正需要
    .cf_SlvSA         ((ENA_SADDR==`SADDR_CONST) ? {SADDR_P[6:0],1'b1} :
                       (ENA_SADDR==`SADDR_CONFIG)? cf_SlvSA  :
                       (ENA_SADDR==`SADDR_NET)   ? ext_SlvSA :
                       8'h0),           // 来自参数或无
    .cf_IdInst        (cf_IdInst),
    .cf_IdRand        (cf_IdRand),
    .cf_Offline       (cf_Offline),
    .cf_Partno        (cf_Partno),
    .cf_IdBcr         (cf_IdBcr),
    .cf_IdDcr         (cf_IdDcr),
    .cf_IdVid         (cf_IdVid),
    .cf_DdrOK         (cf_DdrOK),
    .cf_TspOK         (cf_TspOK),
    .cf_TslOK         (cf_TslOK),
    .cf_MaxRd         (cf_MaxRd),
    .cf_MaxWr         (cf_MaxWr),
    .cf_RstActTim     (cf_RstActTim),
    .cf_BAMatch       (cf_BAMatch),
    .cf_TCclk         (cf_TCclk),
    .cf_s0ignore      (cf_s0ignore),
    .cf_matchss       (cf_matchss),
    .cf_SetDA         (SetDA),          // 如果 [0]=1，希望覆盖 DA（当未使能时）
    .cf_SetMappedDASA (SetMappedDASA),  // 如果支持映射的 DA/SA
    .cf_SetSA10b      (SetSA10b),       // 如果 [1] 是 SA 10bit
    .cf_MasterAcc     (1'b0),           // 仅用于 M+S：从设备可以接受主设备角色
    .cf_IbiExtData    (cf_IbiExtData),	// IBI 字节后的扩展数据
    .cf_IbiMapIdx     (cf_IbiMapIdx),   // IBI 的映射索引
    .cf_HdrCmd        (cf_HdrCmd),      // HDR Cmd 作为 MMR 的使能
    .cf_CccMask       (cf_CccMask),     // 未处理 CCC 的掩码使能
    .cf_vgpio         (cf_vgpio),       // VGPIO 控制
    .vgpio_done       (vgpio_done),     // 仅当 ENA_MAPPED VGPIO 时使用
    .raw_vgpio_byte   (raw_vgpio_byte),
    .map_daa_use      (map_daa_use),    // 如果 MAP 有自动 DAA 寄存器
    .map_daa_dcr      (map_daa_dcr),    // "
    .map_daa_pid      (map_daa_pid),    // "
    .hdr_new_cmd      (hdr_new_cmd),    // PCLK 域中的脉冲
    .raw_hdr_cmd      (raw_hdr_cmd),    // PCLK 域中的脉冲
    .map_rstdaa       (map_rstdaa),     // 如果 rstdaa 且 Map 自动，则 PCLK 域中的脉冲
    .map_setaasa      (map_setaasa),    // PCLK 域中的脉冲，并为 Map 设置 aasa
    .map_daa_ena      (map_daa_ena),    // 如果 MAP DASA/DAA，则 PCLK 域中的脉冲
    .map_sa_idx       (map_sa_idx),     // 如果 daa_ena，则为 map 中的索引
    .map_daa_da       (map_daa_da),     // 如果 map DASA/DAA，则为新 DA
    .i2c_dev_rev      (cf_i2c_dev_rev), // 如果使用，则来自寄存器
    .i2c_sw_rst       (raw_i2c_slvrst), // 如果使用，则为 i2c 从设备复位
    .outp_to_master   (),               // 从未使用
    .raw_ActState     (raw_ActState),
    .raw_EvState      (raw_EvState),
    .raw_TimeC        (raw_TimeC),
    .raw_timec_sync   (raw_timec_sync), // 如果系统支持 SYNC
    .raw_slvr_reset   (raw_slvr_reset),
    .iraw_rst_slvr_reset(iraw_rst_slvr_reset),
    .iraw_slvr_irq    (iraw_slvr_irq),  // 来自 SCL 域的模块复位
    .raw_Request      (raw_Request),
    .raw_DynAddr      (raw_DynAddr),
    .raw_DynChgCause  (raw_DynChgCause),
    .raw_match_idx    (raw_match_idx),
    .raw_matched      (raw_matched),
    .sraw_ActMode     (reg_ActMode),
    .sraw_PendInt     (reg_PendInt),
    .sraw_StatusRes   (reg_StatusRes),
    .outp_IntStates   (inp_IntStates),
    .reg_EvPend       (reg_EvPend),
    .reg_EvIbiByte    (reg_EvIbiByte),
    .outp_EvNoCancel  (inp_EvNoCancel),
    .outp_EvDet       (inp_EvDet),
    .outp_GenErr      (inp_GenErr),
    .outp_DataErr     (inp_DataErr),
    .msk_GenErr       (msk_GenErr),
    .msk_DataErr      (msk_DataErr),
    .reg_clrIntStates (reg_clrIntStates),
    .reg_clrGenErr    (reg_clrGenErr),
    .reg_clrDataErr   (reg_clrDataErr),
    .reg_holdOErr     (reg_holdOErr),
    .outpflg_MaxRd    (inpflg_MaxRd),
    .outpflg_MaxWr    (inpflg_MaxWr),
    .outp_MaxRW       (inp_MaxRW),
    .reg_TbEnd        (reg_TbEnd),
    .reg_TbFlush      (reg_TbFlush),
    .reg_FbFlush      (reg_FbFlush),
    .reg_TxTrig       (reg_TxTrig),
    .reg_RxTrig       (reg_RxTrig),
    .outp_TxCnt       (inp_TxCnt),
    .outp_RxCnt       (inp_RxCnt),
    .outp_TxFull      (inp_TxFull),
    .outp_RxEmpty     (inp_RxEmpty),
    .regflg_wr_cnt    (regflg_wr_cnt),
    .reg_wdata        (reg_wdata),
    .regflg_rd_cnt    (regflg_rd_cnt),
    .outp_fb_data     (notify_fb_data),
    .ixf_fb_free      (ixf_fb_free),
    .oxf_fb_req       (oxf_fb_req),
    .oxf_fb_data      (oxf_fb_data),
    .oxf_fb_eof       (oxf_fb_eof),
    .ixf_tb_avail     (ixf_tb_avail),
    .ixf_tb_last      (ixf_tb_last),
    .ixf_tb_data      (ixf_tb_data),
    .oxf_tb_start     (oxf_tb_start),
    .oxf_tb_used      (oxf_tb_used),
    // 接下来用于 IBI FIFO（用于 IBI EXTDATA）如果使用
    .ibi_wr_fifo      (ibi_wr_fifo),
    .ibi_wr_ack       (ibi_wr_ack),
    // 以下仅在支持主设备时使用，因此在此置位
    .is_slave         (1'b1),
    .d_tb_data_valid  (),
    .tb_pclk_valid    (),
    .d_tb_datab       (),
    .d_tb_end         (),
    .m_tb_datab_ack   (1'b0),
    .fb_data_use      (),
    .m_fb_datab       (8'd0),
    .m_fb_datab_done  (1'b0),
    // 调试观察器
    .full_debug_observ(full_debug_observ),
    // 最后是扫描/DFT 相关
    .scan_single_clock(scan_single_clock),
    .scan_clk         (scan_clk),
    .scan_no_rst      (scan_no_rst),
    .scan_no_gates    (scan_no_gates)
  );
  assign out_SlvDA    = raw_DynAddr;


  //
  // 寄存器是程序员模型寄存器
  //
  i3c_regs  #(.ENA_ID48B(ENA_ID48B),.ID_48B(ID_48B),
              .ID_AS_REGS(ID_AS_REGS),.ID_BCR(ID_BCR),.ID_DCR(ID_DCR),
              .ENA_SADDR(ENA_SADDR),.SADDR_P(SADDR_P),
              .ENA_MAPPED(ENA_MAPPED),.MAP_CNT(MAP_CNT),.MAP_I2CID(MAP_I2CID),
                .MAP_DA_AUTO(MAP_DA_AUTO), .MAP_DA_DAA(MAP_DA_DAA),
              .ENA_IBI_MR_HJ(ENA_IBI_MR_HJ), .ERROR_HANDLING(ERROR_HANDLING),
                .CLK_SLOW_BITS(CLK_SLOW_BITS),.CLK_SLOW_MATCH(CLK_SLOW_MATCH),
              .ENA_CCC_HANDLING(ENA_CCC_HANDLING), .MAX_RDLEN(MAX_RDLEN),
                .MAX_WRLEN(MAX_WRLEN), .MAX_DS_WR(MAX_DS_WR), .MAX_DS_RD(MAX_DS_RD),
                .MAX_DS_RDTURN(MAX_DS_RDTURN), .RSTACT_CONFIG(RSTACT_CONFIG),
              .SEL_BUS_IF(SEL_BUS_IF),
              .FIFO_TYPE(FIFO_TYPE),
                .EXT_FIFO(EXT_FIFO),
                .ENA_TOBUS_FIFO(ENA_TOBUS_FIFO),
              .ENA_FROMBUS_FIFO(ENA_FROMBUS_FIFO),
              .ENA_HDR(ENA_HDR),
              .ENA_TIMEC(ENA_TIMEC),.TIMEC_FREQ_ACC(TIMEC_FREQ_ACC),
              .BLOCK_ID(BLOCK_ID))
    regs
    (
    .PRESETn          (PRESETn),
    .PCLK             (PCLK),
    .PSEL             (PSEL),
    .PENA             (PENA),
    .PADDR            (PADDR[11:2]),
    .PWRITE           (PWRITE),
    .PRDATA           (PRDATA),
    .PWDATA           (PWDATA),
    .PREADY           (PREADY),
    .wr_err           (wr_err),
    .ign_mwrite       (2'b00),
    .cf_SlvEna        (cf_SlvEna),
    .cf_SlvNack       (cf_SlvNack),
    .cf_SlvSA         (cf_SlvSA),
    .cf_IdInst        (cf_IdInst),
    .cf_IdRand        (cf_IdRand),
    .cf_Offline       (cf_Offline),
    .cf_Partno        (cf_Partno),
    .cf_IdBcr         (cf_IdBcr),
    .cf_IdDcr         (cf_IdDcr),
    .cf_IdVid         (cf_IdVid),
    .cf_DdrOK         (cf_DdrOK),
    .cf_TspOK         (cf_TspOK),
    .cf_TslOK         (cf_TslOK),
    .cf_MaxRd         (cf_MaxRd),
    .cf_MaxWr         (cf_MaxWr),
    .cf_RstActTim     (cf_RstActTim),
    .cf_BAMatch       (cf_BAMatch),
    .cf_TCclk         (cf_TCclk),
    .cf_s0ignore      (cf_s0ignore),
    .cf_matchss       (cf_matchss),
    .cf_IbiExtData    (cf_IbiExtData),  // IBI 后的扩展数据
    .cf_IbiMapIdx     (cf_IbiMapIdx),   // IBI 的映射索引
    .cf_i2c_dev_rev   (cf_i2c_dev_rev), // 如果扩展 i2c 设备 ID
    .cf_HdrCmd        (cf_HdrCmd),      // HDR Cmd 作为 MMR 的使能
    .cf_CccMask       (cf_CccMask),     // 未处理 CCC 的掩码使能
    .cf_vgpio         (cf_vgpio),       // VGPIO 控制
    .map_daa_use      (map_daa_use),    // 如果 MAP 有自动 DAA 寄存器
    .map_daa_dcr      (map_daa_dcr),    // "
    .map_daa_pid      (map_daa_pid),    // "
    .SetDA            (SetDA),          // 如果 [0]=1，希望覆盖 DA（当未使能时）
    .SetMappedDASA    (SetMappedDASA),  // 如果支持映射的 DA/SA
    .SetSA10b         (SetSA10b),       // 如果 [1] 是 SA 10-bit
    .is_slave         (1'b1),           // 总是从设备模式
    .master_comp      (1'b0),           // 因为仅从设备，所以不使用
    .raw_ActState     (raw_ActState),
    .raw_EvState      (raw_EvState),
    .raw_TimeC        (raw_TimeC),
    .raw_Request      (raw_Request),
    .raw_DynAddr      (raw_DynAddr),
    .raw_DynChgCause  (raw_DynChgCause),
    .raw_match_idx    (raw_match_idx),  // 最后3个匹配地址
    .inp_dma_last_tb  (dma_req_tb_ack_last),
    .reg_IntEna       (reg_IntEna),
    .reg_DmaCtrl      (reg_DmaCtrl),
    .inp_IntStates    (inp_IntStates),
    .reg_EvPend       (reg_EvPend),
    .reg_EvIbiByte    (reg_EvIbiByte),
    .inp_EvNoCancel   (inp_EvNoCancel),
    .inp_EvDet        (inp_EvDet),
    .inp_GenErr       (inp_GenErr),
    .inp_DataErr      (inp_DataErr),
    .msk_GenErr       (msk_GenErr),
    .msk_DataErr      (msk_DataErr),
    .inp_err_loc      (inp_IntStates[`IS_ERRWARN]),
    .reg_clrIntStates (reg_clrIntStates),
    .reg_clrGenErr    (reg_clrGenErr),
    .reg_clrDataErr   (reg_clrDataErr),
    .reg_holdOErr     (reg_holdOErr),
    .inpflg_MaxRd     (inpflg_MaxRd),
    .inpflg_MaxWr     (inpflg_MaxWr),
    .inp_MaxRW        (inp_MaxRW),
    .reg_TbEnd        (reg_TbEnd),
    .reg_TbFlush      (reg_TbFlush),
    .reg_FbFlush      (reg_FbFlush),
    .reg_TxTrig       (reg_TxTrig),
    .reg_RxTrig       (reg_RxTrig),
    .inp_TxCnt        (inp_TxCnt),
    .inp_RxCnt        (inp_RxCnt),
    .inp_TxFull       (inp_TxFull),
    .inp_RxEmpty      (inp_RxEmpty),
    .regflg_wr_cnt    (regflg_wr_cnt),
    .reg_wdata        (reg_wdata),
    .regflg_rd_cnt    (regflg_rd_cnt),
    .inp_fb_data      (notify_fb_data), // 可能只使用8或16
    .reg_ActMode      (reg_ActMode),
    .reg_PendInt      (reg_PendInt),
    .reg_StatusRes    (reg_StatusRes),
    .hdr_new_cmd      (hdr_new_cmd),    // PCLK 域中的脉冲
    .raw_hdr_cmd      (raw_hdr_cmd),    // PCLK 域中的脉冲
    .map_rstdaa       (map_rstdaa),     // 如果 rstdaa 且 Map 自动，则 PCLK 域中的脉冲
    .map_setaasa      (map_setaasa),    // 如果设置 aasa，则 PCLK 域中的脉冲
    .map_daa_ena      (map_daa_ena),    // 如果 MAP DASA/DAA，则 PCLK 域中的脉冲
    .map_sa_idx       (map_sa_idx),     // 如果 daa_ena，则为 map 中的索引
    .map_daa_da       (map_daa_da),     // 如果 map DASA/DAA，则为新 DA
    .ibi_wr_fifo      (ibi_wr_fifo),
    .ibi_wr_ack       (ibi_wr_ack),
    .exp_owrite_err   (),               // 这个和下一个用于主设备，因此忽略
    .exp_oread_err    ()
    `ifdef USE_D_RESET
    ,.d_reset_r        (d_reset_r)      // 仅当使用 D 输入复位时
    `endif
  );

endmodule