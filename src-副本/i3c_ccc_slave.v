// i3c_ccc_slave.v
// 模块: i3c_ccc_slave
// 功能: MIPI I3C 从设备 CCC 命令处理（必需和可选，通过参数配置）
//       该模块处理已启用的 CCC 命令。
//       所有从设备都支持 ENTDAA 和 RSTDAA，但这些在 i3c_daa_slave 中处理。
//       该模块至少支持必需的 CCC，并与 i3c_daa_slave 配合支持 ENTDAA、RSTDAA、SETDASA 和 SETNEWDA。
//       此外，还可以处理事件、活动状态、状态请求、最大值等。

`include "i3c_params.v"                 // 本地参数/常量

module i3c_ccc_slave #(
    parameter ENA_CCC_HANDLING= 6'd0,   // 支持哪些 CCC，传递下来
    parameter ID_AS_REGS      = 12'd0,  // 用于 VGPIO
    parameter RSTACT_CONFIG   = 26'd0,  // 从设备复位 RSTACT CCC 配置（见 RSTA_xxx_b 字段）
    parameter MAX_DS_WR = 0,            // M->S 的数据速度限制
    parameter MAX_DS_RD = 0,            // S->M 的数据速度限制
    parameter MAX_DS_RDTURN = 0,        // S->M 读请求的延迟需求
    parameter ENA_HDR   = 0,            // 支持的 HDR 模式
    parameter ENA_MASTER= 0,            // 是否为主设备 - 用于 CCC 支持
    parameter PIN_MODEL = `PINM_COMBO,  // 组合逻辑引脚使用
    parameter ENA_TIMEC = 6'b000010,    // 如果为寄存器，则保留、保留、模式1、模式0、同步
    parameter TIMEC_FREQ_ACC = {8'd24,8'd10}// 频率=12MHz（12.0=24），精度为1.0%
  )
  (
  // 第1部分：时钟、复位和引脚输入
  input               clk_SCL_n,        // SCL 下降沿：启动和发送数据
  input               clk_SCL,          // SCL 上升沿：采样数据和读取 T 位
  input               RSTn,             // 主复位
  input               pin_SDA_in,       // 读取的 SDA 引脚
  // 当前上下文信息
  input         [7:0] state_in_CCC,     // 引擎定义的检测到的 CCC 状态/模式：见 `CF_xxx
  input         [1:0] next_ccc,         // 用于广播无数据 CCC
  input         [7:0] dyn_addr,         // 动态地址匹配，[7:1] 为地址，[0]=1 表示有效
  input               int_start_seen,   // 检测到 START 或重复 START 时的单周期脉冲
  input               int_da_matched,   // 匹配到我们的动态地址时的单周期脉冲（且有效）
  input               int_7e_matched,   // 匹配到 i3c 广播地址时的单周期脉冲
  input               int_in_STOP,      // STOP 状态（保持到 START）
  // 当使用 GET CCC 时的输出数据
  output              ccc_tb_now,       // 输出数据时保持
  output              ccc_tb_data,      // 当前数据位
  output              ccc_tb_continue,  // 1 表示继续，0 表示结束
  input               is_9th,           // 控制第9个 T 位的计数
  // 当指示需要数据时，输入数据字节
  input         [7:0] idata_byte,       // 与 CCC 相关的数据字节
  input               idata_done,       // 数据字节有效时的单周期脉冲
  // 用于 DAA 更新动态地址的控制
  input               daa_active,       // 用于 DAA 的 ID 计数
  output        [6:0] ccc_counter,      // 用于 CCC 和 DAA
  input        [63:0] daa_id,           // 用于 GET CCC
  output              set_da,           // 脉冲触发 DA 改变
  output        [7:1] new_da,           // 要设置的 DA
  output              ccc_handling,     // 是否将处理 CCC
  output              ccc_handled,      // 我们正在处理 CCC
  output              int_ccc_handled,  // 处理中，但仅当直接命令是针对我们时
  output              ccc_uh_mask,      // 未处理 CCC 的掩码
  output              ccc_get,          // 1 表示 GET
  input               ccc_is_read,      // i3c 读取 - 用于 SETGET
  input               ccc_real_START,   // 用于取消 RSTACT
  // 我们可能处理的 CCC 命令的输出和输入
  output        [1:0] opt_state_AS,     // 已知的活动状态
  output        [3:0] opt_ev_mask,      // 已知的事件掩码
    // 读取和写入长度。Cf 是当前状态（和起始状态）
  input        [11:0] cf_MaxRd,
  input        [11:0] cf_MaxWr,
  input        [23:0] cf_RstActTim,     // 从设备复位恢复时间
  input         [8:0] cf_vgpio,         // VGPIO 控制
  input         [6:0] cf_CccMask,       // 未处理 CCC 的掩码使能
  output        [1:0] ccc_vgpio_done,   // 当 VGPIO 是 CCC 时
  output        [7:0] ccc_vgpio,
  output              opt_ChgMaxRd,
  output              opt_ChgMaxWr,
  output       [11:0] opt_MaxRdWr,      // 上述之一改变时的值
  output        [2:0] opt_TimeC,        // 启用的时间控制（如果有）- 独热码
  input               opt_timec_oflow,  // 计数器溢出
  output              opt_timec_oflow_clr, // 清除位
  input        [15:0] cf_TCclk,         // 时间控制时钟信息
  output       [12:0] opt_timec_sync,   // 同步信息（如果 [10:8]!=0）- 在 [11]=1 时改变
  output        [3:0] opt_slvr_reset,   // 保留用于未来 - 目前仅完全/无
  input               i_rst_slvr_reset, // 在 SCL 域中
  input               clr_slvr_cause,   // 清除原因 - 清除复位
  output              opt_setaasa,      // SETAASA 时的脉冲
  input               no_setaasa,       // 如果热加入则禁用
  input               cf_MasterAcc,     // 仅用于 M+S，从设备可以接受主设备角色
  output              opt_MasterAcc,    // 如果主设备被接受，在 SCL 中脉冲为1
  input               bus_err,          // FB 协议错误时的脉冲
  input         [2:0] event_pending,    // IBI 或 P2P 或 HJ 是否挂起
  // safe-raw 表示我们可以直接使用
  input         [7:6] sraw_ActMode,     // 系统活动模式（或0，如果未使用）
  input         [3:0] sraw_PendInt,     // 挂起的中断（或0，如果未使用）
  input        [15:8] sraw_StatusRes,   // 状态保留位（或0，如果未使用）
  input               scan_no_rst       // 防止分层复位
  );

  // 目录
  // 1. 使用宏设置参数为真实值或折叠
  // 2. 简单状态机
  // 3. 确定是否处理的机制
  // 4. 我们支持的 CCC 的处理程序
  // 5. 用于 CCC 和 DAA 模式的主计数器

  reg           [6:0] ccc_handle_r;     // 如果要处理，则设置为值
  reg           [6:0] ccc_init;
  wire          [6:0] load;             // 计数器加载值
  wire                rst_daa_n;
  wire                is_get;
  reg           [6:0] cnt;              // 用于 CCC 和 DAA 的通用计数器
  wire          [6:0] use_cnt;
  reg                 prot_err;         // 保持协议错误直到被读取
  reg           [7:0] def_byte;         // 仅当 v1.1 时使用

  // 定义映射的命令。注意，使用 `define 是因为 generate 有自己的作用域，
  // 因此不能将其用于参数（只能用于线网）。
  // 注意：位[5]为1表示GET，[6]为1表示SETGET
  `define CBA(val) ENA_CCC_HANDLING[`ENCCC_BASIC_b]?val:7'd0
  `define CMX(val) ENA_CCC_HANDLING[`ENCCC_MAXES_b]?val:7'd0
  `define CAA(val) ENA_CCC_HANDLING[`ENCCC_AASA]?val:7'd0
  `define CV1(val) ENA_CCC_HANDLING[`ENCCC_V11MIN]?val:7'd0
  `define CMM(val) |ENA_MASTER?val:7'd0
  `define CMT(val) |ENA_TIMEC?val:7'd0
  localparam CHAN_NONE  =      7'h00;
  localparam CHAN_DAA   =      7'h01;   // 使用引擎中的详细信息
  localparam CHAN_ENEC  = `CBA(7'h02);  // 启用事件
  localparam CHAN_DISEC = `CBA(7'h03);  // 禁用事件
  localparam CHAN_SETAS0= `CBA(7'h04);  // 设置活动状态
  localparam CHAN_SETAS1= `CBA(7'h05);  // 设置活动状态
  localparam CHAN_SETAS2= `CBA(7'h06);  // 设置活动状态
  localparam CHAN_SETAS3= `CBA(7'h07);  // 设置活动状态
  localparam CHAN_SMXWL = `CMX(7'h08);  // 设置最大写入长度
  localparam CHAN_SMXRL = `CMX(7'h09);  // 设置最大读取长度
  localparam CHAN_ENTTM = `CMM(7'h0A);  // 进入测试模式
  localparam CHAN_DEFSLV= `CMM(7'h0B);  // 定义从设备：需要 N 长度模型
  localparam CHAN_BRDGT = `CMM(7'h0C);  // 桥接目标
  localparam CHAN_SXTIM = `CMT(7'h0D);  // 设置时间（时间控制）
  localparam CHAN_RSTACT= `CV1(7'h4E);  // 定义 SlaveRst 规则 - SETGET 类型
  localparam CHAN_SAASA = `CAA(7'h0F);  // SETAASA 从 SA 生成 DA
    // 下面的 GET
  localparam CHAN_STATUS=      7'h20;   // 获取状态
  localparam CHAN_GPID  = `CBA(7'h21);  // 获取 PID
  localparam CHAN_GBCR  = `CBA(7'h22);  // 获取 BCR
  localparam CHAN_GDCR  = `CBA(7'h23);  // 获取 DCR
  localparam CHAN_GHDR  = `CBA(7'h24);  // 获取 HDR 能力
  localparam CHAN_GMXWL = `CMX(7'h25);  // 获取最大写入长度
  localparam CHAN_GMXRL = `CMX(7'h26);  // 获取最大读取长度
  localparam CHAN_GMXDS = `CMX(7'h27);  // 最大数据速度限制
  localparam CHAN_GMST  = `CMM(7'h28);  // 主设备移交
  localparam CHAN_GXTIM = `CMT(7'h29);  // 获取时间控制

  // 负载仅用于直接命令
  localparam LOAD_DAA   = 7'h7F;
  localparam LOAD_ENEC  = `CBA(7'd9);
  localparam LOAD_DISEC = `CBA(7'd9);
  localparam LOAD_STATUS= 7'd16;
  localparam LOAD_GPID  = `CBA(7'd48);
  localparam LOAD_GBCR  = `CBA(7'd8);
  localparam LOAD_GDCR  = `CBA(7'd8);
  localparam LOAD_GHDR  = `CBA(7'd8);
  localparam LOAD_SMXWL = `CMX(7'd16);
  localparam LOAD_SMXRL = `CMX(7'd16);
  localparam LOAD_GMXWL = `CMX(7'd16);
  localparam LOAD_GMXRL = `CMX(7'd16);
  localparam LOAD_GMXDS = |MAX_DS_RDTURN?`CMX(7'd40):`CMX(7'd16);
  localparam LOAD_GMST  = `CMM(7'h08); // 如果不接受则 NACK
  localparam LOAD_GXTIM = `CMT(7'd32); // 4 字节
  localparam LOAD_RSTACT= `CV1(7'd8);


  // 进入 CCC 的流程：
  // 0. int_start_seen
  // 1. int_7E 或 int_da
  // 2. state_in_CCC[`CF_BCAST] 或 [`CF_DIRECT] == 1
  //    -- 与 idata_done 同一周期
  //    -- state_in_CCC[`CF_GRP_b] 可能非0，如果是我们必须支持的 CCC，如 SETNEWDA 或 SETDASA
  // 3. 如果是广播，则使用 idata_done 跟随数据
  //    -- 以 start_seen 结束
  // 4. 如果是直接命令，则等待 start_seen
  // 5. 如果是正常 CCC 直接命令，则为 int_da
  //    -- SETDASA 是特殊的，因此如果匹配我们的静态地址或01，引擎会改变 state_in_CCC[`CF_GRP_b]
  // 6. 现在使用 idata_done 直到 start_seen。

  localparam CST_NO_CCC     = 3'd0;
  localparam CST_BCAST_CCC  = 3'd1;     // 如果不是我们的，等待 start_seen
  localparam CST_DIRECT_CCC = 3'd2;     // 如果不是我们的，等待 int_7e 或 START 参数
  localparam CST_NEWDA      = 3'd3;
  localparam CST_DASA       = 3'd4;     // 特殊的，等待 state_in_CCC 改变

  reg     [2:0] ccc_state;
  reg     [2:0] ccc_next_state;

  always @ (posedge clk_SCL_n or negedge RSTn)
    if (!RSTn)
      ccc_state <= CST_NO_CCC;
    else if (~|state_in_CCC[`CF_DIRECT:`CF_BCAST])
      ccc_state <= CST_NO_CCC;
    else
      ccc_state <= ccc_next_state;

  always @ ( * )
    case (ccc_state)
    CST_NO_CCC:                         // 不在 CCC 中；等待直到进入
      ccc_next_state =
          state_in_CCC[`CF_BCAST]  ? CST_BCAST_CCC  :
          state_in_CCC[`CF_DIRECT] ? CST_DIRECT_CCC :
                                     CST_NO_CCC;

    CST_BCAST_CCC:                      // 不是已知的 CCC
      ccc_next_state =
          int_start_seen ? CST_NO_CCC : // 广播以 start 结束
                           CST_BCAST_CCC;

    CST_DIRECT_CCC:
      ccc_next_state =
          int_7e_matched ? CST_NO_CCC : // 直接命令以 7E 或 STOP 结束
          ((state_in_CCC[`CF_GRP_b]==`CFG_NEWDA) & int_da_matched) ? CST_NEWDA :
          (state_in_CCC[`CF_GRP_b]> `CFG_DASA) ? CST_DASA : // SA 或 01
          CST_DIRECT_CCC;

    CST_NEWDA:                          // 针对我们的直接 NEWDA
      // 等待 data_done 获取新的 DA
      ccc_next_state =
          idata_done ? CST_DIRECT_CCC :
                       CST_NEWDA;

    CST_DASA:                           // 针对我们的直接 SETDASA
      // 等待 data_done 获取新的 DA
      ccc_next_state =
          idata_done ? CST_DIRECT_CCC : CST_DASA;

    default:
      ccc_next_state = CST_NO_CCC;

    endcase

  // 确定我们是否支持该 CCC。
  wire new_ccc = (ccc_state==CST_NO_CCC) & (ccc_next_state==CST_NO_CCC);
  always @ (posedge clk_SCL_n or negedge RSTn)
    if (!RSTn)
      ccc_handle_r   <= CHAN_NONE;
    else if (new_ccc) begin
      if (|next_ccc) begin
        ccc_handle_r <= ccc_init;
      end else
        ccc_handle_r <= CHAN_NONE;
    end
  assign is_get          = (ccc_handle_r[6]&ccc_is_read) | // SETGET
                           ccc_handle_r[5]; // GET 与 SET
  assign ccc_get         = is_get;

  // 注意：case 可能误导，因为我们将任何未实际支持的映射到 CHAN_NONE，因此它们会折叠
  always @ ( * )
    casez (idata_byte)                   // 仅在第一个字节有意义
    8'h06,8'h07,8'h86,
     8'h87,8'h88:      ccc_init = CHAN_DAA;
    8'h90:             ccc_init = CHAN_STATUS;
    {1'b?,7'h00}:      ccc_init = CHAN_ENEC;
    {1'b?,7'h01}:      ccc_init = CHAN_DISEC;
    {1'b?,7'h02}:      ccc_init = CHAN_SETAS0;
    {1'b?,7'h03}:      ccc_init = CHAN_SETAS1;
    {1'b?,7'h04}:      ccc_init = CHAN_SETAS2;
    {1'b?,7'h05}:      ccc_init = CHAN_SETAS3;
    {1'b?,7'h09}:      ccc_init = CHAN_SMXWL;
    {1'b?,7'h0A}:      ccc_init = CHAN_SMXRL;
    8'h8B:             ccc_init = CHAN_GMXWL;
    8'h8C:             ccc_init = CHAN_GMXRL;
    8'h94:             ccc_init = CHAN_GMXDS;
    8'h8D:             ccc_init = CHAN_GPID;
    8'h8E:             ccc_init = CHAN_GBCR;
    8'h8F:             ccc_init = CHAN_GDCR;
    8'h95:             ccc_init = CHAN_GHDR; // 在 v1.1 中是 GETCAPS
    /* 我们在硬件中不处理这些 - 可能将来会处理
    8'h0B:             ccc_init = CHAN_ENTTM;
    8'h08:             ccc_init = CHAN_DEFSLV;
    8'h93:             ccc_init = CHAN_BRDGT;
    */
    8'h28:             ccc_init = CHAN_SXTIM;
    8'h98:             ccc_init = CHAN_SXTIM;
    8'h2A:             ccc_init = CHAN_RSTACT;
    8'h9A:             ccc_init = CHAN_RSTACT;
    8'h29:             ccc_init = ~no_setaasa ? CHAN_SAASA : 5'd0;
    8'h91:             ccc_init = cf_MasterAcc ? CHAN_GMST : 5'd0;
    8'h99:             ccc_init = CHAN_GXTIM;
    default:           ccc_init = CHAN_NONE;
    endcase


  // ccc_mask 用于阻止未处理的 CCC 发送到应用程序
  generate if (ID_AS_REGS[`IDREGS_CCCMSK_b]) begin : ccc_unhandled
  wire base_msk   = cf_CccMask[0] & (idata_byte[6:0]<=8'h1F);
  wire basebx_msk = cf_CccMask[1] & (idata_byte>=8'h20) & (idata_byte<=8'h48);
    wire basedx_msk = cf_CccMask[2] & (idata_byte>=8'hA0) & (idata_byte<=8'hBF);
  wire metb_msk   = cf_CccMask[3] & (idata_byte>=8'h49) & (idata_byte<=8'h64);
  wire metd_msk   = cf_CccMask[4] & (idata_byte>=8'hC0) & (idata_byte<=8'hE3);
  wire vendb_msk  = cf_CccMask[5] & (idata_byte>=8'h65) & (idata_byte<=8'h7F);
    wire vendd_msk  = cf_CccMask[6] & (idata_byte>=8'hE4) & (idata_byte<=8'hFF);
  assign ccc_uh_mask = base_msk | basebx_msk | basedx_msk |
                       metb_msk | metd_msk | vendb_msk | vendd_msk;
  end else begin
    assign ccc_uh_mask = 1'b1;
  end endgenerate
  // load 是最高位 + 1。仅用于直接命令
  assign load = ({7{ccc_handle_r==CHAN_DAA}}    & LOAD_DAA) |
                ({7{ccc_handle_r==CHAN_STATUS}} & LOAD_STATUS) |
                ({7{ccc_handle_r==CHAN_ENEC}}   & LOAD_ENEC) |
                ({7{ccc_handle_r==CHAN_DISEC}}  & LOAD_DISEC) |
                ({7{ccc_handle_r==CHAN_SMXWL}}  & LOAD_SMXWL) |
                ({7{ccc_handle_r==CHAN_SMXRL}}  & LOAD_SMXRL) |
                ({7{ccc_handle_r==CHAN_GMXWL}}  & LOAD_GMXWL) |
                ({7{ccc_handle_r==CHAN_GMXRL}}  & LOAD_GMXRL) |
                ({7{ccc_handle_r==CHAN_GMXDS}}  & LOAD_GMXDS) |
                ({7{ccc_handle_r==CHAN_GPID}}   & LOAD_GPID) |
                ({7{ccc_handle_r==CHAN_GBCR}}   & LOAD_GBCR) |
                ({7{ccc_handle_r==CHAN_GDCR}}   & LOAD_GDCR) |
                ({7{ccc_handle_r==CHAN_GHDR}}   & LOAD_GHDR) |
                ({7{ccc_handle_r==CHAN_GMST}}   & LOAD_GMST) |
                ({7{ccc_handle_r==CHAN_GXTIM}}  & LOAD_GXTIM)|
                ({7{ccc_handle_r==CHAN_RSTACT}} & LOAD_RSTACT);


  // ccc_handling 首先是组合逻辑，因为我们需要用它来阻止为应用程序存储字节。然后作为 ccc_handled 寄存
  assign ccc_handling    = ((ccc_state==CST_NO_CCC) & (ccc_next_state==CST_NO_CCC)) ?
                            (|next_ccc & |ccc_init) : |ccc_handle_r;
  assign ccc_handled     = |ccc_handle_r;
  assign int_ccc_handled = ccc_handled &
                           (state_in_CCC[`CF_BCAST] | int_da_matched |
                            (ccc_state==CST_DASA));

  // 仅当 T 位奇偶校验匹配（idata_done）时设置 DA。注意，这里未使用 bit[0]
  assign set_da = ((ccc_state == CST_NEWDA) | (ccc_state == CST_DASA)) &
                  idata_done;
  assign new_da = idata_byte[7:1];

  // 下一个寄存器为 GET_STATUS 记录协议错误
  always @ (posedge clk_SCL_n or negedge RSTn)
    if (!RSTn)
      prot_err <= 1'b0;
    else if (bus_err)
      prot_err <= 1'b1;                 // 记住协议错误
    else if ((ccc_handle_r==CHAN_STATUS) & ccc_tb_now & (use_cnt[3:0]==4'd1))
      prot_err <= 1'b0;                 // 读取时清除

  // 我们转发一些可选的线网
  wire [15:0] max_wrlen;
  wire [15:0] max_rdlen;
  wire [LOAD_GMXDS:0] max_ds;
  wire [31:0] xtime;
  wire  [7:0] get_slvr_value;
  // 现在是与 DA 无关的内置 GET
  wire [15:0] status     = {sraw_StatusRes[14:8],sraw_ActMode[7:6],
                            prot_err,1'b0,
                            |sraw_PendInt ? sraw_PendInt[3:0] :
                            (event_pending[1:0]==2'd1) ? 4'd1 : 4'd0,
                            sraw_StatusRes[15]}; // 用于回绕索引

  generate if (PIN_MODEL == `PINM_COMBO) begin : combo_for_get
    // 组合 SDA_oe/out，因此我们使用所有寄存状态。
    assign use_cnt = cnt;
  end else begin : reg_for_get
    // 寄存 SDA_oe/out，因此我们必须为 GET 使用组合预计数，因为它与输出在同一周期寄存。
    assign use_cnt = (~daa_active & (ccc_handle_r!=CHAN_DAA) & |ccc_handle_r) ?
                              ((int_da_matched & (ccc_state == CST_DIRECT_CCC)) ?
                               load : (|cnt & ~is_9th) ? (cnt-7'd1) : cnt) :
                             cnt;
  end endgenerate

  assign ccc_tb_now = (is_get & |use_cnt);
  wire        get_status = (ccc_handle_r==CHAN_STATUS) & status[use_cnt[3:0]];
  wire [16:0] dcr_bcr    = {daa_id[15:0],1'b0};
  wire [48:0] prov_id    = {daa_id[63:16],1'b0};
  wire        get_bcr    = (ccc_handle_r==CHAN_GBCR) & dcr_bcr[8+use_cnt[3:0]];
  wire        get_dcr    = (ccc_handle_r==CHAN_GDCR) & dcr_bcr[use_cnt[3:0]];
  wire        get_pid    = (ccc_handle_r==CHAN_GPID) & prov_id[use_cnt[5:0]]; // 临时 ID 部分
  wire        get_caps;  // 在 v1.0 中是 GET_HDR
  generate if (ENA_CCC_HANDLING[`ENCCC_V11MIN]) begin : def_get_caps
    // TODO: 需要添加定义字节选项
    wire [7:0]cap1       = {7'd0, dcr_bcr[8+5+1] & ENA_HDR[`HDR_DDR_b]}; // HDR - 如果启用则为 DDR
                          // 流控制 组 版本
    wire [7:0]cap2       = {1'b0, 1'b0, 2'b00, 4'd1}; // v1.1
                         // DB 以下是状态和 getcaps 的定义字节支持
                         //pnd_rd BT crc32 DB stat DB caps DTDT ibi supp  ML
    wire [7:0]cap3       = {1'b0, 1'b0,    1'b0,   1'b0,   1'b0,    1'b0, 1'b0};
    wire [7:0]cap4       = {1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0};
    wire[32:0]caps       = {cap4, cap3, cap2, cap1, 1'b0};
    assign    get_caps   = (ccc_handle_r==CHAN_GHDR) & caps[use_cnt[5:0]];
  end else begin
    wire      get_hdr    = (use_cnt[3:0]==4'd1) & dcr_bcr[8+5+1] &
                              ENA_HDR[`HDR_DDR_b]; // 目前仅 DDR
    assign    get_caps   = (ccc_handle_r==CHAN_GHDR) & get_hdr;
  end endgenerate
  wire        get_mxwrlen= (ccc_handle_r==CHAN_GMXWL)& max_wrlen[use_cnt[3:0]];
  wire        get_mxrdlen= (ccc_handle_r==CHAN_GMXRL)& max_rdlen[use_cnt[3:0]];
  wire        get_mxds   = (ccc_handle_r==CHAN_GMXDS)& max_ds[use_cnt[(|MAX_DS_RDTURN?5:3):0]];
  wire [32:0] xtime_z    = {xtime,1'b0};
  wire        get_xtime  = (ccc_handle_r==CHAN_GXTIM)& xtime_z[use_cnt[5:0]];
  wire  [8:0] mst_acc    = {dyn_addr[7:1],~^dyn_addr[7:1],1'b0};
  wire        get_mstacc = (ccc_handle_r==CHAN_GMST) & |ENA_MASTER & mst_acc[use_cnt[3:0]];
  wire  [8:0] slvr_val   = {get_slvr_value,1'b0};
  wire        get_rstact = (ccc_handle_r==CHAN_RSTACT) & slvr_val[use_cnt[3:0]];
  assign ccc_tb_data     = ccc_tb_now &
                             (get_status | get_bcr | get_dcr | get_pid | get_xtime |
                              get_caps | get_mxwrlen | get_mxrdlen | get_mxds |
                              get_mstacc | get_rstact);
  assign ccc_tb_continue = |use_cnt[5:1];

  // 现在基本类中的可选部分
  generate
    if (ENA_CCC_HANDLING[`ENCCC_BASIC_b]) begin : basic_ccc
      reg        [1:0] act_state;
      reg        [3:0] event_mask;
      assign opt_state_AS = act_state;
      assign opt_ev_mask  = event_mask;

      // 活动状态设置是广播和直接的；
      // 我们从 CCC 的最低2位剥离出 AS
      // 注意：直接版本不同，因为没有数据
      always @ (posedge clk_SCL_n or negedge RSTn)
        if (!RSTn)
          act_state   <= 2'd0;
        else if (next_ccc[`CF_BCAST-1] & (ccc_init[6:2] == (CHAN_SETAS0>>2)) &
                 idata_done)
          act_state   <= ccc_init[1:0];
        else if ((ccc_handle_r[6:2] == (CHAN_SETAS0>>2)) & int_da_matched)
          act_state   <= ccc_handle_r[1:0];

      // 事件掩码设置或清除是广播和直接的；
      // 我们获得一个字节，其中包含设置或清除的位
      // 注意：直接携带字节以启用或禁用
      always @ (posedge clk_SCL_n or negedge RSTn)
        if (!RSTn)
          event_mask   <= 4'b1011;      // 默认启用
        else if (idata_done & ((ccc_state==CST_BCAST_CCC) | (cnt==7'h1))) begin
          if (ccc_handle_r == CHAN_ENEC)
            event_mask <= event_mask | idata_byte[3:0];
          else if (ccc_handle_r == CHAN_DISEC)
            event_mask <= event_mask & ~idata_byte[3:0];
        end
    end else begin
      assign opt_state_AS = 2'd0;
      assign opt_ev_mask  = 4'b1011;
    end
  endgenerate

  generate
    if (|ENA_TIMEC) begin : xtime_ccc
    //FREE_VERSION_CUT - 从免费版本中移除时间控制
    end else
    begin
      assign xtime     = 32'd0;
      assign opt_TimeC = 3'b000;
      assign opt_timec_oflow_clr = 1'b0;
      assign opt_timec_sync = 12'd0;
    end
  endgenerate

  generate
    if (RSTACT_CONFIG[`RSTA_ENA_b]) begin : rstact_supp
      reg [2:0] slvr_rstact;
      reg [3:0] def_byte_sr;
      localparam CUS_RST = (8'h40|RSTACT_CONFIG[`RSTA_VAL_CUS_b]); // 自定义

      wire [2:0] idata_map = ({3{idata_byte[6:0]==7'h00}}   & `RACT_NONE) |
                             ({3{idata_byte[6:0]==7'h02}}   & `RACT_FULL) |
                             ({3{idata_byte[6:0]==CUS_RST}} & `RACT_CUST &
                              {3{RSTACT_CONFIG[`RSTA_CUS_b]}}); // 自定义使能且匹配
      // 注意：当为直接命令时，这是一个定义字节，因此比正常方案更复杂；
      // idata_done 将在 CCCC 之后的字节发出信号，因此我们必须理解它。
      always @ (posedge clk_SCL_n or negedge RSTn)
        if (!RSTn) begin
          slvr_rstact     <= `RACT_DEF;
          def_byte_sr     <= 3'd0;
        end else if (i_rst_slvr_reset | ccc_real_START)
          slvr_rstact     <= `RACT_DEF;  // 回到默认
        else if ((ccc_handle_r==CHAN_RSTACT) & ~ccc_is_read & ~new_ccc)
          if (idata_done & (ccc_state==CST_BCAST_CCC)) // 广播
            slvr_rstact   <= idata_map;  // 直接赋值
          else if (ccc_state==CST_DIRECT_CCC) begin
            if (idata_done & ~int_da_matched)
              def_byte_sr <= {idata_byte[7], idata_map};
            else if (~def_byte_sr[3] & int_da_matched & ~is_get)
              slvr_rstact <= def_byte_sr[2:0]; // 如果不是 GET，则拾取请求
          end

      // 清除复位是 RSTACT 或 GETSTATUS；RSTACT 可以是 GET 或 SET
      wire clr_reset = (idata_done | (ccc_tb_now & is_9th)) &
                        ((ccc_handle_r==CHAN_RSTACT) | (ccc_handle_r==CHAN_STATUS));
      // assign opt_slvr_reset = {clr_reset|clr_slvr_cause, slvr_rstact}; -- 会在外设复位时阻止升级
      assign opt_slvr_reset = {clr_reset, slvr_rstact};
      wire [7:0] map_slvr = ({8{slvr_rstact==`RACT_DEF}}  & 8'h01) |
                            ({8{slvr_rstact==`RACT_FULL}} & 8'h02) |
                            ({8{slvr_rstact==`RACT_NONE}} & 8'h00) |
                            ({8{slvr_rstact==`RACT_CUST}} & CUS_RST);
      assign get_slvr_value = ~def_byte_sr[3] ? map_slvr :
                              (def_byte_sr[2:0]==`RACT_DEF)  ? cf_RstActTim[7:0] :
                              (def_byte_sr[2:0]==`RACT_FULL) ? cf_RstActTim[15:8] :
                              (def_byte_sr[2:0]==`RACT_CUST) ? cf_RstActTim[23:16] :
                                                               8'd0;
    end else begin
      assign opt_slvr_reset = 4'd0;
      assign get_slvr_value = 8'h00;    // 未使用，因此无意义
    end
  endgenerate

  generate
    if (ID_AS_REGS[`IDREGS_VGPIO_b]) begin : vgpio_ccc_supp
      //FREE_VERSION_CUT - 从免费版本中移除 VGPIO
    end else
    begin
      assign ccc_vgpio      = 8'h00;
      assign ccc_vgpio_done = 2'b00;
    end
  endgenerate

  // SETAASA 是独立的，因为在 BASIC 中作为一个加法器
  assign opt_setaasa    = ENA_CCC_HANDLING[`ENCCC_AASA] &
                          (ccc_handle_r == CHAN_SAASA);

  generate
    if (ENA_MASTER) begin : mstacc_ccc
      reg mast_accepted;
      always @ (posedge clk_SCL_n or negedge RSTn)
        if (!RSTn)
          mast_accepted <= 1'b0;
        else if ((ccc_handle_r==CHAN_GMST) & ~|use_cnt[3:0])
          mast_accepted <= 1'b1;        // 在结束时设置
        else if (~cf_MasterAcc)
          mast_accepted <= 1'b0;        // 主设备切换时清除
        else if (int_start_seen)
          mast_accepted <= 1'b0;        // 被 Sr 与 P 终止

      assign opt_MasterAcc = mast_accepted;
    end else begin
      assign opt_MasterAcc = 1'b0;
    end
  endgenerate

  generate
    if (ENA_CCC_HANDLING[`ENCCC_MAXES_b]) begin : maxes_ccc
      // 对于 GET，我们构建位遍历器，它们与其他 GET 进行或运算。
      // 对于 SET，我们仅在此处理。
      reg [11:0] max_chg;
      reg        chgw, chgr;
      reg  [1:0] lcnt;

      assign max_wrlen = {3'd0,cf_MaxWr,1'b0}; // 注意回绕地址
      assign max_rdlen = {3'd0,cf_MaxRd,1'b0}; // 注意回绕地址
      if (|MAX_DS_RDTURN) // RDTURN 是 LSB 顺序
        assign max_ds    = {MAX_DS_WR[7:0],MAX_DS_RD[7:0],MAX_DS_RDTURN[7:0],
                            MAX_DS_RDTURN[15:8],MAX_DS_RDTURN[23:16],1'b0};
      else
        assign max_ds    = {MAX_DS_WR[6:0],MAX_DS_RD[7:0],MAX_DS_WR[7]};

      // 设置最大长度是广播和直接的；广播命令我们获得2个字节，直接命令仅针对我们
      always @ (posedge clk_SCL_n or negedge RSTn)
        if (!RSTn) begin
          max_chg     <= 12'd0;
          chgw        <= 1'b0;
          chgr        <= 1'b0;
          lcnt        <= 2'd0;
        end else if ((ccc_handle_r == CHAN_SMXWL) | (ccc_handle_r == CHAN_SMXRL)) begin
          if (idata_done &
              ((ccc_state==CST_BCAST_CCC) | (~|cnt[2:0] & (ccc_state==CST_DIRECT_CCC)))) begin
            if (~|lcnt) begin
              max_chg[11:8] <= idata_byte[3:0];
              lcnt    <= 2'd1;
            end else if (lcnt == 2'd1) begin
              max_chg[7:0]  <= idata_byte;
              if (ccc_handle_r == CHAN_SMXWL)
                chgw  <= 1'b1;
              else
                chgr  <= 1'b1;
              lcnt    <= 2'd2;
            end // 未来：如果我们允许更大的 IBI 字节计数，为读取拾取第3个字节
          end else begin
            chgw      <= 1'b0;
            chgr      <= 1'b0;
          end
        end else begin
          lcnt        <= 2'd0;
          max_chg     <= 12'd0;
          chgw        <= 1'b0;
          chgr        <= 1'b0;
        end
      assign opt_ChgMaxRd = chgr;
      assign opt_ChgMaxWr = chgw;
      assign opt_MaxRdWr  = max_chg;

    end else begin
      assign opt_ChgMaxRd = 1'b0;
      assign opt_ChgMaxWr = 1'b0;
      assign opt_MaxRdWr  = 12'd0;
      assign max_wrlen    = 16'd0;
      assign max_rdlen    = 16'd0;
      assign max_ds       = 25'd0;
    end
  endgenerate


  // 计数器处理 CCC 命令和 DAA 模式 ID 过程
  // 我们不使用时复位，以免引起问题（包括在 STOP 时）
  assign rst_daa_n = RSTn & (|state_in_CCC[`CF_ONLY_b] | scan_no_rst);
  `Observe(observe_daa_rst, clk_SCL, |state_in_CCC) // 可选的 DFT 观察器

  always @ (posedge clk_SCL_n or negedge rst_daa_n)
    if (!rst_daa_n)
      cnt     <= 7'h00;
    else if (~daa_active) begin
      if (ccc_handle_r == CHAN_DAA)
        cnt   <= 7'h7F;                 // 为 DAA 准备
      else if (|ccc_handle_r) begin
        // 我们处理的一个
        if (int_da_matched & (ccc_state == CST_DIRECT_CCC)) begin
          cnt <= load;
        end else if (|cnt & ~is_9th)
          cnt <= cnt - 7'd1;            // 其他用途向下计数
      end
    end else if (|cnt)
      cnt     <= cnt - 7'd1;            // 向下计数
  assign ccc_counter = cnt;


endmodule