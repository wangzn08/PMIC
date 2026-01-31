/*--------------------------------------------------------------------
此模块为 MIPI I3C 和 I2C 从设备处理引擎的核心组件，支持 SDR 模式。
处理 START/STOP、地址匹配、数据串行化/反串行化、奇偶校验等功能。
--------------------------------------------------------------------*/

`include "i3c_params.v"                 // 本地参数/常量定义

module i3c_sdr_slave_engine #(
    // 参数定义
    parameter         ENA_SADDR = `SADDR_NONE,    // 静态地址使能：无、常量、寄存器/网络
    parameter          SADDR_P  = 0,    // 7位静态地址（位7:1）
    parameter         ENA_MAPPED= 5'd0, // 扩展动态/静态地址使能
    parameter          MAP_CNT  = 4'd1, // 扩展地址数量
    parameter          MAP_I2CID= 24'd0,// I2C设备ID（非零时使能）
    parameter          MAP_DA_AUTO= {5'd0,1'b0,1'b0,3'd0,1'b0,1'b0,1'b0},
    parameter          MAP_DA_DAA = 0,  // DAA模式位数组
    parameter         ENA_IBI_MR_HJ = 0,// 事件使能掩码：IBI/MR/HJ
    parameter         ENA_HDR   = 0,    // HDR-DDR模式支持（仅DDR）
    parameter         PIN_MODEL = `PINM_COMBO, // 引脚模型：组合逻辑、寄存器等
    parameter         priv_sz   = PIN_MODEL[1] ? (ENA_HDR[0] ? 3 : 1) : 0,
    parameter         ENG_DBG_MX=0      // 调试观测器
  )
  (
  //==================================================================
  // 1. 时钟、复位与保持信号
  //==================================================================
  input               clk_SCL_n,        // SCL下降沿：起始条件和数据发送
  input               clk_SCL,          // SCL上升沿：数据采样和读取T位
  input               clk_SDA,          // SDA上升沿：STOP检测（当SCL=1时）
  input               RSTn,             // 全局复位
  input               SDR_hold,         // SDR保持（HDR模式时有效）
  input               cf_SdrOK,         // I3C SDR使能（否则仅I2C）

  //==================================================================
  // 2. I3C总线引脚信号
  //==================================================================
  input               pin_SCL_in,       // SCL引脚输入（用于空闲检测和START/STOP）
  input               pin_SDA_in,       // SDA引脚输入
  output              pin_SDA_out,      // SDA输出（用于读操作、ACK和仲裁）
  output              pin_SDA_oena,     // SDA输出使能
  output  [priv_sz:0] pin_SDA_oena_rise,// 特殊上升沿输出使能（EXT_REG模式）
  output              i2c_spike_ok,     // I2C尖峰滤波器使能
  output              i2c_hs_enabled,   // I2C高速模式指示

  //==================================================================
  // 3. 状态指示与中断信号
  //==================================================================
  output              int_start_seen,   // START/重复START检测脉冲
  output              int_start_err,    // STOP后SDA=1错误（保持直到START）
  output              int_da_matched,   // 动态地址匹配脉冲
  output              int_sa_matched,   // 静态地址匹配脉冲
  output              int_7e_matched,   // I3C广播地址（0x7E）匹配脉冲
  output              int_in_STOP,      // STOP状态（保持直到START）
  output              int_event_sent,   // 事件发送脉冲
  output              int_event_ack,    // 事件ACK脉冲（与int_event_sent同时）
  output              int_in_evproc,    // 事件处理中标志

  //==================================================================
  // 4. 临时配置
  //==================================================================
  input               cf_SlvNack,       // 强制NACK匹配的地址

  //==================================================================
  // 5. 发送缓冲区（To-Bus）接口
  //==================================================================
  input               tb_data_valid,    // 数据有效标志（1=缓冲区满）
  input         [7:0] tb_datab,         // 待发送数据字节
  input               tb_end,           // 数据结束标志
  output              tb_datab_ack,     // 数据消耗确认脉冲
  output              tb_urun_nack,     // 无数据导致的NACK脉冲
  output              tb_urun,          // 无数据但未结束的脉冲
  output              tb_term,          // 主机终止读操作

  //==================================================================
  // 6. 接收缓冲区（From-Bus）接口
  //==================================================================
  input               fb_data_use,      // 缓冲区使用标志（1=可用）
  output        [7:0] fb_datab,         // 接收到的数据字节
  output              fb_datab_done,    // 数据接收完成脉冲
  output              fb_datab_err,     // I3C奇偶校验错误
  output              fb_orun,          // 缓冲区溢出脉冲
  output              fb_s0s1_err,      // S0/S1锁定错误
  input               rd_abort,         // 读操作中止（时钟停滞>60us）

  //==================================================================
  // 7. 状态机与模式指示
  //==================================================================
  output              state_accslv,     // 从设备读/写活跃状态
  output              state_rd,         // 发送数据状态（读或IBI数据）
  output              state_wr,         // 接收数据状态（SDR写）
  output              in_daa_mode,      // ENTDAA模式状态
  output              state_dir,        // 方向标志（基于头部RnW位）

  //==================================================================
  // 8. CCC相关信号
  //==================================================================
  output        [7:0] state_in_CCC,     // CCC状态/模式（见CF_xxx定义）
  output        [1:0] next_ccc,         // 下一CCC类型（广播/定向）
  output        [1:0] next_hdr,         // 下一HDR模式（[0]=HDR,[1]=DDR）
  output              in_ddr,           // DDR模式激活标志
  input               ccc_handling,     // CCC本地处理标志
  input               ccc_handled,      // CCC已处理标志
  input               int_ccc_handled,  // CCC定向到本设备
  input               ccc_uh_mask,      // 未处理CCC掩码
  input               ccc_get,          // GET类型CCC标志
  output              ccc_is_read,      // CCC读操作标志
  output              ccc_real_START,   // 真实START（用于取消RSTACT）
  output              int_ccc,          // 未处理CCC中断脉冲
  output              done_rdy,         // CCC处理就绪标志（提前一个周期）
  output        [7:0] done_buff,        // 主设备发送的CCC数据
  input               ccc_tb_now,       // CCC数据待发送
  input               ccc_tb_data,      // CCC数据位（同时钟域）
  input               ccc_tb_continue,  // CCC数据继续标志
  output              is_9th,           // 第9位时钟周期标志
  input               opt_s0ignore,     // 忽略S0错误请求

  //==================================================================
  // 9. DAA（动态地址分配）相关
  //==================================================================
  output              daa_active,       // DAA处理活跃标志
  input               daa_act_ok,       // DAA使能（用于映射地址）
  input               daa_inp_drv,      // DAA输入驱动使能
  input               daa_inp_bit,      // DAA输入数据位
  input               daa_done,         // DAA完成脉冲（即使奇偶错误）
  input               daa_acknack,      // ENTDAA新DA的ACK/NACK

  //==================================================================
  // 10. 地址匹配相关
  //==================================================================
  input         [7:0] dyn_addr,         // 动态地址（位[7:1]有效，[0]=1表示已分配）
  input         [7:0] opt_static_addr,  // 静态地址（位[7:1]有效，[0]=1表示启用）
  input [(MAP_CNT*10)-1:0] SetMappedDASA, // 映射地址数组
  input         [2:0] SetSA10b,         // 10位静态地址配置
  output       [12:0] opt_match_idx,    // 最后匹配的地址索引
  input               cf_vgpio,         // VGPIO CCC触发标志
  output              vgpio_done,       // VGPIO完成脉冲
  output        [7:0] vgpio_byte,       // VGPIO字节数据
  input         [1:0] ccc_vgpio_done,   // VGPIO CCC完成信号
  input         [7:0] ccc_vgpio,        // VGPIO CCC数据

  //==================================================================
  // 11. HDR-DDR相关
  //==================================================================
  input               hdr_newcmd,       // HDR新命令脉冲（用于MMR）
  output        [7:0] hdr_cmd,          // HDR命令数据
  output              raw_dasa_ena,     // 自动DASA匹配脉冲
  output        [3:0] map_sa_idx,       // 映射地址索引

  //==================================================================
  // 12. I2C扩展功能
  //==================================================================
  input         [2:0] i2c_dev_rev,      // 设备版本号（用于DeviceID）
  output              i2c_sw_rst,       // I2C软件复位

  //==================================================================
  // 13. 事件处理（IBI/MR/HJ）
  //==================================================================
  input         [2:0] event_pending,    // 待处理事件：MR=3, IBI=2, HJ=1
  input               force_sda,        // 强制SDA输出（当SCL=1时）
  input               ibi_has_byte,     // IBI附带字节标志
  input         [7:0] opt_ibi_byte,     // IBI附加字节数据
  input               ibi_timec,        // IBI时间戳控制标志
  output        [1:0] ibi_timec_marks,  // 时间戳标记脉冲（SC1/SC2）
  output        [2:0] ibi_timec_sel,    // 时间戳选择器
  input               ibi_extdata,      // IBI后切换为读模式标志
  input         [3:0] ibi_mapidx,       // IBI映射地址索引
  output              ibi_in_extdata,   // IBI扩展数据读取状态

  //==================================================================
  // 14. HDR-DDR数据接口
  //==================================================================
  input         [1:0] ddr_fb_ctrl,      // 接收控制：0=收集，1=推送LSB，2=推送MSB
  input               ddr_fb_MSB,       // DDR接收字节1
  input               ddr_fb_LSB,       // DDR接收字节2
  input         [1:0] ddr_tb_ctrl,      // 发送控制：0=关闭，1=用缓冲区，2=用本模块位，3=复制MSB
  input         [1:0] ddr_tb_bits,      // 前导码和奇偶校验位（上升/下降沿）
  input         [1:0] ddr_tb_oe,        // 前导码输出使能
  output        [2:0] obit_cnt,         // 位计数器
  output        [7:0] byte_1st,         // 第一个字节
  output        [7:0] byte_2nd,         // 第二个字节

  //==================================================================
  // 15. 调试与测试
  //==================================================================
  output[ENG_DBG_MX:0]eng_debug_observ, // 调试观测器
  input               scan_no_rst       // 扫描测试：禁止分层复位
  );

  //==================================================================
  // 内部信号定义
  //==================================================================
  reg                 STOP_r;           // STOP状态寄存器
  wire                rst_STOP_n;       // STOP复位（低有效）
  wire                any_START_d;      // START/重复START检测
  wire                real_START_d;     // 真实START检测（STOP后）
  reg                 init_stop;        // 初始STOP状态

  reg                 SDA_r;            // SDA采样寄存器（SCL上升沿）
  reg                 start_possible;   // SCL下降沿时SDA为低
  reg                 start_any;        // START/重复START
  reg                 start_invalid;    // STOP后无效START检测
  reg                 start_r;          // 真实START（非重复）

  wire                is_dasa;          // SETDASA CCC模式
  wire                addr_match;       // 地址匹配标志
  wire                map_da_match, map_sa_match; // 映射地址匹配
  wire                in_sa2byt;        // 10位地址匹配状态
  wire                matched_7e;       // 广播地址0x7E匹配
  wire                s0_error, s0_error2; // S0错误检测
  wire                matched_p2p;      // 点对点地址0x01匹配

  reg                 was_read;         // 方向寄存器（基于RnW位）
  reg      [2:0]      bit_cnt;          // 位计数器（0-7）
  reg      [7:0]      fb_datab_r;       // 接收数据寄存器
  reg      [7:0]      ccc_byte;         // CCC数据字节寄存器
  wire                use_ccc_byte;     // CCC字节使用标志
  reg                 use_ccc_r;        // CCC字节使用寄存器
  wire    [7:0]       work_datab;       // 当前工作数据
  reg                 datab_done;       // 数据接收完成脉冲
  reg                 datab_orun;       // 数据溢出标志
  wire                parity;           // 奇偶校验位
  wire                rdata_bit;        // 发送数据位
  wire    [1:0]       rddr_bits;        // DDR发送位

  reg                 daa_lost;         // DAA仲裁失败标志
  reg                 event_won;        // 事件仲裁获胜标志
  wire                event_losing;     // 事件仲裁失败中
  wire                event_drive;      // 事件头部驱动标志
  wire    [7:0]       event_hdr;        // 事件头部地址+RnW
  wire                event_hdr_b;      // 事件头部当前位
  wire                force_START;      // IBI强制START

  reg      [1:0]      timec_pos;        // 时间戳位置索引
  reg                 ibi_rd;           // IBI扩展数据读取标志
  wire                ack_ok;           // ACK允许标志
  wire                map_spc;          // 映射特殊ACK
  wire                vgpio_spc;        // VGPIO特殊ACK
  wire                vgpio_msg;        // VGPIO消息匹配
  wire                vgpio_ccc;        // VGPIO CCC处理
  wire                dev_spc;          // I2C设备ID特殊ACK
  wire                spc_data_valid;   // 特殊数据有效（I2C DevID）
  wire                spc_data_end;     // 特殊数据结束
  wire    [7:0]       spc_data;         // 特殊数据（I2C DevID）

  reg      [7:0]      in_ccc;           // CCC状态跟踪寄存器
  reg      [1:0]      ccc_supp;         // CCC支持标志
  reg                 ccc_uh_r;         // 未处理CCC寄存器
  wire                ccc_valid_dir;    // 定向CCC方向有效
  wire                next_int_ccc, next_bc_ccc, next_dc_ccc;
  reg                 pass_ccc;         // CCC传递标志

  wire                is_i3c = dyn_addr[0] & cf_SdrOK; // I3C协议标志
  wire                i3c_protocol = cf_SdrOK & (is_i3c | (|in_ccc));
  reg                 in_i3c_msg;       // I3C消息内标志

  wire                pend_IBI = event_pending[1:0]==2'h1; // IBI待处理
  wire                pend_MR  = event_pending[1:0]==2'h2; // MR待处理
  wire                pend_HJ  = event_pending[1:0]==2'h3; // HJ待处理
  wire                i3c_end  = (ccc_handled ? ~ccc_tb_continue : (tb_end|~tb_data_valid));

  reg                 end_flag;         // 发送结束标记
  reg                 is_term;          // 主机终止标志
  wire                will_term;        // 即将终止标志
  wire                i2c_nack;         // I2C NACK检测
  wire                end_read;         // 读结束标志
  wire                ibi_extd;         // IBI扩展数据有效

  wire                SDA_oe, SDA_out;  // SDA输出控制
  wire                r_SDA_oena;       // 寄存器模式输出使能

  //==================================================================
  // 状态机定义
  //==================================================================
  reg      [3:0]      state_base;       // 基础状态
  wire    [3:0]       state;            // 当前状态
  reg      [3:0]      next_state;       // 下一状态

  // 状态编码
  localparam ST_WAIT_SrP      = 4'b0000; // 等待START/STOP
  localparam ST_WRITE         = 4'b0001; // 主设备写
  localparam ST_A7_A0_RnW     = 4'b0011; // 地址头部（包括RnW）
  localparam ST_IBI_BYTE      = 4'b0100; // IBI数据字节
  localparam ST_IBI9TH        = 4'b0101; // IBI第9位
  localparam ST_MACK          = 4'b0110; // 主设备ACK/NACK（对事件）
  localparam ST_ACK_NACK      = 4'b1000; // 本设备ACK/NACK地址
  localparam ST_READ          = 4'b1001; // 从设备读
  localparam ST_EV_A7_A0_RnW  = 4'b1011; // 事件+地址头部
  localparam ST_R9TH          = 4'b1100; // 读第9位（I2C ACK，I3C T位）
  localparam ST_W9TH          = 4'b1101; // 写第9位（I2C ACK，I3C奇偶校验）
  localparam ST_DAA           = 4'b1110; // ENTDAA模式

  //==================================================================
  // 调试观测器生成
  //==================================================================
  `ifdef ENA_DBG_OBSERVE
   `include "dbg_observ_eng.v"   // 集成时替换为实际文件
  `else
   assign eng_debug_observ = {1'b0}; // 默认无观测器输出
  `endif

  //==================================================================
  // 1. STOP检测逻辑（SDA上升沿时钟域）
  //==================================================================
  assign rst_STOP_n  = RSTn & (~any_START_d | pin_SCL_in | scan_no_rst); 
  `Observe(observe_STOP_rst, clk_SCL, ~any_START_d ^ pin_SCL_in ^ ~STOP_r  ^ ~rd_abort)

  always @ (posedge clk_SDA or negedge rst_STOP_n)
    if (!rst_STOP_n)
      STOP_r      <= 1'b0;
    else if (pin_SCL_in & ~SDR_hold)   // SCL高时SDA上升
      STOP_r      <= 1'b1;
  assign int_in_STOP = STOP_r | init_stop;

  //==================================================================
  // 2. SDA采样寄存器（SCL上升沿时钟域）
  //==================================================================
  always @ (posedge clk_SCL or negedge RSTn)
    if (!RSTn)
      SDA_r <= 1'b0;
    else if ((~SDR_hold|next_hdr[0]) | ddr_fb_MSB | ddr_fb_LSB)
      SDA_r <= pin_SDA_in;

  always @ (posedge clk_SCL or negedge RSTn)
    if (!RSTn)
      init_stop <= 1'b1;
    else if (~SDR_hold)
      init_stop <= 1'b0;

  //==================================================================
  // 3. START检测逻辑（SCL下降沿时钟域）
  //==================================================================
  always @ (posedge clk_SCL_n or negedge RSTn)  
    if (!RSTn) begin
      start_possible <= 1'b0;
      start_any      <= 1'b0;
      start_invalid  <= 1'b0;
      start_r        <= 1'b0;
    end else if (~SDR_hold) begin
      start_possible <= ~pin_SDA_in;
      start_any      <= (int_in_STOP | SDA_r);
      start_invalid  <= pin_SDA_in & int_in_STOP;
      if (int_in_STOP)
        start_r      <= 1'b1;
      else if (~pin_SDA_in & SDA_r)
        start_r      <= 1'b0;
    end 
  assign any_START_d    = start_possible & start_any;
  assign real_START_d   = any_START_d & start_r;
  assign ccc_real_START = real_START_d;
  assign int_start_seen = any_START_d;
  assign int_start_err  = start_invalid;

  //==================================================================
  // 4. 主状态机
  //==================================================================
  wire state_rst_n = RSTn & (scan_no_rst | ~rd_abort);
  always @ (posedge clk_SCL_n or negedge state_rst_n) 
    if (!state_rst_n)
      state_base <= ST_WAIT_SrP;
    else if (SDR_hold)
      state_base <= ST_WAIT_SrP;
    else
      state_base <= next_state;

  assign state        = any_START_d ? (event_drive ? ST_EV_A7_A0_RnW : ST_A7_A0_RnW) : 
                                      STOP_r ? ST_WAIT_SrP : state_base;
  assign state_accslv = |state & ~int_in_STOP;
  assign state_rd     = (^state[3:2] | (state==ST_R9TH)) & |state[2:0] & ~int_in_STOP;
  assign state_wr     = ((state==ST_WRITE)|(state==ST_W9TH)) & ~int_in_STOP;
  assign is_9th       = state == ST_R9TH;
  assign state_dir    = was_read;

  // 状态转换逻辑
  always @ ( * ) 
    case (state)
      ST_A7_A0_RnW, ST_EV_A7_A0_RnW:
        next_state = |bit_cnt ? ((event_won&~event_losing) ? ST_EV_A7_A0_RnW : ST_A7_A0_RnW) : 
                     (event_won&~event_losing) ? ST_MACK : addr_match ? ST_ACK_NACK : ST_WAIT_SrP;

      ST_ACK_NACK:
        next_state = (in_daa_mode & was_read) ? ST_DAA : ~ack_ok ? ST_WAIT_SrP :
                     was_read ? ((tb_data_valid | dev_spc | ccc_handled) ? ST_READ : ST_WAIT_SrP) : ST_WRITE; 

      ST_READ:
        next_state = (is_term | rd_abort) ? ST_WAIT_SrP : |bit_cnt ? ST_READ : ST_R9TH;

      ST_R9TH:
        next_state = (end_read | i2c_nack | rd_abort) ? ST_WAIT_SrP : ST_READ;
   
      ST_WRITE:
        next_state = |bit_cnt ? ST_WRITE : ST_W9TH;

      ST_W9TH:
        next_state = (in_i3c_msg&(parity!=SDA_r)) ? ST_WAIT_SrP : ST_WRITE;

      ST_DAA:
        next_state = (daa_lost | daa_done) ? ST_WAIT_SrP : ST_DAA;

      ST_MACK:
        next_state = SDA_r ? ST_WAIT_SrP :
                     (pend_IBI & ibi_has_byte) ? ST_IBI_BYTE : ST_WAIT_SrP;
                   
      ST_IBI_BYTE:
        next_state = |bit_cnt ? ST_IBI_BYTE : ST_IBI9TH;

      ST_IBI9TH:
        next_state = ~|timec_pos ? (ibi_extd ? ST_READ : ST_WAIT_SrP) : ST_IBI_BYTE;
 
      ST_WAIT_SrP:
        next_state = ST_WAIT_SrP;

      default:
        next_state = ST_WAIT_SrP;
    endcase

  //==================================================================
  // 5. 方向与位计数器
  //==================================================================
  always @ (posedge clk_SCL_n or negedge RSTn) 
    if (!RSTn)
      was_read <= 1'b0;
    else if ((state[2:0] == ST_A7_A0_RnW[2:0]) & (bit_cnt == 3'h0))
      was_read <= SDA_r;

  always @ (posedge clk_SCL_n or negedge RSTn) 
    if (!RSTn)
      bit_cnt   <= 3'h7;
    else if (~SDR_hold) begin
      if (~pin_SDA_in & (int_in_STOP | SDA_r))
        bit_cnt <= 3'h7;
      else if ((state[2:0] == ST_A7_A0_RnW[2:0]) | (state == ST_WRITE) | 
               (state == ST_IBI_BYTE) | (state == ST_READ) |
               ((PIN_MODEL!=`PINM_COMBO) & (state == ST_ACK_NACK)))
        if ((state != next_state) & (next_state != ST_A7_A0_RnW))
          bit_cnt <= 3'h7;
        else
          bit_cnt <= bit_cnt - 3'h1;
    end else if (ENA_HDR[`HDR_DDR_b]) begin
      if (ddr_fb_MSB | ddr_fb_LSB)
        bit_cnt <= bit_cnt - 3'h1;
      else if (ddr_tb_ctrl[0])
        bit_cnt <= bit_cnt - 3'h1;
      else
        bit_cnt <= 3'h7;
    end

  //==================================================================
  // 6. 发送数据路径（To-Bus）
  //==================================================================
  assign rdata_bit    = ccc_handled ? ccc_tb_data :
                        spc_data_valid? spc_data[bit_cnt] :
                        tb_data_valid ? tb_datab[bit_cnt] : 1'b1;

  generate if (ENA_HDR[`HDR_DDR_b]) 
    assign rddr_bits  = ddr_tb_bits[1:0];
  else
    assign rddr_bits  = 2'b00;
  endgenerate

  assign tb_datab_ack = ((state==ST_R9TH)| &ddr_tb_ctrl) & 
                        (~spc_data_valid & tb_data_valid) & ~ccc_handled;
  wire   read_urun    = (state==ST_READ) & ~tb_data_valid & ~spc_data_valid & ~ccc_handled & ~is_term;
  assign tb_urun      = read_urun & ~ibi_rd;
  assign tb_urun_nack = (state==ST_ACK_NACK) & was_read & ~tb_data_valid & ~dev_spc &
                        ~ccc_handled & ~matched_7e;

  always @ (posedge clk_SCL_n or negedge RSTn) 
    if (!RSTn) 
      is_term <= 1'b0;
    else if ((state==ST_R9TH) & ~end_read & ~spc_data_valid)
      is_term <= will_term;
    else if (is_term)
      is_term <= 1'b0;
  assign tb_term      = is_term;
  assign i2c_nack     = ~in_i3c_msg & SDA_r;
  assign will_term    = in_i3c_msg ? ~pin_SDA_in : SDA_r;
  assign end_read     = ccc_handled ? ~ccc_tb_continue : end_flag;

  always @ (posedge clk_SCL_n or negedge RSTn) 
    if (!RSTn) 
      end_flag <= 1'b1;
    else if (tb_datab_ack | (state==ST_ACK_NACK) | (next_state==ST_R9TH))
      end_flag <= spc_data_valid ? spc_data_end : (tb_end | ~tb_data_valid); 

  //==================================================================
  // 7. 接收数据路径（From-Bus）
  //==================================================================
  wire not_app_ccc = (ccc_handled | ~in_ccc[`CF_DIRECT]);
  always @ (posedge clk_SCL_n or negedge RSTn) 
    if (!RSTn) 
      use_ccc_r   <= 1'b1;
    else if (~SDR_hold)
      if ((state[2:0]==ST_A7_A0_RnW[2:0]) & not_app_ccc)
        use_ccc_r <= 1'b1;
      else if ((state==ST_ACK_NACK) & ~ccc_handled & ~matched_7e)
        use_ccc_r <= 1'b0;
      else if ((state==ST_ACK_NACK) & ~ccc_handled & matched_7e)
        use_ccc_r <= 1'b1;
      else if (((state==ST_W9TH) | (state==ST_R9TH)) & ~ccc_handled)
        use_ccc_r <= 1'b0;
  assign use_ccc_byte = use_ccc_r | real_START_d | (any_START_d & not_app_ccc);

  always @ (posedge clk_SCL_n or negedge RSTn) 
    if (!RSTn) 
      fb_datab_r            <= 8'd0;
    else if (next_bc_ccc | next_dc_ccc) begin
      fb_datab_r[7:0]       <= ccc_byte;
    end else if (~SDR_hold & ~use_ccc_byte) begin
      if (((state[2:0] == ST_A7_A0_RnW[2:0]) | (state == ST_WRITE)))
        fb_datab_r[bit_cnt] <= SDA_r;
    end else if (ddr_fb_LSB & ENA_HDR[`HDR_DDR_b]) begin
      fb_datab_r[{bit_cnt[1:0],1'b1}] <= SDA_r;
      fb_datab_r[{bit_cnt[1:0],1'b0}] <= pin_SDA_in;
    end else if ((ddr_fb_ctrl==2'b01) & ENA_HDR[`HDR_DDR_b])
      fb_datab_r[7:0]       <= ccc_byte;
  assign hdr_cmd = fb_datab;

  always @ (posedge clk_SCL_n or negedge RSTn) 
    if (!RSTn) 
      ccc_byte            <= 8'd0;
    else if (~SDR_hold & use_ccc_byte) begin
      if (((state[2:0] == ST_A7_A0_RnW[2:0]) | (state == ST_WRITE)))
        ccc_byte[bit_cnt] <= SDA_r;
    end else if (ddr_fb_MSB & ENA_HDR[`HDR_DDR_b]) begin
      ccc_byte[{bit_cnt[1:0],1'b1}] <= SDA_r;
      ccc_byte[{bit_cnt[1:0],1'b0}] <= pin_SDA_in;
    end
  assign work_datab= use_ccc_byte ? ccc_byte : fb_datab_r[7:0];
  assign fb_datab  = fb_datab_r[7:0];
  assign done_buff = (state==ST_WRITE) ? {work_datab[7:1],SDA_r} : work_datab; 
  assign obit_cnt  = bit_cnt;
  assign byte_2nd  = ccc_byte;
  assign byte_1st  = fb_datab_r;

  always @ (posedge clk_SCL_n or negedge RSTn) 
    if (!RSTn) 
      datab_done  <= 1'b0;
    else if (~|bit_cnt & (state == ST_WRITE) & (~in_ccc[0] | ccc_uh_r)) 
      datab_done  <= 1'b1;
    else 
      datab_done  <= 1'b0;
  always @ (posedge clk_SCL_n or negedge RSTn) 
    if (!RSTn) 
      datab_orun  <= 1'b0;
    else if (~STOP_r & (&bit_cnt[2:0]) & ~fb_data_use &
                 ((state == ST_WRITE) | (state[2:0] == ST_A7_A0_RnW[2:0])))
      datab_orun  <= 1'b1;
    else if (|ddr_fb_ctrl & ~fb_data_use)
      datab_orun  <= 1'b1;
    else 
      datab_orun  <= 1'b0;
  assign done_rdy      = (state == ST_W9TH) & in_i3c_msg;
  assign fb_datab_done = ~ccc_handled & 
                         ((~use_ccc_byte & datab_done) | pass_ccc | (|ddr_fb_ctrl)) & 
                         ~in_sa2byt & ~dev_spc & ~vgpio_ccc & ~vgpio_msg;
  assign fb_datab_err  = (state==ST_W9TH) & in_i3c_msg & (parity!=SDA_r);
  assign fb_s0s1_err   =  ~opt_s0ignore & (s0_error | s0_error2 | 
                            (fb_datab_err & (in_ccc[`CF_ONLY_b]==`CF_ONLY_POSS)));
  assign fb_orun   = datab_orun;
  assign parity    = ^work_datab ^ 1'b1;

  always @ (posedge clk_SCL_n or negedge RSTn) 
    if (!RSTn)
      in_i3c_msg   <= 1'b0;
    else if ((state[2:0] == ST_A7_A0_RnW[2:0]) & (bit_cnt==3'd1))
      if (i3c_protocol | ({work_datab[7:2],SDA_r} == 7'h7E))
        in_i3c_msg <= 1'b1;
      else
        in_i3c_msg <= 1'b0;

  //==================================================================
  // 8. 地址匹配逻辑
  //==================================================================
  generate 
  if (ENA_MAPPED[`MAP_ENA_b]) begin : match_mapped
    //FREE_VERSION_CUT - 免费版本中移除扩展映射和I2C扩展
  end else 
  begin
    assign map_da_match  = 1'b0;
    assign map_sa_match  = 1'b0;
    assign map_sa_idx    = 4'd0;
    assign map_spc       = 1'b0;
    assign opt_match_idx = 13'd0;
    assign in_sa2byt     = 1'b0;
    assign i2c_sw_rst    = 1'b0;
    assign vgpio_done    = 1'b0;
    assign vgpio_byte    = 8'd0;
    assign vgpio_spc     = 1'b0;
    assign vgpio_msg     = 1'b0;
    assign vgpio_ccc     = 1'b0;
    assign i2c_hs_enabled= 1'b0;
    assign raw_dasa_ena  = 1'b0;
  end

  if (|MAP_I2CID) begin : i2c_dev_id
    //FREE_VERSION_CUT - 免费版本中移除I2C设备ID
  end else 
  begin
    assign dev_spc        = 1'b0;
    assign spc_data_valid = 1'b0;
    assign spc_data_end   = 1'b0;
    assign spc_data       = 8'd0;
  end
  endgenerate

  assign is_dasa        = in_ccc[`CF_GRP_b] == `CFG_DASA;
  assign addr_match     = matched_7e |
                          (matched_p2p & is_dasa) |
                          (is_i3c ? (map_da_match|(work_datab[7:1]==dyn_addr[7:1])|vgpio_spc) : 
                            (map_sa_match|map_spc|dev_spc|(opt_static_addr[0]&(work_datab[7:1]==opt_static_addr[7:1]))));
  wire [7:1] err_tst    = work_datab[7:1] ^ 7'h7E;
  assign s0_error       = (state==ST_A7_A0_RnW) & ~|bit_cnt &  
                          ~matched_7e & ~|(err_tst & (err_tst-1)) &
                          is_i3c;
  assign s0_error2      = (state==ST_A7_A0_RnW) & ~|bit_cnt &
                          matched_7e & SDA_r & ~in_daa_mode;
  wire   is_da_matched  = (state==ST_ACK_NACK) & is_i3c & ~cf_SlvNack & 
                          (map_da_match|(work_datab[7:1]==dyn_addr[7:1])) & 
                          (~ccc_handled | ccc_valid_dir);
  wire   da_suppress    = vgpio_ccc | (in_ccc[0] & ~ccc_uh_mask);
  assign int_da_matched = is_da_matched & ~da_suppress;
  assign int_sa_matched = (state==ST_ACK_NACK) & ~is_i3c & ~cf_SlvNack &
                          ((map_sa_match&~dev_spc)|(opt_static_addr[0]&(work_datab[7:1]==opt_static_addr[7:1])));
  assign matched_7e     = (work_datab[7:1]==7'h7E);
  assign matched_p2p    = (work_datab[7:1]==7'h01);
  assign int_7e_matched = (state==ST_ACK_NACK) & matched_7e;
  assign ccc_valid_dir  = ccc_get == was_read;
  assign ccc_is_read    = (state==ST_A7_A0_RnW) ? SDA_r : was_read;
  wire   is_our_ccc     = state_in_CCC[`CF_BCAST] | (is_dasa & int_sa_matched) |
                          (is_dasa & map_sa_match & MAP_DA_AUTO[`MAPDA_DASA_b]) |
                          is_da_matched;

  reg    was_tb_val;
  wire   is_tb_val      = (tb_data_valid & was_tb_val) | dev_spc; 
  wire   valid_sa       = int_sa_matched & (~in_ccc[`CF_DIRECT] | is_dasa);
  wire   our_data       = is_da_matched | valid_sa | map_spc | dev_spc | vgpio_spc |
                          (is_dasa & map_sa_match & MAP_DA_AUTO[`MAPDA_DASA_b]);
  wire   our_ready      = ccc_handled ? is_our_ccc : 
                                        (was_read ? is_tb_val : fb_data_use);
  wire   daa_7e         = (ccc_byte[7:1]==7'h7E);
  assign ack_ok         = in_daa_mode ? (was_read&daa_7e&(~dyn_addr[0] | daa_act_ok)) :
                          our_data ? our_ready : 
                          ((matched_7e & ~was_read) |
                           (matched_p2p & is_dasa));

  always @ (posedge clk_SCL or negedge RSTn) 
    if (!RSTn)
      was_tb_val <= 1'b1;
    else if (our_data)
      was_tb_val <= tb_data_valid;
    else if (~was_tb_val)
      was_tb_val <= 1'b1;

  //==================================================================
  // 9. I2C尖峰滤波器控制
  //==================================================================
  reg spike_lock;
  always @ (posedge clk_SCL_n or negedge RSTn) 
    if (!RSTn)
      spike_lock <= 1'b0;
    else if (~spike_lock)
      if ((state[2:0] == ST_A7_A0_RnW[2:0]) & (bit_cnt == 3'h0) & (work_datab[7:1]==7'h7E))
      spike_lock <= 1'b1;
      else if (is_i3c)
        spike_lock <= 1'b1;
  assign i2c_spike_ok = ~spike_lock & ~i2c_hs_enabled;

  //==================================================================
  // 10. 事件处理逻辑
  //==================================================================
  assign event_losing = SDA_r != event_hdr_b;
  always @ (posedge clk_SCL_n or negedge RSTn) 
    if (!RSTn)
      event_won   <= 1'b0;
    else if (event_pending[2])
      event_won   <= 1'b1;
    else if (|event_pending[1:0]) begin
      if (state == ST_EV_A7_A0_RnW) begin
        if (event_losing)
          event_won <= 1'b0;
      end else if ((next_state != ST_MACK) & (state != ST_IBI_BYTE) & (next_state != ST_IBI_BYTE))
        event_won <= 1'b0;
    end else
      event_won   <= 1'b0;

  always @ (posedge clk_SCL_n or negedge RSTn) 
    if (!RSTn)
      ibi_rd      <= 1'b0;
    else if ((state==ST_MACK) & pend_IBI & ibi_has_byte & ibi_extdata) 
      ibi_rd      <= 1'b1;
    else if (ibi_rd)
      if ((state==ST_WAIT_SrP) | (state==ST_A7_A0_RnW))
        ibi_rd    <= 1'b0;
  assign ibi_in_extdata = ibi_rd & ~int_in_STOP;

  assign int_event_sent = ((state==ST_MACK) & 
                           (pin_SDA_in | ~ibi_has_byte | ~pend_IBI)) |
                          ((state==ST_IBI9TH) & pend_IBI);
  assign int_event_ack  = ((state==ST_MACK) & ~pin_SDA_in) |
                           (state==ST_IBI9TH);
  assign int_in_evproc  = (state==ST_MACK) | (state==ST_EV_A7_A0_RnW) |
                          (state==ST_IBI_BYTE) | (state==ST_IBI9TH);
  assign ibi_timec_marks[0] = (state==ST_IBI_BYTE) & (bit_cnt==3'd7) &
                              ~|timec_pos;
  assign ibi_timec_marks[1] = (state==ST_IBI_BYTE) & (bit_cnt==3'd7) &
                              (timec_pos==2'd1);
  assign ibi_extd       = ibi_extdata & tb_data_valid;

  //==================================================================
  // 11. CCC状态跟踪
  //==================================================================
  wire ccc_reset_n = RSTn & (~STOP_r | scan_no_rst); 
  always @ (posedge clk_SCL_n or negedge ccc_reset_n) 
    if (!ccc_reset_n)
      in_ccc                <= 5'd0;
    else if ((state==ST_ACK_NACK) & ~was_read) begin
      if (matched_7e) begin
        if (in_ccc[`CF_DIRECT] | in_ccc[`CF_DAA_M])
          in_ccc[`CF_ONLY_b]<= 5'd1;
        else
          in_ccc[`CF_POSS]  <= 1'b1;
        in_ccc[`CF_GRP_b]   <= 3'd0;
      end else if (is_dasa) begin
        if (opt_static_addr[0] & (work_datab[7:1]==opt_static_addr[7:1]))
          in_ccc[`CF_GRP_b] <= `CFG_DASA_SA;
        else if (work_datab[7:1]==7'h01)
          in_ccc[`CF_GRP_b] <= `CFG_DASA_P2P;
        else
          in_ccc[`CF_GRP_b] <= `CFG_DASA;
      end else if (in_ccc[`CF_GRP_b] == `CFG_RSTDAA_D) begin
        if (is_i3c & (work_datab[7:1]==dyn_addr[7:1]))
          in_ccc[`CF_GRP_b] <= `CFG_RSTDAA_US;
      end
    end else if ((in_ccc[`CF_DIRECT:0]==3'b001) & (state==ST_W9TH) & (parity==SDA_r)) begin
      in_ccc[`CF_DIRECT:`CF_BCAST] <= work_datab[7] ? 2'b10 : 2'b01;
      if (work_datab == `CCC_ENTDAA)
        in_ccc[`CF_DAA_M]   <= 1'b1;
      else if (work_datab[7:3] == `CCC_ENTHDR_MSK) begin
        in_ccc[`CF_HDR_M]   <= 1'b1;
        in_ccc[`CF_GRP_b]   <= |work_datab[2:0] ? 3'd0 : `CFG_HDRDDR;
      end else if (work_datab == `CCC_SETDASA_D)
        in_ccc[`CF_GRP_b]   <= `CFG_DASA;
      else if (work_datab == `CCC_SETNEWDA_D)
        in_ccc[`CF_GRP_b]   <= `CFG_NEWDA;
      else if (work_datab == `CCC_RSTDAA)
        in_ccc[`CF_GRP_b]   <= `CFG_RSTDAA_US;
      else if (work_datab == `CCC_RSTDAA_D)
        in_ccc[`CF_GRP_b]   <= `CFG_RSTDAA_D;
      else
        in_ccc[`CF_GRP_b]   <= 0;
    end else if (|in_ccc & any_START_d) begin
      if (real_START_d |
          (in_ccc[`CF_ONLY_b] == `CF_ONLY_POSS) |
          (in_ccc[`CF_BCAST] & ~in_ccc[`CF_DAA_M])) begin
        in_ccc[`CF_ONLY_b]  <= 5'd0;
        in_ccc[`CF_GRP_b]   <= 3'd0;
      end
    end 
  assign state_in_CCC = in_ccc & {8{~STOP_r}};
  assign in_daa_mode  = in_ccc[`CF_DAA_M];
  assign next_ccc     = ((in_ccc[`CF_DIRECT:0]==3'b001) & 
                          ((next_state==ST_W9TH) | (state_base==ST_W9TH))) ?
                          (work_datab[7] ? 2'b10 : 2'b01) : 2'b00;
  assign next_hdr[0]  = (in_ccc[`CF_DIRECT:0]==3'b001) & (state==ST_W9TH) &
                        (parity==SDA_r) & (work_datab[7:3]==`CCC_ENTHDR_MSK);
  assign next_hdr[1]  =  ~|work_datab[2:0];
  generate if (ENA_HDR[`HDR_DDR_b]) begin : ddr_act
    reg  ddr_active_r;
    always @ (posedge clk_SCL or negedge ccc_reset_n) 
      if (!ccc_reset_n)
        ddr_active_r <= 1'b0;
      else if ((in_ccc[`CF_DIRECT:0]==3'b001) & (state==ST_W9TH) &
               (work_datab[7:3]==`CCC_ENTHDR_MSK) & ~|work_datab[2:0] &
               (parity==pin_SDA_in))
        ddr_active_r <= 1'b1;
    assign in_ddr = ddr_active_r;
  end else begin
    assign in_ddr = 1'b0;
  end endgenerate

  assign next_int_ccc = (in_ccc[`CF_DIRECT:0]==3'b001) & (state==ST_W9TH) & 
                        (parity==SDA_r) & ~ccc_handling & ccc_uh_mask & ~cf_SlvNack;
  assign next_bc_ccc  = next_int_ccc & ~work_datab[7];
  assign next_dc_ccc  = ccc_supp[1] & int_da_matched & in_ccc[`CF_DIRECT];
  always @ (posedge clk_SCL_n or negedge RSTn) 
    if (!RSTn) begin
      ccc_supp    <= 2'b00;
      ccc_uh_r    <= 1'b0;
    end else if (next_int_ccc) begin
      if (work_datab[7])
        ccc_supp  <= 2'b10;
      else if (work_datab[7:3] != `CCC_ENTHDR_MSK)
        ccc_supp  <= 2'b01;
      ccc_uh_r    <= 1'b1;
    end else if (ccc_supp[0])
      ccc_supp[0] <= 1'b0;
    else if (ccc_handling | ~ccc_uh_r)
      ccc_supp    <= 2'b00;
    else if (next_dc_ccc)
      ccc_supp[0] <= 1'b1;
    else if (~|in_ccc[`CF_DIRECT:0] & (ccc_supp[1] | ccc_uh_r)) begin
      ccc_supp[1] <= 1'b0;
      ccc_uh_r    <= 1'b0;
    end
  assign int_ccc = ccc_supp[0] & ~vgpio_ccc;
  always @ (posedge clk_SCL_n or negedge ccc_reset_n) 
    if (!ccc_reset_n)
      pass_ccc   <= 1'b0;
    else if (next_dc_ccc  |
             (next_bc_ccc & (work_datab[7:3] != `CCC_ENTHDR_MSK)))
      pass_ccc   <= 1'b1;
    else if (pass_ccc)
      pass_ccc   <= 1'b0;

  //==================================================================
  // 12. DAA仲裁逻辑
  //==================================================================
  generate if (PIN_MODEL == `PINM_COMBO) begin : combo_for_daa
    always @ (posedge clk_SCL_n or negedge RSTn) 
      if (!RSTn)
        daa_lost   <= 1'b0;
      else if (in_daa_mode)
        if (state == ST_ACK_NACK)
          daa_lost <= 1'b0;
        else if ((state == ST_DAA) & daa_inp_drv)
          daa_lost <= ~SDA_r & daa_inp_bit;
    assign daa_active = (state == ST_DAA) & (~is_i3c | daa_act_ok);
  end else begin
    always @ (posedge clk_SCL_n or negedge RSTn) 
      if (!RSTn)
        daa_lost   <= 1'b0;
      else if (in_daa_mode)
        if (state == ST_A7_A0_RnW)
          daa_lost <= 1'b0;
        else if ((state == ST_DAA) & daa_inp_drv)
          daa_lost <= ~SDA_r & ~r_SDA_oena;
    wire next_daa = (state==ST_ACK_NACK) & in_daa_mode & was_read;
    wire stay_daa = (state==ST_DAA) & ~(daa_lost | daa_done);
    assign daa_active = (next_daa | stay_daa)  & (~is_i3c | daa_act_ok);
  end endgenerate

  //==================================================================
  // 13. 时间戳控制
  //==================================================================
  always @ (posedge clk_SCL_n or negedge RSTn) 
    if (!RSTn)
      timec_pos <= 2'd0;
    else if ((state==ST_IBI_BYTE) & ~|bit_cnt & ibi_timec)
      timec_pos <= timec_pos + 2'd1;
    else if ((state != ST_IBI_BYTE) & (state != ST_IBI9TH))
      timec_pos <= 2'd0;
  assign ibi_timec_sel = {(state==ST_IBI_BYTE)|(state==ST_IBI9TH),
                          timec_pos};

  //==================================================================
  // 14. SDA引脚输出控制
  //==================================================================
  wire [7:1] ibi_da;
  generate  if (ENA_MAPPED[`MAP_ENA_b]) begin : ibi_mapped
    genvar i,j;
    wire [(MAP_CNT*8)-1:0] map_regs_ad  = SetMappedDASA[(MAP_CNT*10)-1:(MAP_CNT*2)];
    wire [MAP_CNT:0] da[0:7];
    for (i = 1; i <= MAP_CNT; i = i + 1) begin : make_da
      for (j = 0; j < 8; j = j +1) begin : make_da2
        assign da[7-j][i] = (ibi_mapidx==i) ? map_regs_ad[(((i-1)*8)+7) - j] : 1'b0;
      end
    end
    for (j = 0; j < 8; j = j +1) begin : make_da3
      assign da[j][0] = dyn_addr[j];
    end
    for (j = 1; j < 8; j = j +1) begin : make_ibida
      assign ibi_da[j] = |da[j];
    end
  end else begin
    assign ibi_da = dyn_addr[7:1];
  end endgenerate

  assign event_drive = start_r & event_won;
  assign event_hdr   = pend_HJ ? {7'h02,1'b0} : {ibi_da, pend_IBI};
  assign event_hdr_b = event_hdr[bit_cnt];
  assign force_START = force_sda & (STOP_r | init_stop) & pin_SCL_in;

  // 引脚输出控制逻辑（根据PIN_MODEL选择组合或寄存器模式）
  generate if (PIN_MODEL == `PINM_COMBO) begin : use_combo_pins
    assign SDA_oe = 
                  ((state==ST_EV_A7_A0_RnW) & ~event_hdr_b) |
                  ((state==ST_ACK_NACK)  & ack_ok &       
                      (~in_i3c_msg | (~event_drive & (~pin_SCL_in | was_read)))) |
                  ((state==ST_DAA)       & daa_inp_drv & ~daa_lost & ~daa_inp_bit) |
                  ((state==ST_MACK)      & ((next_state==ST_IBI_BYTE) & pin_SCL_in)) |
                  (state==ST_IBI_BYTE)     |
                  ((state==ST_IBI9TH)    & (|timec_pos |  ~pin_SCL_in | ibi_extd)) | 
                  ((state==ST_READ)      & ~is_term & (in_i3c_msg | ~rdata_bit)) |
                  ((state==ST_R9TH)      & (in_i3c_msg & ~pin_SCL_in)) | 
                  ((state==ST_W9TH)      & ~in_i3c_msg) |
                  (ENA_HDR[`HDR_DDR_b]   & |ddr_tb_ctrl & ddr_tb_oe[~pin_SCL_in]);

    assign SDA_out = 
                  ((state==ST_DAA)       & daa_inp_bit) |
                  ((state==ST_IBI_BYTE)  & opt_ibi_byte[bit_cnt]) |
                  ((state==ST_IBI9TH)    & (|timec_pos|ibi_extd)) | 
                  ((state==ST_READ)      & (in_i3c_msg & rdata_bit)) |
                  ((state==ST_R9TH)      & ~i3c_end) |
                  (ENA_HDR[`HDR_DDR_b]   & |ddr_tb_ctrl & rddr_bits[~pin_SCL_in]);

    wire is_valid_ddr = ENA_HDR[`HDR_DDR_b] & |ddr_tb_ctrl;
    assign pin_SDA_oena = force_START | ((|state[3:2] |is_valid_ddr) & SDA_oe);
    assign pin_SDA_out  = pin_SDA_oena ? SDA_out : 1'b1;
    assign pin_SDA_oena_rise = 1'b0;
    assign r_SDA_oena = 0;
  end else begin : use_reg_pins
    wire is_da_or_sa    = is_i3c ? (map_da_match|(work_datab[7:1]==dyn_addr[7:1])|vgpio_spc) : 
                                   (map_sa_match|map_spc|dev_spc|(opt_static_addr[0]&(work_datab[7:1]==opt_static_addr[7:1])));
    wire willbe_read    = SDA_r;
    wire willbe_our_ccc = state_in_CCC[`CF_BCAST] | (is_dasa & is_da_or_sa) |
                          ((ccc_get == willbe_read) & is_da_or_sa);
    wire our_ack        = ccc_handled ? willbe_our_ccc :
                          (willbe_read ? (tb_data_valid|dev_spc) : fb_data_use);
    wire willbe_daa_7e  = (ccc_byte[7:1]==7'h7E);
    wire pre_ack_ok     = in_daa_mode ? (willbe_read&willbe_daa_7e&(~dyn_addr[0]|daa_act_ok)) :
                          is_da_or_sa ? our_ack : 
                          ((matched_7e & ~willbe_read) |
                           (matched_p2p & is_dasa));
    wire daa_will_lose   = (state==ST_DAA) ? (~SDA_r & ~r_SDA_oena & ~daa_acknack) : 
                                             daa_lost;
    wire [2:0] rdata_bc  = ((state==ST_ACK_NACK)|(state==ST_R9TH)|(state==ST_IBI9TH)) ? 3'd7 : 
                          (bit_cnt-3'd1);
    wire rdata_prebit    = ccc_handled    ? ccc_tb_data :
                           spc_data_valid ? spc_data[rdata_bc] :
                           tb_data_valid  ? tb_datab[rdata_bc] : 
                           1'b1;
    wire [2:0] event_bc  = int_in_STOP ? 3'd7 : (bit_cnt-3'd1);
    wire event_pre_hdr_b = event_hdr[event_bc];
    wire [2:0] ibi_bc    = ((state==ST_MACK)|(state==ST_IBI9TH)) ? 3'd7 : 
                           (bit_cnt-3'd1);
    wire   next_start    = int_in_STOP | (SDA_r & ~pin_SDA_in);
    wire next_timec_pos0 = ~ibi_timec |
                           ((state==ST_IBI_BYTE) & ~|bit_cnt & (&timec_pos));

    assign SDA_oe = (|next_state[3:2] & ~next_start & (
                  ((next_state==ST_ACK_NACK)  & pre_ack_ok) |
                  ((next_state==ST_DAA)       & daa_inp_drv & ~daa_will_lose & ~daa_inp_bit) |
                  ((next_state==ST_READ)      & (in_i3c_msg | ~rdata_prebit)) |
                  ((next_state==ST_R9TH)      & in_i3c_msg) | 
                  ((next_state==ST_W9TH)      & ~in_i3c_msg) |
                  ((next_state==ST_EV_A7_A0_RnW) & ~event_pre_hdr_b) |
                  (next_state==ST_IBI_BYTE)     |
                  (next_state==ST_IBI9TH)
                  )) |
                  (int_in_STOP & ~SDR_hold & event_pending[2] & |event_pending[1:0] & ~event_pre_hdr_b) |
                  (ENA_HDR[`HDR_DDR_b] & ddr_tb_oe[1]);

    assign SDA_out = 
                  ((next_state==ST_DAA)       & daa_inp_bit)  |
                  ((next_state==ST_IBI_BYTE)  & opt_ibi_byte[ibi_bc]) |
                  ((next_state==ST_IBI9TH)    & (~next_timec_pos0 | ibi_extd)) | 
                  ((next_state==ST_READ)      & (in_i3c_msg & rdata_prebit)) |
                  ((next_state==ST_R9TH)      & ~i3c_end)     |
                  (ENA_HDR[`HDR_DDR_b]        & |ddr_tb_ctrl & rddr_bits[1]);

    wire rise_chg = in_i3c_msg &
                    ~any_START_d & ~STOP_r    &
                    ((r_SDA_oena & (
                       ((state_base==ST_ACK_NACK)  & ~was_read) |
                       (state_base==ST_IBI9TH)                  |
                       (state_base==ST_R9TH)                    )
                     ) |
                     (~r_SDA_oena & (
                       ((state_base==ST_MACK) & (~pin_SDA_in & (pend_IBI & ibi_has_byte))) )
                     ) |
                       (ENA_HDR[`HDR_DDR_b]   & ddr_tb_oe[0])
                     );

    reg   SDA_oena_r;
    assign r_SDA_oena = SDA_oena_r;
    always @ (posedge clk_SCL_n or negedge state_rst_n) 
      if (!state_rst_n)
        SDA_oena_r <= 1'b0;
      else if (SDA_oe != SDA_oena_r)
        SDA_oena_r <= ~SDA_oena_r;

    if (PIN_MODEL == `PINM_REG) begin : pinm_reg_local
      reg SDA_out_r;
      always @ (posedge clk_SCL_n or negedge RSTn) 
        if (!RSTn)
          SDA_out_r <= 1'b0;
        else if (SDA_out_r != SDA_out)
          SDA_out_r <= ~SDA_out_r;
      wire hdr_xor = (force_START & SDA_out_r) | 
                     (ENA_HDR[`HDR_DDR_b] & ddr_tb_oe[1] & pin_SCL_in & (rddr_bits[0] != SDA_out_r));
      `ifdef REGPAD_NOSTOP
        assign pin_SDA_oena = force_START | (SDA_oena_r ^ (rise_chg & pin_SCL_in));
      `else
        assign pin_SDA_oena = force_START | ((SDA_oena_r ^ (rise_chg & pin_SCL_in)) & ~STOP_r);
      `endif
      assign pin_SDA_out  = SDA_out_r ^ hdr_xor;
      assign pin_SDA_oena_rise = 1'b0;
    end else begin : pinm_reg_ext
      assign pin_SDA_oena = SDA_oe;
      assign pin_SDA_oena_rise[0] = rise_chg;
      if (ENA_HDR[`HDR_DDR_b]) begin : hdr_pinext
        assign pin_SDA_oena_rise[3] = ddr_tb_oe[1];
        assign pin_SDA_oena_rise[2] = rddr_bits[0];
      end 
      assign pin_SDA_oena_rise[1] = force_START;
      assign pin_SDA_out  = SDA_out;
    end
  end endgenerate

endmodule