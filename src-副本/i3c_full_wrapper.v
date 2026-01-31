`include "i3c_params.v"                 // 本地参数/常量

// 主模块：I3C完整封装模块
// 功能：MIPI I3C外层封装，用于基于网络的应用，包含时钟域交叉(CDC)和数据缓冲管理
// 说明：
// 1. 实例化Slave组件和可选的Master中的Slave组件
// 2. 使用系统时钟（如APB总线的PCLK）处理时钟域交叉
// 3. 管理数据缓冲/FIFO，包括外部FIFO信号支持
module i3c_full_wrapper #(
    // 参数由上层驱动，控制模块构建和运行方式
    parameter ENA_ID48B       = `ID48B_CONST, // ID48位常量或寄存器
    parameter  ID_48B         = 48'h0,  // 必须从上填充48位ID
    parameter ID_AS_REGS      = 12'd0,  // 用于ID和其他用途的寄存器
    parameter  ID_BCR         = 8'd0,   // 如果不是来自寄存器，则从上填充BCR
    parameter  ID_DCR         = 8'd0,   // 如果不是来自寄存器，则从上填充DCR
    parameter ENA_SADDR       = `SADDR_NONE, // 从地址模式：无、常量、寄存器/网络
    parameter  SADDR_P        = 0,      // 7位从地址
    parameter ENA_MAPPED      = 5'd0,   // 是否允许额外的DA/SA及相关功能
    parameter  MAP_CNT        = 4'd1,   // 允许的额外DA/SA数量
    parameter  MAP_I2CID      = 24'd0,  // I2C扩展的DevID
    parameter  MAP_DA_AUTO    = {5'd0,1'b0,1'b0,3'd0,1'b0,1'b0,1'b0},
    parameter  MAP_DA_DAA     = 0,      // 非MMR且PID/DCR!=0时的位数组
    parameter ENA_IBI_MR_HJ   = 0,      // IBI/MR/HJ事件使能掩码
    parameter  CLK_SLOW_BITS  = 6,      // 总线可用计数器位数
    parameter  CLK_SLOW_MATCH = 6'd47,  // 总线可用计数匹配值
    parameter  CLK_SLOW_HJMUL = 10'd1000,// HJ超时乘法器
    parameter  ERROR_HANDLING = 3'd0,   // 错误处理配置
    parameter  ENA_TIMEC      = 6'b000010,    // 时间控制使能
    parameter  TIMEC_FREQ_ACC = {8'd24,8'd10},// 时间控制频率和精度
    parameter ENA_CCC_HANDLING= 6'd0,   // CCC处理支持
    parameter RSTACT_CONFIG   = 26'd0,  // 从机复位RSTACT CCC配置
    parameter MAX_RDLEN       = 0,      // 默认S->M最大长度
    parameter MAX_WRLEN       = 0,      // 默认M->S最大长度
    parameter MAX_DS_WR       = 0,      // M->S数据速度限制
    parameter MAX_DS_RD       = 0,      // S->M数据速度限制
    parameter MAX_DS_RDTURN   = 0,      // S->M读取请求延迟需求
    parameter SEL_BUS_IF      = 5'd0,   // 总线接口选择
    parameter FIFO_TYPE       = 0,      // FIFO类型和宽度
    parameter  EXT_FIFO       = 3'd0,   // 外部FIFO选择
    parameter  ENA_TOBUS_FIFO = 0,      // 到总线FIFO深度（2的幂）
    parameter  ENA_FROMBUS_FIFO=0,      // 从总线FIFO深度（2的幂）
    parameter ENA_HDR         = 0,      // HDR模式使能
    parameter BLOCK_ID        = 0,      // 块ID寄存器使能
    parameter ENA_MASTER      = 0,      // 主设备使能
    parameter PIN_MODEL       = `PINM_COMBO, // 引脚模型
    parameter priv_sz         = PIN_MODEL[1] ? (ENA_HDR[0] ? 3 : 1) : 0,// 私有信号宽度
    parameter [7:0] PID_CNT   = MAP_DA_AUTO[`MAPDA_DAA_PID_lb+4:`MAPDA_DAA_PID_lb], // PID计数
    parameter FULL_DBG_MX     = 0       // 调试观察器宽度
  )
  (
  // 时钟和复位
  input               RSTn,             // 系统复位
  input               CLK,              // 系统时钟
  input               CLK_SLOW,         // IBI强制时钟
  output              slow_gate,        // CLK_SLOW门控信号
  input               CLK_SLOW_TC,      // 时间控制时钟
  output              tc_slow_gate,     // CLK_SLOW_TC门控信号
  input               clk_FastTernary,  // 三态时钟
  // 引脚定义
  input               pin_SCL_in,       // SCL输入
  output              pin_SCL_out,      // SCL输出
  output              pin_SCL_oena,     // SCL输出使能
  input               pin_SDA_in,       // SDA输入
  output              pin_SDA_out,      // SDA输出
  output              pin_SDA_oena,     // SDA输出使能
  output  [priv_sz:0] pin_SDA_oena_rise,// SDA上升沿输出使能
  output              i2c_spike_ok,     // I2C尖峰滤波器使能
  output              i2c_hs_enabled,   // I2C高速模式指示
  // 配置输入
  input               cf_SlvEna,        // 从设备使能
  input               cf_SlvNack,       // 不ACK私有消息
  input         [7:0] cf_SlvSA,         // 从设备I2C地址
  input         [3:0] cf_IdInst,        // ID实例
  input               cf_IdRand,        // ID随机部分号
  input               cf_Offline,       // 离线模式
  input        [31:0] cf_Partno,        // 部件号
  input         [7:0] cf_IdBcr,         // BCR
  input         [7:0] cf_IdDcr,         // DCR
  input        [14:0] cf_IdVid,         // 供应商ID
  input               cf_DdrOK,         // DDR模式使能
  input               cf_TspOK,         // TSP模式使能
  input               cf_TslOK,         // TSL模式使能
  input        [11:0] cf_MaxRd,         // 最大读取长度
  input        [11:0] cf_MaxWr,         // 最大写入长度
  input        [23:0] cf_RstActTim,     // 从设备复位恢复时间
  input         [7:0] cf_BAMatch,       // 总线可用匹配值
  input        [15:0] cf_TCclk,         // 时间控制时钟信息
  input               cf_s0ignore,      // 忽略S0错误
  input               cf_matchss,       // 匹配Start/Stop
  input         [7:0] cf_SetDA,         // 主设备动态地址
  input [(MAP_CNT*10)-1:0] cf_SetMappedDASA, // 映射的SA/DA
  input         [2:0] cf_SetSA10b,      // 10位从地址
  input               cf_MasterAcc,     // 接受主设备权限
  input               cf_IbiExtData,    // IBI扩展数据
  input         [3:0] cf_IbiMapIdx,     // IBI映射索引
  input         [1:0] cf_HdrCmd,        // HDR命令使能
  input         [6:0] cf_CccMask,       // CCC掩码
  input         [8:0] cf_vgpio,         // VGPIO控制
  output              vgpio_done,       // VGPIO完成脉冲
  output        [7:0] raw_vgpio_byte,   // VGPIO字节数据
  input   [MAP_CNT-1:0] map_daa_use,      // MAP自动DAA使能
  input [(MAP_CNT*8)-1:0] map_daa_dcr,    // MAP DAA的DCR
  input [(MAP_CNT*PID_CNT)-1:0] map_daa_pid, // MAP DAA的PID
  output              hdr_new_cmd,      // HDR新命令脉冲
  output        [7:0] raw_hdr_cmd,      // HDR命令原始数据
  output              map_rstdaa,       // MAP RSTDAA脉冲
  output              map_setaasa,      // MAP SETAASA脉冲
  output              map_daa_ena,      // MAP DAA使能脉冲
  output        [3:0] map_sa_idx,       // MAP SA索引
  output        [7:1] map_daa_da,       // MAP DAA新地址
  // I2C扩展功能
  input         [2:0] i2c_dev_rev,      // 设备版本
  output              i2c_sw_rst,       // 软件复位
  // SCL域原始状态输出
  output      [29:28] raw_ActState,     // 活动状态
  output      [27:24] raw_EvState,      // 事件状态
  output        [2:0] raw_TimeC,        // 时间控制状态
  output        [6:0] raw_Request,      // 总线请求状态
  output        [7:0] raw_DynAddr,      // 动态地址
  output        [2:0] raw_DynChgCause,  // 动态地址变化原因
  output       [13:0] raw_timec_sync,   // 时间控制同步
  output        [3:0] raw_slvr_reset,   // 从设备复位控制
  input               iraw_rst_slvr_reset, // SCL域清除从设备复位
  input               iraw_slvr_irq,    // SCL域块复位
  output       [12:0] raw_match_idx,    // 匹配索引
  output              raw_matched,      // 地址匹配指示
  // 安全原始状态输入
  input         [7:6] sraw_ActMode,     // 系统活动模式
  input         [3:0] sraw_PendInt,     // 挂起中断
  input        [15:8] sraw_StatusRes,   // 状态保留位
  // 寄存器接口输出
  output       [19:8] outp_IntStates,   // 中断状态
  input         [2:0] reg_EvPend,       // 事件请求
  input         [7:0] reg_EvIbiByte,    // IBI字节数据
  output              outp_EvNoCancel,  // 事件不可取消标志
  output      [22:20] outp_EvDet,       // 事件详情
  output        [5:0] outp_GenErr,      // 通用错误
  output       [11:8] outp_DataErr,     // 数据错误
  input         [5:0] msk_GenErr,       // 通用错误掩码
  input        [11:8] msk_DataErr,      // 数据错误掩码
  input        [19:8] reg_clrIntStates, // 清除中断状态
  input         [5:0] reg_clrGenErr,    // 清除通用错误
  input        [11:8] reg_clrDataErr,   // 清除数据错误
  input               reg_holdOErr,     // 保持溢出错误
  output              outpflg_MaxRd,    // 最大读取改变标志
  output              outpflg_MaxWr,    // 最大写入改变标志
  output       [11:0] outp_MaxRW,       // 最大读/写值
  output              outp_to_master,   // 切换到主设备
  // 数据/FIFO接口
  input               reg_TbEnd,        // 到总线数据结束
  input               reg_TbFlush,      // 到总线缓冲刷新
  input               reg_FbFlush,      // 从总线缓冲刷新
  input         [5:4] reg_TxTrig,       // TX触发级别
  input         [7:6] reg_RxTrig,       // RX触发级别
  output      [20:16] outp_TxCnt,       // TX计数器
  output      [28:24] outp_RxCnt,       // RX计数器
  output              outp_TxFull,      // TX FIFO满
  output              outp_RxEmpty,     // RX FIFO空
  input         [1:0] regflg_wr_cnt,    // 写计数标志
  input         [7:0] reg_wdata,        // 写数据
  input         [1:0] regflg_rd_cnt,    // 读计数标志
  output        [7:0] outp_fb_data,     // 读数据
  // 外部FIFO接口
  input               ixf_tb_avail,     // 到总线数据可用
  input               ixf_tb_last,      // 到总线最后字节
  input         [7:0] ixf_tb_data,      // 到总线数据
  output              oxf_tb_start,     // 到总线开始脉冲
  output              oxf_tb_used,      // 到总线数据使用脉冲
  input               ixf_fb_free,      // 从总线空间可用
  output              oxf_fb_req,       // 从总线请求脉冲
  output        [7:0] oxf_fb_data,      // 从总线数据
  output              oxf_fb_eof,       // 从总线帧结束
  // IBI FIFO接口
  input        [10:0] ibi_wr_fifo,      // IBI FIFO写信号
  output              ibi_wr_ack,       // IBI FIFO写确认
  // 主/从模式选择
  input               is_slave,         // 从模式使能
  output              d_tb_data_valid,  // 到总线数据有效
  output              tb_pclk_valid,    // PCLK域到总线数据有效
  output        [7:0] d_tb_datab,       // 到总线数据
  output              d_tb_end,         // 到总线数据结束
  input               m_tb_datab_ack,   // 主设备到总线数据确认
  output              fb_data_use,      // 从总线数据使用
  input         [7:0] m_fb_datab,       // 主设备从总线数据
  input               m_fb_datab_done,  // 主设备从总线数据完成
  // 调试接口
  output[FULL_DBG_MX:0]full_debug_observ, // 调试观察器
  // 扫描测试接口
  input               scan_single_clock,// 单时钟域扫描
  input               scan_clk,         // 扫描时钟
  input               scan_no_rst,      // 禁止分层复位
  input               scan_no_gates     // 禁止时钟门控
  );

  // 内部连线定义
  wire                PCLK    = CLK;    // 简化命名
  wire                PRESETn = RSTn;   // 简化命名
  
  // 事件管理相关信号
  wire          [2:0] event_pending;
  wire                force_sda;
  wire                ibi_has_byte;
  wire          [7:0] opt_ibi_byte;
  wire                ibi_timec;
  wire          [1:0] ibi_timec_marks;
  wire          [2:0] ibi_timec_sel;
  wire                raw_ibi_in_extdata;
  wire                timec_oflow;
  wire                timec_oflow_clr;
  wire                hold_engine;
  wire                slvena_delay;
  
  // IBI FIFO相关信号
  wire                ibs_tb_data_valid;
  wire          [7:0] ibs_tb_datab;
  wire                ibs_tb_end;
  wire                ibs_tb_datab_ack;
  
  // 时钟相关信号
  wire                clk_SCL;
  wire                clk_SCL_n;
  
  // 数据缓冲相关信号
  wire                s_tb_datab_ack;
  wire                s_tb_data_valid;
  wire          [7:0] s_tb_datab;
  wire                s_tb_end;
  wire                d_tb_datab_ack, d2_tb_datab_ack;
  wire                tb_urun_nack;
  wire                tb_urun;
  wire                tb_term;
  wire          [7:0] fb_datab, s_fb_datab;
  wire                fb_datab_done, s_fb_datab_done;
  wire                fb_datab_err;
  wire          [2:0] fb_ddr_errs;
  wire                fb_s0s1_err;
  wire                brk_s0s1_err;
  wire                abt_s0s1_err;
  wire                read_abort;
  wire                fb_orun;
  wire                local_fb_int;
  wire                localn_fb_ready;
  wire          [7:0] localn_fb_data;
  wire                localn_fb_ack;
  wire                avail_tb_ready;
  wire          [7:0] avail_tb_data;
  wire                avail_tb_ack; 
  wire                avail_tb_end; 
  wire                local_tb_int;
  
  // PCLK域缓冲信号
  wire          [3:0] set_fb_err;
  wire                set_rd_err;
  wire                set_fb_orun;
  wire                set_tb_urun_nack;
  wire                set_tb_urun;
  wire                set_tb_term;
  wire                flg_maxrd, flg_maxwr;
  
  // SCL域中断和状态信号
  wire                int_start_seen;
  wire                int_start_err;
  wire                int_da_matched;
  wire                int_sa_matched;
  wire                int_7e_matched;
  wire                int_ddr_matched;
  wire                int_in_STOP;
  wire                int_event_sent;
  wire                int_event_ack;
  wire                int_in_evproc;
  wire                int_ccc_handled;
  wire                int_ccc;
  wire                state_in_ccc;
  wire                state_in_daa;
  wire                state_in_hdr;
  wire                opt_MasterAcc;
  wire          [6:0] raw_req;
  wire                clr_slvr_cause;
  wire                raw_vgpio_done;
  wire                dyn_addr_chg;
  wire                fb_hdr_exit;
  wire                scl_hdr_new_cmd;
  wire          [7:0] scl_hdr_cmd;
  wire                raw_setaasa;
  wire                raw_rstdaa;
  wire                raw_daa_ena;

  // 调试观察器支持
  `ifdef ENA_DBG_OBSERVE
   `include "dbg_observ_full.v"
  `else
   localparam SLV_DBG_MX = 0;
   wire [SLV_DBG_MX:0] slv_debug_observ;
   assign full_debug_observ = 0;
  `endif

  // 主/从模式FIFO选择逻辑
  assign d2_tb_datab_ack = is_slave ? s_tb_datab_ack : m_tb_datab_ack;
  assign fb_datab        = is_slave ? s_fb_datab      : m_fb_datab;
  assign fb_datab_done   = is_slave ? s_fb_datab_done : m_fb_datab_done;
  assign tb_pclk_valid   = |outp_TxCnt & 
                (~d2_tb_datab_ack | (outp_TxCnt!=4'b0001));

  // IBI FIFO多路选择器
  generate if (ENA_IBI_MR_HJ[`EV_EXTFIFO_b]) begin : ibi_fifo_mux
    assign s_tb_data_valid = raw_ibi_in_extdata ? ibs_tb_data_valid : d_tb_data_valid;
    assign s_tb_datab      = raw_ibi_in_extdata ? ibs_tb_datab      : d_tb_datab;
    assign s_tb_end        = raw_ibi_in_extdata ? ibs_tb_end        : d_tb_end;
    assign ibs_tb_datab_ack= raw_ibi_in_extdata ? s_tb_datab_ack    : 1'b0;
    assign d_tb_datab_ack  = raw_ibi_in_extdata ? 1'b0              : s_tb_datab_ack;
  end else begin
    assign s_tb_data_valid = d_tb_data_valid;
    assign s_tb_datab      = d_tb_datab;
    assign s_tb_end        = d_tb_end;
    assign d_tb_datab_ack  = d2_tb_datab_ack;
    assign ibs_tb_datab_ack= 1'b0;
  end endgenerate

  // 从总线数据缓冲管理模块
  // 功能：处理来自主设备的数据缓冲，支持FIFO或单缓冲模式
  i3c_data_frombus #(.FIFO_TYPE(FIFO_TYPE),.ENA_FROMBUS_FIFO(ENA_FROMBUS_FIFO),
                     .ANY_HDR(|ENA_HDR))
    frombus 
    (
    .RSTn             (PRESETn), 
    .CLK              (PCLK), 
    .SCL              (clk_SCL), 
    .SCL_n            (clk_SCL_n),
    .notify_fb_ready  (localn_fb_ready), 
    .notify_fb_data   (localn_fb_data),
    .notify_fb_ack    (localn_fb_ack), 
    .fb_flush         (reg_FbFlush),
    .set_fb_err       (set_fb_err), 
    .set_fb_orun      (set_fb_orun), 
    .clear_fb_err     (reg_clrDataErr[11:8]),
    .clear_fb_orun    (reg_clrGenErr[0]), 
    .avail_byte_cnt   (outp_RxCnt),
    .avail_fb_empty   (outp_RxEmpty),
    .rx_trig          (reg_RxTrig),
    .int_rx           (local_fb_int),
    .fb_data_use      (fb_data_use), 
    .fb_datab         (fb_datab), 
    .fb_datab_done    (fb_datab_done), 
    .fb_datab_err     (fb_datab_err),
    .fb_orun          (fb_orun),
    .fb_ddr_errs      (fb_ddr_errs),
    .fb_s0s1_err      (fb_s0s1_err),
    .brk_s0s1_err     (brk_s0s1_err),
    .scan_no_rst      (scan_no_rst)
  );  
  assign outp_fb_data  = localn_fb_data;
  
  // 从总线FIFO接口生成
  generate if (FIFO_TYPE[`FIFO_EXT_b]==0 || EXT_FIFO==0) begin : no_fifo_fb
    assign localn_fb_ack   = |regflg_rd_cnt;
    assign oxf_fb_req      = 0;
    assign oxf_fb_data     = 0;
    assign oxf_fb_eof      = 0;
  end else begin : fifo_fb
    assign oxf_fb_req  = ixf_fb_free & localn_fb_ready;
    assign oxf_fb_data = localn_fb_data;
    assign localn_fb_ack = ixf_fb_free;
    SYNC_ASet_Seq2 sync_fb_eof(.CLK(PCLK), .RSTn(PRESETn), 
                               .async_set(int_start_seen | int_in_STOP), 
                               .local_clear(1'b1), .o_pulse(oxf_fb_eof));
  end endgenerate

  // 到总线数据缓冲管理模块
  // 功能：处理发送到主设备的数据缓冲，支持FIFO或单缓冲模式
  i3c_data_tobus #(.FIFO_TYPE(FIFO_TYPE),.EXT_FIFO(EXT_FIFO),
                   .ENA_TOBUS_FIFO(ENA_TOBUS_FIFO))
    tobus 
    (
    .RSTn             (PRESETn), 
    .CLK              (PCLK), 
    .SCL              (clk_SCL), 
    .SCL_n            (clk_SCL_n),
    .avail_tb_ready   (avail_tb_ready),
    .avail_tb_data    (avail_tb_data),
    .avail_tb_ack     (avail_tb_ack),
    .avail_tb_end     (avail_tb_end),
    .tb_flush         (reg_TbFlush),
    .avail_tb_full    (outp_TxFull),
    .set_tb_urun_nack (set_tb_urun_nack),
    .set_tb_urun      (set_tb_urun),
    .set_tb_term      (set_tb_term),
    .clear_tb_urun_nack(reg_clrGenErr[2]),
    .clear_tb_urun    (reg_clrGenErr[1]),
    .clear_tb_term    (reg_clrGenErr[3]),
    .avail_byte_cnt   (outp_TxCnt),
    .tx_trig          (reg_TxTrig),
    .int_tb           (local_tb_int),
    .tb_data_valid    (d_tb_data_valid), 
    .tb_datab         (d_tb_datab), 
    .tb_end           (d_tb_end), 
    .tb_datab_ack     (d_tb_datab_ack), 
    .tb_urun_nack     (tb_urun_nack), 
    .tb_urun          (tb_urun),
    .tb_term          (tb_term),
    .scan_no_rst      (scan_no_rst)
  );  
  
  // 到总线FIFO接口生成
  generate if (FIFO_TYPE[`FIFO_EXT_b]==0 || EXT_FIFO==0) begin : no_fifo_tb
    assign avail_tb_ready = |regflg_wr_cnt; 
    assign avail_tb_data  = reg_wdata[7:0];
    assign avail_tb_end   = reg_TbEnd;
    assign oxf_tb_start   = 0;
    assign oxf_tb_used    = 0;
  end else begin : fifo_tb
    assign avail_tb_ready = ixf_tb_avail;
    assign avail_tb_data  = ixf_tb_data;
    assign oxf_tb_used    = avail_tb_ack;
    assign avail_tb_end   = ixf_tb_last;
    SYNC_AClr_Seq2 sync_start(.CLK(PCLK), .RSTn(PRESETn), .local_set(ixf_tb_avail),  
                              .async_clear(int_start_seen | int_in_STOP), 
                              .o_pulse(oxf_tb_start));
  end endgenerate

  // IBI FIFO实例化（用于IBI扩展数据）
  generate if (ENA_IBI_MR_HJ[`EV_EXTFIFO_b]) begin : ibi_fifo_inst
    i3c_data_ibi_ext ibi_fifo
    (
      .RSTn             (PRESETn), 
      .CLK              (PCLK), 
      .SCL              (clk_SCL), 
      .SCL_n            (clk_SCL_n),
      .avail_tb_ready   (ibi_wr_fifo[8]),
      .avail_tb_data    (ibi_wr_fifo[7:0]),
      .avail_tb_end     (ibi_wr_fifo[9]),
      .avail_tb_ack     (ibi_wr_ack),
      .tb_flush         (ibi_wr_fifo[10]),
      .avail_byte_cnt   (),
      .tb_data_valid    (ibs_tb_data_valid), 
      .tb_datab         (ibs_tb_datab), 
      .tb_end           (ibs_tb_end), 
      .tb_datab_ack     (ibs_tb_datab_ack), 
      .scan_no_rst      (scan_no_rst)
    );
  end else begin
    assign ibs_tb_data_valid = 1'b0;
    assign ibs_tb_datab      = 8'd0;
    assign ibs_tb_end        = 1'b0;
    assign ibi_wr_ack        = 1'b0;
  end endgenerate
  
  // I3C从设备包装器实例
  // 功能：处理SCL域中的所有从设备功能
  i3c_slave_wrapper #(.ENA_ID48B(ENA_ID48B),.ID_48B(ID_48B),
                      .ID_AS_REGS(ID_AS_REGS),.ID_BCR(ID_BCR),.ID_DCR(ID_DCR),
                      .ENA_SADDR(ENA_SADDR),.SADDR_P(SADDR_P),
                      .ENA_MAPPED(ENA_MAPPED),.MAP_CNT(MAP_CNT), .MAP_I2CID(MAP_I2CID),
                      .MAP_DA_AUTO(MAP_DA_AUTO), .MAP_DA_DAA(MAP_DA_DAA),
                      .ENA_IBI_MR_HJ(ENA_IBI_MR_HJ),
                      .ENA_CCC_HANDLING(ENA_CCC_HANDLING),.RSTACT_CONFIG(RSTACT_CONFIG),
                      .MAX_RDLEN(MAX_RDLEN), .MAX_WRLEN(MAX_WRLEN), .MAX_DS_WR(MAX_DS_WR), 
                      .MAX_DS_RD(MAX_DS_RD), .MAX_DS_RDTURN(MAX_DS_RDTURN),
                      .ENA_HDR(ENA_HDR),.ENA_MASTER(ENA_MASTER),.PIN_MODEL(PIN_MODEL), 
                      .ENA_TIMEC(ENA_TIMEC),.TIMEC_FREQ_ACC(TIMEC_FREQ_ACC),
                      .SLV_DBG_MX(SLV_DBG_MX))
    slave 
    (
    .RSTn             (PRESETn), 
    .pin_SCL_in       (pin_SCL_in), 
    .pin_SCL_out      (pin_SCL_out), 
    .pin_SCL_oena     (pin_SCL_oena), 
    .pin_SDA_in       (pin_SDA_in), 
    .pin_SDA_out      (pin_SDA_out), 
    .pin_SDA_oena     (pin_SDA_oena), 
    .pin_SDA_oena_rise(pin_SDA_oena_rise), 
    .i2c_spike_ok     (i2c_spike_ok),
    .i2c_hs_enabled   (i2c_hs_enabled),
    .slv_enable       (slvena_delay & is_slave & ~hold_engine), 
    .int_start_seen   (int_start_seen), 
    .int_start_err    (int_start_err), 
    .int_da_matched   (int_da_matched), 
    .int_sa_matched   (int_sa_matched), 
    .int_7e_matched   (int_7e_matched), 
    .int_ddr_matched  (int_ddr_matched),
    .int_in_STOP      (int_in_STOP), 
    .int_event_sent   (int_event_sent), 
    .int_event_ack    (int_event_ack),
    .int_in_evproc    (int_in_evproc),
    .int_ccc_handled  (int_ccc_handled),
    .int_ccc          (int_ccc),
    .state_req        (raw_req),
    .opt_state_AS     (raw_ActState),
    .opt_ev_mask      (raw_EvState),
    .opt_TimeC        (raw_TimeC),
    .opt_timec_sync   (raw_timec_sync[12:0]),
    .opt_slvr_reset   (raw_slvr_reset),
    .opt_match_idx    (raw_match_idx),
    .i_rst_slvr_reset (iraw_rst_slvr_reset),
    .clr_slvr_cause   (clr_slvr_cause),
    .state_in_ccc     (state_in_ccc), 
    .state_in_daa     (state_in_daa), 
    .state_in_hdr     (state_in_hdr),
    .state_dir        (),
    .dyn_addr         (raw_DynAddr), 
    .dyn_addr_chg     (dyn_addr_chg),
    .dyn_chg_cause    (raw_DynChgCause),
    .opt_static_addr  ((ENA_SADDR==`SADDR_CONST) ? {SADDR_P[6:0],1'b1} :
                       (ENA_SADDR==`SADDR_CONFIG)? cf_SlvSA :
                       (ENA_SADDR==`SADDR_NET)   ? cf_SlvSA :
                       8'h0),
    .event_pending    (event_pending), 
    .force_sda        (force_sda), 
    .ibi_has_byte     (ibi_has_byte), 
    .opt_ibi_byte     (opt_ibi_byte), 
    .ibi_timec        (ibi_timec),
    .ibi_timec_marks  (ibi_timec_marks),
    .ibi_timec_sel    (ibi_timec_sel),
    .ibi_extdata      (cf_IbiExtData),
    .ibi_mapidx       (cf_IbiMapIdx),
    .ibi_in_extdata   (raw_ibi_in_extdata),
    .timec_oflow      (timec_oflow),
    .timec_oflow_clr  (timec_oflow_clr),
    .oclk_SCL         (clk_SCL),
    .oclk_SCL_n       (clk_SCL_n),
    .tb_data_valid    (s_tb_data_valid), 
    .tb_datab         (s_tb_datab), 
    .tb_end           (s_tb_end),
    .tb_datab_ack     (s_tb_datab_ack), 
    .tb_urun_nack     (tb_urun_nack), 
    .tb_urun          (tb_urun), 
    .tb_term          (tb_term),
    .fb_data_use      (fb_data_use), 
    .fb_datab         (s_fb_datab), 
    .fb_datab_done    (s_fb_datab_done), 
    .fb_datab_err     (fb_datab_err), 
    .fb_orun          (fb_orun), 
    .fb_ddr_errs      (fb_ddr_errs),
    .fb_s0s1_err      (fb_s0s1_err),
    .brk_s0s1_err     (brk_s0s1_err | abt_s0s1_err),
    .rd_abort         (read_abort),
    .fb_hdr_exit      (fb_hdr_exit),
    .obit_cnt         (),
    .ccc_byte         (),
    .willbe_ccc       (),
    .cf_IdInst        (cf_IdInst), 
    .cf_IdRand        (cf_IdRand), 
    .cf_Partno        (cf_Partno), 
    .cf_IdBcr         (cf_IdBcr), 
    .cf_IdDcr         (cf_IdDcr),
    .cf_IdVid         (cf_IdVid),
    .cf_MaxRd         (cf_MaxRd),
    .cf_MaxWr         (cf_MaxWr),
    .cf_RstActTim     (cf_RstActTim),
    .cf_SlvNack       (cf_SlvNack),
    .cf_SdrOK         (1'b1),
    .cf_DdrOK         (cf_DdrOK), 
    .cf_TspOK         (cf_TspOK), 
    .cf_TslOK         (cf_TslOK),
    .cf_TCclk         (cf_TCclk),
    .cf_s0ignore      (cf_s0ignore),
    .cf_SetDA         (cf_SetDA),
    .cf_SetMappedDASA (cf_SetMappedDASA),
    .cf_SetSA10b      (cf_SetSA10b),
    .cf_HdrCmd        (cf_HdrCmd),
    .cf_vgpio         (cf_vgpio),
    .cf_CccMask       (cf_CccMask),
    .cf_MasterAcc     (cf_MasterAcc),
    .map_daa_use      (map_daa_use),
    .map_daa_dcr      (map_daa_dcr),
    .map_daa_pid      (map_daa_pid),
    .opt_MasterAcc    (opt_MasterAcc),
    .raw_vgpio_done   (raw_vgpio_done),
    .raw_vgpio_byte   (raw_vgpio_byte),
    .raw_hdr_newcmd   (scl_hdr_new_cmd),
    .raw_hdr_cmd      (scl_hdr_cmd),
    .raw_setaasa      (raw_setaasa),
    .raw_rstdaa       (raw_rstdaa),
    .raw_daa_ena      (raw_daa_ena),
    .map_sa_idx       (map_sa_idx),
    .map_daa_da       (map_daa_da),
    .i2c_dev_rev      (i2c_dev_rev),
    .i2c_sw_rst       (i2c_sw_rst),
    .sraw_ActMode     (sraw_ActMode),
    .sraw_PendInt     (sraw_PendInt),
    .sraw_StatusRes   (sraw_StatusRes),
    .opt_ChgMaxRd     (flg_maxrd),
    .opt_ChgMaxWr     (flg_maxwr),
    .opt_MaxRdWr      (outp_MaxRW),
    .slv_debug_observ (slv_debug_observ),
    .scan_single_clock(scan_single_clock), 
    .scan_clk         (scan_clk), 
    .scan_no_rst      (scan_no_rst), 
    .scan_no_gates    (scan_no_gates)
  );
  assign raw_timec_sync[13] = clk_SCL;
  assign raw_Request = {raw_req[6:4], raw_req[3]&~raw_req[6],raw_req[2:0]};
  assign raw_matched = int_da_matched | int_sa_matched;

  // CDC控制信号定义
  wire         st_start, st_start_err, st_stop, st_dachg, st_ccchand;
  wire         st_ddrmatch, st_ccc, st_event, st_slvrst;
  wire   [2:0] st_matched;
  wire  [19:8] IntStates;

  // 中断状态分配
  assign IntStates[`IS_START]   = st_start;
  assign IntStates[`IS_MATCHED] = |st_matched[1:0];
  assign IntStates[`IS_STOP]    = st_stop;
  assign IntStates[`IS_RXPEND]  = local_fb_int;
  assign IntStates[`IS_TXPEND]  = local_tb_int;
  assign IntStates[`IS_DACHG]   = st_dachg;
  assign IntStates[`IS_CCC]     = st_ccc;
  assign IntStates[`IS_ERRWARN] = |(outp_GenErr & msk_GenErr) | 
                                  (|(outp_DataErr & msk_DataErr)) |
                                  reg_holdOErr;
  assign IntStates[`IS_DDRMATCH]= st_ddrmatch;
  assign IntStates[`IS_CHANDLED]= st_ccchand;
  assign IntStates[`IS_EVENT]   = st_event;
  assign IntStates[`IS_SLVRST]   = st_slvrst;

  assign outp_IntStates = IntStates;
  assign outp_GenErr    = {1'b0, st_start_err,set_tb_term,set_tb_urun_nack,
                          set_tb_urun,set_fb_orun};
  assign outp_DataErr   = {set_fb_err[3:1],
                           set_fb_err[0]|set_rd_err};

  // SCL到PCLK域的同步模块
  // 功能：同步各种状态和中断信号
  SYNC_2PH_S2C_STATE synch_int_start(.rst_n(PRESETn), .scl(clk_SCL), .clk(PCLK), 
                                     .trig_scl(int_start_seen & (~cf_matchss|(|st_matched[1:0]))), 
                                     .out_clk(st_start), .clear_clk(reg_clrIntStates[`IS_START]));
  
  wire starterr_rst_n = PRESETn & (~fb_hdr_exit | scan_no_rst);
  `Observe(observe_starterr_rst_n, clk_SCL, ~fb_hdr_exit)
  SYNC_2PH_LVLH_S2C_STATE synch_int_start_err(.rst_n(starterr_rst_n), .scl(clk_SCL), .clk(PCLK), 
                                         .trig_scl(int_start_err), 
                                         .out_clk(st_start_err), .clear_clk(reg_clrGenErr[4]));

  SYNC_2PH_S2C_STATE synch_da_match(.rst_n(PRESETn), .scl(clk_SCL), .clk(PCLK), 
                                    .trig_scl(int_da_matched), 
                                    .out_clk(st_matched[0]), .clear_clk(reg_clrIntStates[`IS_MATCHED]));
  SYNC_2PH_S2C_STATE synch_sa_match(.rst_n(PRESETn), .scl(clk_SCL), .clk(PCLK), 
                                    .trig_scl(int_sa_matched), 
                                    .out_clk(st_matched[1]), .clear_clk(reg_clrIntStates[`IS_MATCHED]));
  SYNC_2PH_S2C_STATE synch_7e_match(.rst_n(PRESETn), .scl(clk_SCL), .clk(PCLK), 
                                    .trig_scl(int_7e_matched), 
                                    .out_clk(st_matched[2]), .clear_clk(reg_clrIntStates[`IS_MATCHED]));

  SYNC_2PH_LVLH_S2C_STATE synch_stop(.rst_n(PRESETn), .scl(clk_SCL), .clk(PCLK), 
                                    .trig_scl(int_in_STOP&(~cf_matchss|(|st_matched[1:0]))),
                                    .out_clk(st_stop), .clear_clk(reg_clrIntStates[`IS_STOP]));

  SYNC_2PH_S2C_STATE synch_dachg(.rst_n(PRESETn), .scl(clk_SCL), .clk(PCLK), 
                                      .trig_scl(dyn_addr_chg),  
                                      .out_clk(st_dachg), .clear_clk(reg_clrIntStates[`IS_DACHG]));

  SYNC_2PH_S2C_STATE synch_ddrmatch(.rst_n(PRESETn), .scl(clk_SCL), .clk(PCLK), 
                                    .trig_scl(int_ddr_matched),
                                    .out_clk(st_ddrmatch), .clear_clk(reg_clrIntStates[`IS_DDRMATCH]));

  SYNC_2PH_LVLH_S2C_STATE synch_ccchand(.rst_n(PRESETn), .scl(clk_SCL), .clk(PCLK), 
                                       .trig_scl(int_ccc_handled),
                                       .out_clk(st_ccchand), .clear_clk(reg_clrIntStates[`IS_CHANDLED]));

  SYNC_2PH_LVLH_S2C_STATE synch_ccc(.rst_n(PRESETn), .scl(clk_SCL), .clk(PCLK), 
                                       .trig_scl(int_ccc),
                                       .out_clk(st_ccc), .clear_clk(reg_clrIntStates[`IS_CCC]));

  // 从设备复位中断同步
  generate
    if (RSTACT_CONFIG[`RSTA_ENA_b]) begin : slvrst_irq
      SYNC_2PH_LVLH_S2C_STATE synch_slvrst(.rst_n(PRESETn), .scl(clk_SCL), .clk(PCLK),
                                           .trig_scl(iraw_slvr_irq), .out_clk(st_slvrst),
                                           .clear_clk(reg_clrIntStates[`IS_SLVRST]));
      SYNC_AClr_C2S sync_slvrst_clr(.CLK(PCLK), .RSTn(PRESETn), 
                                    .local_set(reg_clrIntStates[`IS_SLVRST]), 
                                    .async_clear(1'b1), .o_value(clr_slvr_cause));
    end else begin
      assign st_slvrst      = 1'b0;
      assign clr_slvr_cause = 1'b0;
    end
  endgenerate 

  // VGPIO完成同步
  generate
    if (ENA_MAPPED[`MAP_VGPIO_b]) begin : vgpio_sync
      SYNC_2PH_LVLH_S2C_STATE sync_vgpio_done(.rst_n(PRESETn), .scl(clk_SCL), .clk(PCLK),
                                           .trig_scl(raw_vgpio_done), .out_clk(vgpio_done),
                                           .clear_clk(vgpio_done));
    end else begin
      assign vgpio_done = 1'b0;
    end
  endgenerate 

  // 最大长度限制同步
  generate 
    if (ENA_CCC_HANDLING[`ENCCC_MAXES_b]) begin : maxes_sync
      SYNC_Pulse_S2C sync_maxrd(.SCL(clk_SCL), .CLK(PCLK), .RSTn(PRESETn), 
                                .local_set(flg_maxrd), .o_pulse(outpflg_MaxRd));
      SYNC_Pulse_S2C sync_maxwr(.SCL(clk_SCL), .CLK(PCLK), .RSTn(PRESETn), 
                                .local_set(flg_maxwr), .o_pulse(outpflg_MaxWr));
    end else begin
      assign outpflg_MaxRd = 0;
      assign outpflg_MaxWr = 0;
    end
  endgenerate 

  // 切换到主设备同步
  generate
    if (ENA_MASTER) begin : mstacc_sync
      SYNC_S2C sync_to_mast(.rst_n(PRESETn), .clk(PCLK), 
                            .scl_data(opt_MasterAcc&int_in_STOP), 
                            .out_clk(outp_to_master));
    end else begin
      assign outp_to_master = 1'b0;
    end

    // HDR命令同步
    if (ID_AS_REGS[`IDREGS_HDRCMD_b]) begin : hdrcmd_new
      SYNC_Pulse_S2C sync_hdr_new(.SCL(clk_SCL), .CLK(PCLK), .RSTn(PRESETn), 
                                  .local_set(scl_hdr_new_cmd),
                                  .o_pulse(hdr_new_cmd));
      reg [7:0] hdr_cmd_hold;
      always @ (posedge clk_SCL or negedge PRESETn)
        if (!PRESETn)
          hdr_cmd_hold <= 8'd0;
        else if (scl_hdr_new_cmd)
          hdr_cmd_hold <= scl_hdr_cmd; 
      assign raw_hdr_cmd = hdr_cmd_hold;
    end else begin
      assign hdr_new_cmd = 1'b0;
      assign raw_hdr_cmd = 8'd0;
    end

    // MAP SETAASA同步
    if (MAP_DA_AUTO[`MAPDA_AASA_b]) begin : syn_map_setaasa
      SYNC_Pulse_S2C sync_setaasa(.SCL(clk_SCL), .CLK(PCLK), .RSTn(PRESETn), 
                                  .local_set(raw_setaasa), 
                                  .o_pulse(map_setaasa));
    end else begin
      assign map_setaasa = 1'b0;
    end

    // MAP RSTDAA同步
    if (|MAP_DA_AUTO[2:0]) begin : syn_map_rstdaa
      SYNC_Pulse_S2C sync_rstdaa(.SCL(clk_SCL), .CLK(PCLK), .RSTn(PRESETn), 
                                  .local_set(raw_rstdaa), 
                                  .o_pulse(map_rstdaa));
    end else begin
      assign map_rstdaa = 1'b0;
    end

    // MAP DAA使能同步
    if (MAP_DA_AUTO[`MAPDA_DASA_b]|MAP_DA_AUTO[`MAPDA_DAA_b]) begin : map_da_ena
      SYNC_Pulse_S2C sync_mapdaaena(.SCL(clk_SCL), .CLK(PCLK), .RSTn(PRESETn), 
                                  .local_set(raw_daa_ena), 
                                  .o_pulse(map_daa_ena));
    end else begin
      assign map_daa_ena = 1'b0;
    end
  endgenerate

  // 事件管理CDC模块
  generate 
    if (ENA_IBI_MR_HJ != 0) begin : sync_events
      reg              scl_event_sent;
      reg              scl_event_ack;
      reg        [2:0] clk_ev_det_r;
      wire       [1:0] ev_det_as_code, pclk_ev_det_as_code;
      reg              ev_det_chg;
      wire       [7:0] timec_ibi_byte;

      // 事件请求处理
      wire        allow_ev = ((reg_EvPend[1:0]==2'd3) & raw_EvState[27]) |
                             ((reg_EvPend[1:0]==2'd2) & raw_EvState[25]) |
                             ((reg_EvPend[1:0]==2'd1) & raw_EvState[24]);
      assign event_pending = {(reg_EvPend[2]&int_in_STOP&allow_ev), 
                               (reg_EvPend[1:0] & {2{allow_ev}})};

      // 事件发送和确认注册
      wire ev_rst_n = PRESETn & (~clk_ev_det_r[2] | scan_no_rst);
      always @ (posedge clk_SCL or negedge ev_rst_n)
        if (!ev_rst_n) begin
          scl_event_sent <= 1'b0;
          scl_event_ack  <= 1'b0;
        end else if (int_event_sent) begin
          scl_event_sent <= 1'b1;
          scl_event_ack  <= int_event_ack;
        end else if (~scl_event_sent & scl_event_ack)
          scl_event_ack  <= 1'b0;
      
      // 事件详情解码
      assign ev_det_as_code = (scl_event_sent & scl_event_ack)       ? 2'd3 :
                              (scl_event_sent & ~scl_event_ack)      ? 2'd2 :
                              (~scl_event_sent& |event_pending[1:0]) ? 2'd1 :
                                                                       2'd0;

      // 事件详情同步到PCLK域
      SYNC_S2C #(.WIDTH(2)) sync_ev_det_as_code(.rst_n(PRESETn), .clk(PCLK), 
                                        .scl_data(ev_det_as_code), 
                                        .out_clk(pclk_ev_det_as_code));
      always @ (posedge PCLK or negedge PRESETn)
        if (!PRESETn)
          ev_det_chg <= 1'b0;
        else if (|(clk_ev_det_r[1:0] ^ pclk_ev_det_as_code))
          ev_det_chg <= 1'b1;
        else
          ev_det_chg <= 1'b0;

      // PCLK域事件详情注册
      always @ (posedge PCLK or negedge PRESETn)
        if (!PRESETn)
          clk_ev_det_r      <= 3'd0;
        else if (&clk_ev_det_r[1:0]) begin
          if (~clk_ev_det_r[2])
            clk_ev_det_r[2] <= 1'b1;
          else if (reg_EvPend[2])
            clk_ev_det_r    <= 3'd0;
        end else if (reg_EvPend == 3'b100)
          clk_ev_det_r      <= 3'd0;
        else if (ev_det_chg)
          if (~|clk_ev_det_r[1:0])
            clk_ev_det_r    <= {1'b0,pclk_ev_det_as_code};
          else if (pclk_ev_det_as_code[1])
            clk_ev_det_r    <= {1'b0,pclk_ev_det_as_code};
      assign outp_EvDet    = clk_ev_det_r[2:0];

      // 事件中断同步
      SYNC_2PH_LVLH_S2C_STATE synch_event(.rst_n(PRESETn), .scl(clk_SCL), .clk(PCLK), 
                                          .trig_scl(&ev_det_as_code),
                                          .out_clk(st_event), .clear_clk(reg_clrIntStates[`IS_EVENT]));

      // 事件不可取消同步
      SYNC_S2C sync_no_cancel (.rst_n(PRESETn), .clk(PCLK), .scl_data(int_in_evproc), 
                               .out_clk(outp_EvNoCancel));

      // 慢速计数器相关信号
      wire syn_s0s1;
      wire syn_inread;
      wire raw_rd_abort;
      wire sync_gate;
      reg  enabled;
      wire delay_ena, exit_pulse, delay_ena_done;
      
      // S0/S1错误同步
      SYNC_S2C sync_s0s1_err(.rst_n(RSTn), .clk(CLK_SLOW), .scl_data(fb_s0s1_err), 
                             .out_clk(syn_s0s1));
      
      // 从设备使能延迟处理
      always @ (posedge PCLK or negedge PRESETn)
        if (~PRESETn) begin
          enabled <= 1'b0;
        end else if (cf_SlvEna ^ enabled) begin
          if (~cf_Offline | ~cf_SlvEna) begin
            enabled <= cf_SlvEna;
          end else if (delay_ena_done | exit_pulse) begin
            enabled <= 1'b1;
          end
        end
      assign slvena_delay = enabled;
      
      // 离线使能同步
      SYNC_S2C ena_ol_sync(.rst_n(PRESETn), .clk(CLK_SLOW), 
                           .scl_data(cf_SlvEna & (cf_SlvEna^enabled)), 
                           .out_clk(delay_ena));
      
      // HDR退出脉冲处理
      wire exit_rst_n = PRESETn & (~fb_hdr_exit | scan_no_rst);
      reg exit_match;
      always @ (posedge PCLK or negedge exit_rst_n)
        if (!exit_rst_n) 
          exit_match <= 1'b1;
        else if (exit_match)
          exit_match <= 1'b0;
      assign exit_pulse = exit_match;
      
      // 60us超时完成同步
      SYNC_C2S done_60_sync(.rst_n(PRESETn), .scl(PCLK), .clk_data(abt_s0s1_err), 
                            .out_scl(delay_ena_done));
      
      // 读取状态同步
      SYNC_S2C sync_in_read(.rst_n(RSTn), .clk(CLK_SLOW), 
                            .scl_data(raw_DynAddr[0]&raw_req[3]), 
                            .out_clk(syn_inread));
      
      // 读取中止处理
      assign read_abort = ERROR_HANDLING[`ERR_RDABT_b] & raw_rd_abort & raw_req[3]; 
      SYNC_2PH_S2C_STATE synch_fb_err0(.rst_n(RSTn), .scl(CLK_SLOW), .clk(PCLK), 
                                   .trig_scl(ERROR_HANDLING[`ERR_RDABT_b] & raw_rd_abort), 
                                   .out_clk(set_rd_err), .clear_clk(reg_clrDataErr[8]));
      
      // 慢速时钟门控
      wire raw_gate = ~(cf_SlvEna & (cf_SlvEna^enabled)) & ~fb_s0s1_err &
                      ~(raw_DynAddr[0]&raw_req[3]) & 
                      ~(ERROR_HANDLING[`ERR_RDABT_b] & raw_rd_abort);
      assign slow_gate = sync_gate & raw_gate;

      // 慢速计数器模块
      // 功能：处理IBI、MR、S0/S1超时、读取中止和HJ超时计数
      i3c_slow_counters #(.ENA_IBI_MR_HJ(ENA_IBI_MR_HJ),
                            .CLK_SLOW_BITS(CLK_SLOW_BITS),.CLK_SLOW_MATCH(CLK_SLOW_MATCH),
                            .CLK_SLOW_HJMUL(CLK_SLOW_HJMUL),
                            .ENA_TIMEC(ENA_TIMEC),.TIMEC_FREQ_ACC(TIMEC_FREQ_ACC),
                          .ENA_CCC_HANDLING(ENA_CCC_HANDLING))
        counters 
        (
          .RSTn             (RSTn), 
          .clk_SCL_n        (clk_SCL_n), 
          .clk_SCL          (clk_SCL), 
          .CLK_SLOW         (CLK_SLOW),
          .slow_gate        (sync_gate),
          .cf_SlvEna        (cf_SlvEna), 
          .cf_BAMatch       (cf_BAMatch),
          .event_pending    (event_pending),
          .pin_SCL_in       (pin_SCL_in),
          .pin_SDA_in       (pin_SDA_in),
          .run_60           (syn_s0s1 | delay_ena), 
          .run_100          (syn_inread & ERROR_HANDLING[`ERR_RDABT_b]),
          .int_in_STOP      (int_in_STOP), 
          .force_sda        (force_sda),
          .done_60          (abt_s0s1_err),
          .done_100         (raw_rd_abort), 
          .hold_engine      (hold_engine)
        );

      // IBI字节处理
      if (ENA_IBI_MR_HJ[`EV_IBI_DAT_b]) begin : ibi_byte
        assign ibi_has_byte  = 1'b1;
        assign opt_ibi_byte  = ~|ibi_timec_sel[1:0] ? 
                               (reg_EvIbiByte | (ibi_timec?8'h80:8'h00)) : 
                               timec_ibi_byte;
      end else begin
        assign ibi_has_byte  = 1'b0;
        assign opt_ibi_byte  = 8'd0;
      end

      // 时间控制处理
      if (|ENA_TIMEC) begin : timec_handling
        wire                    ev_pend;
        wire                    time_oflow;
        wire                    syn_tc_gate;
        
        SYNC_C2S #(.WIDTH(1)) sync_IBI_pend(.rst_n(RSTn), .scl(CLK_SLOW_TC), 
                                    .clk_data(event_pending[0]), .out_scl(ev_pend));
        SYNC_AClr_C2S sync_tcoflow(.CLK(CLK_SLOW_TC), .RSTn(RSTn), .local_set(time_oflow), 
                                   .async_clear(timec_oflow_clr), .o_value(timec_oflow));
        
        assign tc_slow_gate = syn_tc_gate & ~event_pending[0] & ~timec_oflow;

        // 时间控制模块
        // 功能：处理IBI时间戳和超时控制
        i3c_time_control time_ctrl(
          .RSTn          (RSTn), 
          .CLK_SLOW      (CLK_SLOW_TC), 
          .clk_SCL_n     (clk_SCL_n), 
          .timec_ena     (raw_TimeC),
          .event_start   (ev_pend),
          .was_nacked    (clk_ev_det_r[2:0]==3'b010),
          .sc1_stop      (ibi_timec_marks[0]),
          .sc2_stop      (ibi_timec_marks[1]),
          .time_info_sel (ibi_timec_sel),
          .time_info_byte(timec_ibi_byte),
          .ibi_timec     (ibi_timec),
          .time_overflow (time_oflow),
          .slow_gate     (syn_tc_gate),
          .scan_no_rst   (scan_no_rst));
      end else begin
        assign tc_slow_gate    = 1;
        assign timec_ibi_byte  = 8'd0;
        assign ibi_timec       = 1'b0;
        assign timec_oflow     = 1'b0;
      end

    end else begin // 无IBI、MR、HJ功能
      assign event_pending = 3'd0;
      assign force_sda     = 1'b0;
      assign hold_engine   = 1'b0;
      assign outp_EvDet    = 3'd0;
      assign st_event      = 1'b0;
      assign slow_gate     = 1'b1;
      assign tc_slow_gate  = 1'b1;
      assign ibi_has_byte  = 1'b0;
      assign opt_ibi_byte  = 8'd0;
      assign ibi_timec     = 1'b0;
      assign timec_oflow   = 1'b0;
      assign abt_s0s1_err  = 1'b0;
      assign read_abort    = 1'b0;
      assign set_rd_err    = 1'b0;
      assign slvena_delay  = cf_SlvEna;
    end
  endgenerate

  // 参数检查逻辑
  // 功能：检查参数组合的有效性，生成警告信号
  localparam ID_TST = ~|ENA_ID48B || ENA_ID48B>`ID48B_CONST_NONE || ID_48B==48'd0 ||
                      (ENA_ID48B==`ID48B_CONST_NONE && ~ID_AS_REGS[`IDREGS_VID_b]) ||
                      (ENA_ID48B<=`ID48B_CONST_INST && ID_48B[32]);
  wire [0:0] bad_id;
  assign bad_id[ID_TST] = 1'b0;
  
  localparam IAR_TST = |ID_AS_REGS[11] || 
                       ID_AS_REGS[`IDREGS_INST_b]&(ENA_ID48B!=`ID48B_CONST_INST) ||
                       ID_AS_REGS[`IDREGS_RAND_b]&(ENA_ID48B< `ID48B_CONST_PARTNO) ||
                       ID_AS_REGS[`IDREGS_VID_b] &(ENA_ID48B!=`ID48B_CONST_NONE) ||
                       ID_AS_REGS[`IDREGS_DAWR_b]&|ENA_MASTER ||
                       (ID_AS_REGS[`IDREGS_VGPIO_b]&~ENA_MAPPED[`MAP_VGPIO_b]) ||
                       ID_AS_REGS[`IDREGS_SLVENA_b]&|ENA_MASTER;
  wire [0:0] bad_id_as_reg;
  assign bad_id_as_reg[IAR_TST] = 1'b0;
  
  localparam BCR_TST = ~ID_AS_REGS[`IDREGS_BCR_b] &
                       (ID_BCR[7:6]!=(ENA_MASTER?2'b01:2'b00) ||
                        ID_BCR[1]!=ENA_IBI_MR_HJ[`EV_IBI_b] ||
                        ID_BCR[2]!=ENA_IBI_MR_HJ[`EV_IBI_DAT_b] ||
                        (ENA_IBI_MR_HJ[`EV_IBI_DAT_b]&~ENA_IBI_MR_HJ[`EV_IBI_b]) ||
                        (|ENA_TIMEC&~ENA_IBI_MR_HJ[`EV_IBI_DAT_b]));
  wire [0:0] bad_bcr;
  assign bad_bcr[BCR_TST] = 1'b0;
  
  localparam SA_INV = SADDR_P<3 || SADDR_P>=8'h7E ||
                      SADDR_P==8'h7C || SADDR_P==8'h7A || SADDR_P==8'h76 ||
                      SADDR_P==8'h6E || SADDR_P==8'h5E || SADDR_P==8'h3E;
  localparam SA_TST = ENA_SADDR>`SADDR_CONFIG ||
                      (ENA_SADDR!=`SADDR_CONST && |SADDR_P) ||
                      (ENA_SADDR==`SADDR_CONST && SA_INV);
  wire [0:0] bad_sa;
  assign bad_sa[SA_TST] = 1'b0;
  
  localparam CLK_TST = (|(CLK_SLOW_BITS|CLK_SLOW_MATCH) & ~|ENA_IBI_MR_HJ) ||
                       CLK_SLOW_MATCH>=(1<<CLK_SLOW_BITS);
  wire [0:0] bad_clk;
  assign bad_clk[CLK_TST] = 1'b0;
  
  localparam MAXES   = MAX_RDLEN|MAX_WRLEN;
  localparam ALLMX   = MAXES|MAX_DS_WR|MAX_DS_RD|MAX_DS_RDTURN;
  localparam CCC_TST = ENA_CCC_HANDLING>6'h3F ||
                       MAXES>16'hFFFF ||
                       (~ENA_CCC_HANDLING[`ENCCC_MAXES_b]&|ALLMX) ||
                       (ENA_CCC_HANDLING[`ENCCC_MAXES_b]&
                             (MAX_DS_WR>7||MAX_DS_RD>63||MAX_DS_RDTURN>=(1<<24))) ||
                       (ENA_CCC_HANDLING[`ENCCC_V11MIN]!=RSTACT_CONFIG[`RSTA_ENA_b]);
  wire [0:0] bad_ccc;
  assign bad_ccc[CCC_TST] = 1'b0;
  
  localparam TIMEC_TST = (ENA_TIMEC[2] & ~ENA_TIMEC[1]) |
                         (ENA_TIMEC[0] & ~ENA_TIMEC[1]) | 
                         (|ENA_TIMEC[4:3]);
  wire [0:0] bad_timec;
  assign bad_timec[TIMEC_TST] = 1'b0;
  
  localparam SELBUS_TST = (SEL_BUS_IF>=(1<<5));
  wire [0:0] bad_selbus;
  assign bad_selbus[SELBUS_TST] = 1'b0;
  
  localparam FIFO_TST = FIFO_TYPE>=(1<<4) || &FIFO_TYPE[1:0] ||
                        (FIFO_TYPE[2] & ~FIFO_TYPE[0]) ||
                        (FIFO_TYPE[3] & FIFO_TYPE[0]) ||
                        (~|FIFO_TYPE[1:0] & |(ENA_TOBUS_FIFO|ENA_FROMBUS_FIFO)) ||
                        (FIFO_TYPE[0] & ~|(ENA_TOBUS_FIFO|ENA_FROMBUS_FIFO)) ||
                        (~FIFO_TYPE[`FIFO_EXT_b] & |EXT_FIFO) ||
                        (FIFO_TYPE[`FIFO_EXT_b] & ~|EXT_FIFO) ||
                        EXT_FIFO>2;
  wire [0:0] bad_fifo;
  assign bad_fifo[FIFO_TST] = 1'b0;
  
  localparam PINMODEL_TST = (PIN_MODEL>2) || (PIN_MODEL[1]&|ENA_MASTER);
  wire [0:0] bad_pin_model;
  assign bad_pin_model[PINMODEL_TST] = 1'b0;
  
  localparam HDR_TST = |ENA_HDR[2:1];
  wire [0:0] bad_hdr;
  assign bad_hdr[HDR_TST] = 1'b0;
  
  localparam MAP_TST = (~ENA_MAPPED[0] & |ENA_MAPPED[4:1]) ||
                       (|MAP_CNT[3:1] & ~ENA_MAPPED[0]) ||
                       (MAP_CNT>8) ||
                       (|MAP_I2CID & ~ENA_MAPPED[0]) ||
                       (ENA_MAPPED[`MAP_I2C_SA10_b] & ENA_MAPPED[`MAP_VGPIO_b]) ||
                       (ENA_MAPPED[1] & ~|MAP_CNT) ||
                       (~ENA_MAPPED[0]&|MAP_DA_AUTO) ||
                       ((~MAP_DA_AUTO[2]|MAP_DA_AUTO[6])&|MAP_DA_DAA) ||
                       (~MAP_DA_AUTO[2]&|MAP_DA_AUTO[12:6]) ||
                       (MAP_DA_AUTO[6]&(MAP_DA_AUTO[12:8]>18)) ||
                       (&MAP_DA_AUTO[7:6]&(MAP_DA_AUTO[12:8]>10));
  wire [0:0] bad_map;
  assign bad_map[MAP_TST] = 1'b0;

  // 免费版本参数检查
  `ifndef IS_NOT_FREE
   localparam FREE_TST = |ENA_MAPPED[`MAP_VGPIO_b:`MAP_I2C_SA10_b] ||
                         (|MAP_DA_AUTO[`MAPDA_DAA_DCR_b:`MAPDA_DASA_b]) ||
                         (|ENA_HDR) || (|ENA_TIMEC));
   wire [0:0] not_in_free;
   assign not_in_free[FREE_TST] = 1'b0;
  `endif

endmodule