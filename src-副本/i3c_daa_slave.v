// 设计信息
// 文件名称        : i3c_daa_slave.v
// 描述            : MIPI I3C 动态地址（CCC）处理模块
//                    此模块处理 ENTDAA 的 ID/BCD/DCR，支持从设备获取其 I3C 动态地址
//                    （当未通过 SETDASA 完成时）。
//                    它逐位输出数据，并读取主设备分配的动态地址。它与 i3c_ccc_slave
//                    模块协同处理其他与动态地址相关的 CCCs，包括 RSTDAA、SETNEWDA、SETDASA。

`include "i3c_params.v"                 // 局部参数/常量

module i3c_daa_slave #(
    // 参数：ID 和 CCC 参数不在此处，由独立模块处理
    //       局部参数及其含义如下
    parameter         ENA_ID48B = `ID48B_CONST, // 常量 vs. 寄存器
    parameter         ID_48B    = 48'h0,// 必须由上层填充部分或全部48位ID
    parameter         ID_AS_REGS= 12'd0,// [0]=IDINST,[1]=ISRAND,[2]=IDDCR,[3]=IDBCR,[4]=VID,[5]=DAwr
    parameter         ID_BCR    = 8'd0, // 若非来自寄存器，则由上层填充BCR
    parameter         ID_DCR    = 8'd0, // 若非来自寄存器，则由上层填充DCR
    parameter         ENA_MAPPED= 5'd0, // 是否允许额外动态/静态地址及相关功能
    parameter         MAP_CNT   = 4'd1, // 允许的额外动态/静态地址数量
    parameter         MAP_I2CID = 24'd0,// !=0 表示 I2C 扩展为 DevID
                      //     PID[位置:0],DCR, MMR, 保留 DAA, AASA,DASA,
    parameter         MAP_DA_AUTO={5'd0,1'b0,1'b0,3'd0,1'b0,1'b0,1'b0},
    parameter         MAP_DA_DAA= 0,    // 如果不是 MMR 且 PID/DCR !=0，则为位数组
    parameter         PIN_MODEL = `PINM_COMBO, // 组合引脚使用模式
    // 以下为计算值
    parameter  [7:0]  PID_CNT = MAP_DA_AUTO[`MAPDA_DAA_PID_lb+4:`MAPDA_DAA_PID_lb]
  )
  (
  // 时钟、复位和来自外部的引脚输入
  input               clk_SCL_n,        // SCL 下降沿：开始和发送数据
  input               clk_SCL,          // SCL 上升沿：采样数据和读取 T 位
  input               RSTn,             // 主复位
  input               pin_SDA_in,       // 读取的 SDA 引脚
  input         [7:0] state_in_CCC,     // 引擎定义检测到的 CCC 状态/模式：参见 CF_xxx
  input               daa_active,       // 引擎指示正在处理 DAA；如果失去则变低
  output              daa_act_ok,       // 用于映射用途的 DAA
  output              daa_inp_drv,      // 为 1 表示预期输出
  output              daa_inp_bit,      // 当 daa_inp_drv==1 时的输出位
  output              daa_done,         // 指示 DA 输出并在 ACK/NACK 周期
  output              daa_acknack,      // ENTDAA 新 DA 的 ACK/NACK
  output        [7:0] dyn_addr,         // 一旦设置的动态地址
  output              dyn_addr_chg,     // 当 DA 更改时脉冲一个 SCL 周期
  output        [2:0] dyn_chg_cause,    // 上次更改的原因
  input         [6:0] id64_cnt,         // DAA 时 64 位 ID 以及 DA 和 ACK 的计数器
  output       [63:0] daa_out,          // 为 CCC GET 命令导出
  output              raw_map_daa,      // 如果使用了映射 DAA（参见索引）
  output        [3:0] daa_map_idx,      // 与之关联的索引
  output        [7:1] map_da,           // 映射分配的待使用动态地址
  input               map_chg,          // 映射更改发生
  // 来自 CCC 的更改动态地址请求
  input               set_da,           // 强制更改动态地址的脉冲
  input         [7:1] new_da,           // 要设置的动态地址
  input               set_aasa,         // 特殊 SA 转 DA
  input         [7:0] old_sa,           // SETAASA 使用的 SA
  // 来自寄存器或外部系统的网络，取决于 ID_AS_REGS
  input         [3:0] cf_IdInst,        // 来自寄存器/网络或未使用
  input               cf_IdRand,        // 若使用则来自寄存器
  input        [31:0] cf_Partno,        // 若使用则来自寄存器
  input         [7:0] cf_IdBcr,         // 若使用则来自寄存器
  input         [7:0] cf_IdDcr,         // 若使用则来自寄存器
  input        [14:0] cf_IdVid,         // 若使用则来自寄存器
  input               cf_DdrOK,         // 可或入 BCR
  input         [7:0] cf_SetDA,         // 为主主机设置动态地址
  input   [MAP_CNT-1:0] map_daa_use,      // 哪些映射是自动 DAA
  input [(MAP_CNT*8)-1:0] map_daa_dcr,      // 映射自动 DAA 的 DCRs
  input [(MAP_CNT*PID_CNT)-1:0] map_daa_pid, // 映射自动 DAA 的 PID 部分
  input [(MAP_CNT*10)-1:0] SetMappedDASA,// 映射的 SAs/DAs（若支持）
  input               scan_no_rst       // 防止分层复位
  );

  // 计数器 [2:0]=字节内位（7到0），[5:3]=ID[0:5],BCR,DCR 使用 7到0，[6]=1 表示 ID，0 表示 DA
  reg           [7:0] i3c_addr;         // 动态地址
  reg                 parity;
  wire                parity_matched;
  wire                map_parity;
  reg           [1:0] delay_i3c;
  wire                rst_i3c_n;
  wire         [63:0] daa_out_0;        // 映射 DA=0 时的输出

  // 在 ENTDAA 竞争期间输出，除非失去且在获取新 DA 时
  assign daa_inp_bit    = daa_acknack ? ~parity_matched : daa_out[id64_cnt[5:0]];
  assign daa_inp_drv    = daa_active & state_in_CCC[`CF_DAA_M] &
                            (id64_cnt[6] | daa_acknack);
  assign parity_matched = parity == ^i3c_addr[7:1] ^ 1'b1;
  wire valid_aasa       = set_aasa & |old_sa & ~i3c_addr[0];
    // 注意：只有主主机会设置 cf_SetDA，除非用于保持；两者都不活跃
  assign dyn_addr       = cf_SetDA[0] ? cf_SetDA : i3c_addr;
  assign dyn_addr_chg   = (state_in_CCC[`CF_GRP_b] == `CFG_RSTDAA_US) |
                          set_da | delay_i3c[1] | (set_aasa & |old_sa & ~i3c_addr[0]) |
                          raw_map_daa | map_chg;

  generate if (ID_AS_REGS[`IDREGS_DAWR_b]) begin : set_chg_cause
    reg    [2:0] chg_cause; 
    assign dyn_chg_cause = chg_cause;
    always @ (posedge clk_SCL or negedge RSTn)
      if (~RSTn)
        chg_cause   <= 3'd0;
      else if (raw_map_daa|map_chg)
        chg_cause   <= 3'd4;            // 仅当启用映射自动时
      else if (dyn_addr_chg) begin
        if ((state_in_CCC[`CF_GRP_b] == `CFG_RSTDAA_US))
          chg_cause <= 3'd3;            // 通过 RSTDAA 重置
        else if (set_da | set_aasa)
          chg_cause <= 3'd2;            // SETDASA、SETAASA、SETNEWDA
        else
          chg_cause <= 3'd1;            // ENTDAA
      end     
  end else begin
    assign dyn_chg_cause = 2'b00;
  end endgenerate

  // 对于映射的动态/静态地址使用，我们允许它们通过复位从寄存器接口清除动态地址。
  // 这样可以在 ENTDAA 模式下获取新的动态地址。模型是复制到映射然后复位。
  assign rst_i3c_n = RSTn & (scan_no_rst | ~&cf_SetDA[7:0]);
  `Observe(observe_DA_rst, clk_SCL, ~&cf_SetDA[7:0]) // 可选的 DFT 观察点
  generate if (PIN_MODEL == `PINM_COMBO) begin : rec_da_combo
    // 注意：此逻辑使用 clk_SCL 而非 SCL_n，以便在上升沿读取
    always @ (posedge clk_SCL or negedge rst_i3c_n)
      if (!rst_i3c_n) begin
        i3c_addr                    <= 8'd0;
        parity                      <= 1'b0;
        delay_i3c                   <= 2'b00;
      end else if (state_in_CCC[`CF_GRP_b] == `CFG_RSTDAA_US)
        i3c_addr[0]                 <= 1'b0; // 失去动态地址
      else if (set_da)                  // 通过 CCC 设置（NEWDA/DASA）
        i3c_addr                    <= {new_da, 1'b1};
      else if (|delay_i3c) begin
        // 我们延迟一个周期正式成为 i3c，以便确认它
        i3c_addr[0]                 <= 1'b1;
        delay_i3c                   <= {delay_i3c[0],1'b0}; // 2 个周期清除
      end else if (i3c_addr[0])
        ;                               // 如果已分配则永远不会为我们
      else if (valid_aasa)              // 如果 SA 有效，则从 SA 设置动态地址
        i3c_addr                    <= {old_sa[7:1],1'b1};
      else if (daa_active & ~id64_cnt[6]) begin
        if (id64_cnt[3]) begin
          if (|id64_cnt[2:0])
            i3c_addr[id64_cnt[2:0]] <= pin_SDA_in;
          else
            parity                  <= pin_SDA_in; // DAA 中第 8 位是奇偶校验位
        end else 
          delay_i3c[0]              <= parity_matched;
      end 

    assign daa_done       = ~id64_cnt[6] & (id64_cnt[3:0]==4'd7); // 在 ACK/NACK 时
    assign daa_acknack    = daa_done;
  end else begin : rec_da_reg
    // 注意：此逻辑使用 clk_SCL 而非 SCL_n，以便在上升沿读取
    // 对于寄存器用法，从输出（下降沿）切换到输入（上升沿）需要 1 个周期的间隙，
    // 这与组合逻辑不同，因此我们必须使用不同的数值
    always @ (posedge clk_SCL or negedge rst_i3c_n)
      if (!rst_i3c_n) begin
        i3c_addr                    <= 8'd0;
        parity                      <= 1'b0;
        delay_i3c                   <= 2'b00;
      end else if (state_in_CCC[`CF_GRP_b] == `CFG_RSTDAA_US)
        i3c_addr[0]                 <= 1'b0; // 失去动态地址
      else if (set_da)                  // 通过 CCC 设置（NEWDA/DASA）
        i3c_addr                    <= {new_da, 1'b1};
      else if (|delay_i3c) begin
        // 如果奇偶校验不匹配则产生 S3 错误 - 跳过并重试
        i3c_addr[0]                 <= parity_matched;
        delay_i3c                   <= {delay_i3c[0],1'b0};
      end else if (i3c_addr[0])
        ;                               // 如果已分配则永远不会为我们
      else if (set_aasa & |old_sa)      // 如果 SA 有效，则从 SA 设置动态地址
        i3c_addr                    <= {old_sa[7:1],1'b1};
      else if (daa_active) begin
        if (~id64_cnt[6]) begin // 主设备发送动态地址
          if (id64_cnt[3]) begin
            if (~&id64_cnt[2:0])
              i3c_addr[id64_cnt[2:0]+1]<= pin_SDA_in;
          end else if (&id64_cnt[2:0]) begin
            parity                  <= pin_SDA_in;
            delay_i3c[0]            <= 1'b1;
          end
        end
      end 

    assign daa_done       = ~id64_cnt[6] & (id64_cnt[3:0]==4'd6); // 在 ACK/NACK 时
    assign daa_acknack    = ~id64_cnt[6] & (id64_cnt[3:0]==4'd7); // 寄存器确认
  end endgenerate

  // 现在使用 id64_cnt 提取我们需要的 ID 位
  generate 
    if (ENA_ID48B==`ID48B_CONST) begin : id_all_const
      assign daa_out_0[63:16] = ID_48B;    // 全部为常量
    end
    if (ENA_ID48B==`ID48B_CONST_INST) begin : id_const_but_inst
      assign daa_out_0[63:16] = {ID_48B[47:16],cf_IdInst[3:0],ID_48B[11:0]};
    end
    if (ENA_ID48B==`ID48B_CONST_PARTNO) begin : id_mostly_reg
      assign daa_out_0[63:16] = {ID_48B[47:33],cf_IdRand,cf_Partno[31:0]};
    end
    if (ENA_ID48B==`ID48B_CONST_NONE) begin : id_all_reg
      assign daa_out_0[63:16] = {cf_IdVid[14:0],cf_IdRand,cf_Partno[31:0]};
    end
  endgenerate

  // 现在提取我们需要的 BCR 和 DCR 位
  generate 
    if (ID_AS_REGS[`IDREGS_BCR_b]) begin : bcr_reg
      assign daa_out_0[15:8] = cf_IdBcr[7:0];
    end else begin : bcr_const
      // 注意：我们应该从参数中将特性或入 BCR，还是依赖它们正确设置？
      assign daa_out_0[15:8] = {ID_BCR[7:5]|cf_DdrOK,ID_BCR[4:0]};
    end
    if (ID_AS_REGS[`IDREGS_DCR_b]) begin : dcr_reg
      assign daa_out_0[7:0] = cf_IdDcr[7:0];
    end else begin : dcr_const
      assign daa_out_0[7:0] = ID_DCR[7:0];
    end
  endgenerate 

  generate
    genvar i;
    if (ENA_MAPPED[`MAP_ENA_b] & MAP_DA_AUTO[`MAPDA_DAA_b]) begin : auto_daa
      //FREE_VERSION_CUT - 从免费版本中移除扩展映射
    end else begin
      // 无映射自动 DAA
      assign daa_out     = daa_out_0;
      assign daa_act_ok  = 1'b0;
      assign daa_map_idx = 0;
      assign raw_map_daa = 1'b0;
      assign map_da      = 7'd0;
      assign map_parity  = 1'b0;
    end
  endgenerate

endmodule