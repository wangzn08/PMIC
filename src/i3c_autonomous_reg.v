// 设计信息
// 文件名称        : i3c_autonomous_reg.v
// 描述            : MIPI I3C 消息处理到寄存器的自动生成寄存器模块
//                    此模块包含自动生成的寄存器，以支持希望具有标准 i2c/i3c "寄存器" 接口的
//                    自主从设备模型。即，i2c/i3c 的第一个写入数据是进入此寄存器组的索引，
//                    然后主设备可以写入一个或多个寄存器，或切换到 i2c/i3c 读取操作来读取
//                    一个或多个寄存器。参数完全控制这些寄存器的构建，并支持连续和非连续的
//                    断点，以支持所有正常模型。
//                    系统中的状态机会被通知这些更改，以便在之后采取行动。

`include "i3c_params.v"                 // 局部参数/常量

module i3c_autonomous_reg #(
      // 规则：[0]=仅在RUN[idx]=0时可写，[1]=在RUN[idx]=0时忽略写入，
      //       [2]=仅在RUN[idx]时触碰，[3]=写操作时触碰（非读）
      //       [4]=在S（非Sr）时重置索引，[5]=Magic寄存器（总线）
      //       [7:6]=DDR规则：0:CMD=索引，1:CMD=索引（除7F外），2:忽略CMD
      //             （参见 REG_DDRCMD_WRIDX，当[7:6]=2时。仅设置索引CMD值）
      //       [8]=从无效寄存器读取时返回NACK，[15:9]=保留
    parameter         REG_RULES    = 16'd0,   // 上述规则
    parameter         MAX_REG      = 8'h00,   // 最后一个索引，范围1至255
      // 以下是设置位
    parameter         REG_WRITABLE = {MAX_REG+1{1'b0}}, // 可写寄存器
    parameter         REG_READABLE = {MAX_REG+1{1'b0}}, // 可读寄存器
    parameter         REG_RUN      = {MAX_REG+1{1'b0}}, // 不连续边界
    parameter         REG_MASK     = {((MAX_REG+1)*8){1'b1}}, // 全位或部分位掩码
    parameter         REG_BLEND    = {MAX_REG+1{1'b0}}, // 混合写/读
    parameter         REG_REMAP    = {((MAX_REG+1)*8){1'b1}}, // 可选重映射
    parameter         REG_RESET    = {((MAX_REG+1)*8){1'b0}}, // 写触发器的复位值
    parameter         REG_DDRCMD_WRIDX=8'h00,// 仅当 REG_RULES[7:6]=2 时使用
    parameter         MAGIC_RULES  = 4'b0000,// 规则位。[0]=RW vs. WO，[1]=CCCs，[2]=无索引
    parameter         MAGIC_MASK   = 8'h00,  // 匹配掩码
    parameter         MAGIC_MATCH  = 8'h00,  // 通过掩码匹配（索引&掩码）== 匹配值
      // 以下为计算值 - 不传入。如果是 CCC，我们总是需要一个字节
    parameter         IDXMX        = MAGIC_RULES[1]?7 :
                                     (MAX_REG<4)?1:(MAX_REG<8)?2:(MAX_REG<16)?3:
                                     (MAX_REG<32)?4:(MAX_REG<64)?5:(MAX_REG<128)?6:7
  )
  (
  // 我们仅在 SCL 时钟域中操作
  input               clk_SCL_n,        // SCL 下降沿：开始和发送数据
  input               clk_SCL,          // SCL 上升沿：采样数据和读取 T 位
  input               CLK,              // 用于同步的系统时钟
  input               RSTn,             // 主复位
  // 与总线之间交换的数据字节
  input               bus_new1,         // 到我们的新事务
  input               bus_read,         // 是读取操作 - 与 bus_new 同时
  input               bus_ddr,          // new1 是 DDR 模式 vs. SDR/i2c
  input               bus_in_ddr,       // 模式为 HDR-DDR
  input               bus_done1,        // 事务结束（STOP 或重复 START）
  input               bus_in_STOP,      // 总线停止状态（无 SCL 时钟），用于 bus_done 情况
  input               fb_new1,          // 来自总线的数据到达
  input         [7:0] fb_data,          // fb_new1 时的数据
  input               fb_is_ccc,        // 为 1 表示 fb_new1 与 CCC 相关（仅在启用时）
  output              fb_err,           // 无效写入尝试
  input               tb_need1,         // 需要下一个字节用于读取
  output        [7:0] tb_data,          // 我们返回的数据
  output              tb_end,           // 读取的最后一个字节（写入时忽略）
  output              tb_err,           // 读取下溢错误
  output              tx_valid,         // 用于在读取不 OK 时 NACK
  // 现在是将要传输到系统 CLK 域（原始和同步）的向量
  output              osync_started,    // 当新事务开始时脉冲 1
  output              o_read,           // 为 1 表示开始的事务是从我们的读取（保持）
  output              osync_done,       // 当读写（对我们）结束时脉冲 1
  output              osync_any_touch,  // 当 osync_done=1 时，若有任何寄存器被读写则脉冲 1
  output  [MAX_REG:0] oraw_reg_touch,   // 若寄存器被读写，则对应位为 1（OK 时清除）
  input   [MAX_REG:0] iraw_touch_OK,    // 系统接受来自 CLK 域的触碰：规则为脉冲/保持
  output [(8*MAX_REG)+7:0] wo_regs,     // 主设备写入的寄存器 - 原始值
  input  [(8*MAX_REG)+7:0] ro_regs,     // 系统提供的供主设备读取的寄存器 - 原始值
  // 现在 Magic 寄存器，意味着映射到使用基于 SCL 的总线的系统中。若不使用，
  // 这些信号不被使用。
  // 注意：magic 和普通寄存器的参数必须匹配（如果两者都有，则 magic 寄存器不占用普通寄存器位置，
  // 只有当 magic 为只写时，才标记为可读）。
  output        [7:0] mr_idx,           // 当前寄存器索引（自动递增）
  output        [7:0] mr_next_idx,      // 事务结束后的下一个索引
  output              mr_dir,           // 0 表示写入，1 表示读取（注意时序）
  output              mr_trans_w_done,  // 脉冲一个周期；在 wdata 结束时（非 idx）
  output              mr_trans_r_done,  // 脉冲一个周期；在 rdata 结束时
  output              mr_trans_ddrcmd,  // 来自 DDR 的 CMD
  output              mr_trans_ccc_done,// 如果启用：与 _w_done 相同，脉冲一个周期
  output        [7:0] mr_wdata,         // 当 mr_dir=0 且 trans_w_done=1 时的数据
  input         [7:0] mr_rdata,         // 来自系统的实时数据 - 在 trans_r_done 时更改
  input               mr_rdata_none,    // 为 1 表示无读取数据 - 如果是第一个字节将返回 NACK
  input               mr_rdata_end,     // 读取时为 1 表示最后一个字节
  input               scan_no_rst       // 防止分层复位
  );
  // 我们在写入时将第一个字节作为索引加载
  reg       [IDXMX:0] fbus_index;
  wire                index_magic;      // 为 1 表示 magic 匹配
  wire                ccc_magic;        // 通过 mr 总线处理的未处理 CCC
  wire                bnew_magic;       // 用于 DDR 的 bus new 传递
  wire      [IDXMX:0] new_index;
  wire      [IDXMX:0] map_index, remap_index;
  reg                 fbus_wr;
  wire                use_ddr;          // 特殊 CMD 抑制
  wire                index_only;
  wire      [IDXMX:0] next_index;
  reg                 sys_read;
  reg                 wr_none;
  wire          [7:0] tobus_data[0:MAX_REG]; // RO 寄存器（系统拥有）+ W/R
  wire    [MAX_REG+1:0] is_valid;
  wire    [MAX_REG+1:0] is_end;
  wire    [MAX_REG:0] is_writable;
  wire    [MAX_REG:0] is_readable;
  wire                s_not_sr; // 仅在 S 时重置索引使用
  localparam ALL_MAGIC = REG_RULES[5] & ~|MAGIC_MASK & ~|MAGIC_MATCH; // 所有 magic 寄存器
  
  //
  // 构建以正常方式处理的寄存器以及空位
  //

  // 首先构建数据池
  // 注意：除非有意义（例如可读回），否则不允许 MAGIC 寄存器与其他寄存器重叠
  genvar i;
    // 使用 `define 因为某些 lint 工具不接受 generate 中的 localparam
  `define UPPER (((i+1)*8)-1)
  generate 
    // 我们将寄存器创建为触发器，但未使用的将被优化掉
    // 注意：我们有 4 种索引寄存器类型：
    // - RO：仅可读，由系统控制
    // - WR：可由主设备写入，因此可读回
    // - 混合：部分位可写，部分位可读，使用掩码
    // - 两阶段：同时具有 RO 和 WO。主设备写入 WO，读取 RO
    if (ALL_MAGIC) begin : magic
      // 将所有内部信号绑定为 0
      assign wo_regs     = 0; 
      assign is_writable = 0;
      assign is_readable = 0;
      assign is_valid    = 0;
      assign is_end      = 0;
      for (i = 0; i <= MAX_REG; i = i + 1) begin : init_unused
        assign tobus_data[i] = 0;
      end
      // 将外部信号绑定为 0
      assign oraw_reg_touch  = 0;
      assign osync_any_touch = 0;
    end else begin
      for (i = 0; i <= MAX_REG; i = i + 1) begin : auto_regs
        if (REG_WRITABLE[i]) begin
          // 可写，因此创建触发器
          reg [7:0] wreg;                 // 注意：如果被掩码，触发器将被移除
          always @ (posedge clk_SCL or negedge RSTn)
            if (~RSTn) 
              wreg <= REG_RESET[`UPPER -: 8];
            else if (fb_new1 & ~wr_none & fbus_wr & (fbus_index==i))
              wreg <= fb_data & REG_MASK[`UPPER -: 8]; // 仅在被掩码的位置
          assign wo_regs[`UPPER -: 8] = wreg & REG_MASK[`UPPER -: 8];
          if (REG_READABLE[i])
            // 同时具有 WO 和 RO - 两个寄存器或混合
            assign tobus_data[i] = ro_regs[`UPPER -: 8] &
                                   (REG_BLEND[i] ? ~REG_MASK[`UPPER -: 8] :
                                                   REG_MASK[`UPPER -: 8]);
          else
            // 仅可写，因此读取上次写入的值
            assign tobus_data[i] = wreg & REG_MASK[`UPPER -: 8]; 
        end else if (REG_READABLE[i]) begin
          // 仅可读 - 由系统提供
          assign wo_regs[`UPPER -: 8] = 8'd0; // 主设备无法写入
          assign tobus_data[i]       = ro_regs[`UPPER -: 8] & REG_MASK[`UPPER -: 8]; 
        end else begin
          // 无寄存器 - 不可读也不可写
          assign wo_regs[`UPPER -: 8] = 8'd0; 
          assign tobus_data[i]       = 8'd0;
        end
        // 现在关于运行和结束
        assign is_writable[i] = REG_WRITABLE[i] ? 1'b1 : 1'b0;
        assign is_readable[i] = REG_READABLE[i] ? 1'b1 : 1'b0;
        assign is_valid[i]    = is_writable[i] | is_readable[i];
        assign is_end[i]      = REG_RUN[i] ? 1'b0 : 1'b1;
        // 如果希望，我们标记所有被写入和读取的寄存器
        if (~REG_RULES[2] | ~REG_RUN[i]) begin : touch_bit
          // 寄存器规则[2] 表示仅在 RUN=0 的点触碰
          wire local_set = (fb_new1 & ~wr_none & fbus_wr & REG_WRITABLE[i]) |
                           (tb_need1 & REG_READABLE[i] & ~REG_RULES[3]);
                            // 如果未禁止，读取时触碰
          reg touchbit;
          // 我们异步重置触碰位，因为它来自不同的时钟域。
          // 释放时无风险，因为 D 输入不会在此时改变（按规则）
          wire async_rstn = RSTn & (~iraw_touch_OK[i] | scan_no_rst);

          always @ (posedge clk_SCL or negedge async_rstn)
            if (!async_rstn)
              touchbit <= 1'b0;
            else if (local_set & (fbus_index==i))
              touchbit <= 1'b1;   // 通过来自 touch_OK 的异步复位清除
          assign oraw_reg_touch[i] = touchbit;
          /// 不需要：SYNC_AClr_S2C sync_touch(clk_SCL, async_rstn, local_set, 1'b0, oraw_reg_touch[i]);
        end else 
          assign oraw_reg_touch[i] = 1'b0;
      end // 循环
      assign is_end[MAX_REG+1]   = 1'b1;    // 因为使用 next_ 检查
      assign is_valid[MAX_REG+1] = 1'b0;  // 因为使用 next_ 检查
      assign osync_any_touch     = |oraw_reg_touch & osync_done;// 现在有任何位为 1
    end // 非 magic
  endgenerate

  // 
  // 索引和可选的 CCC 处理（magic）
  //

  // 现在处理写入时的索引（第一个写入值）以及推进索引的读写请求
  //（实际的写操作在上面）
  always @ (posedge clk_SCL or negedge RSTn)
    if (!RSTn) begin
      fbus_index        <= 0;
      fbus_wr           <= 1'b0;
    end else if (bus_new1) begin
      if (fb_new1 & use_ddr) begin
        // DDR 的特殊情况，新匹配和数据在同一周期
        fbus_wr         <= 1'b1;
        fbus_index      <= remap_index[IDXMX:0]; // 写入的第一个是索引
      end else
        fbus_wr         <= 1'b0;
    end else if (fb_new1) begin
      if (~fbus_wr) begin         // 第一个写入总是索引
        fbus_wr         <= 1'b1;
        fbus_index      <= fb_is_ccc ? fb_data : remap_index[IDXMX:0]; // 写入的第一个是索引
      end else if (~is_end[next_index] & is_writable[next_index] & ~fb_is_ccc & ~index_only)
        fbus_index      <= next_index;
      else if (index_magic & ~fb_is_ccc & ~index_only)
        fbus_index      <= next_index;
    end else if (tb_need1) begin
      // 读取将从上次的索引开始
      if (~is_end[next_index] & is_valid[next_index] & ~fb_is_ccc)
        fbus_index      <= next_index;
      else if (index_magic & ~fb_is_ccc & MAGIC_RULES[0])
        fbus_index      <= next_index;
    end else if (bus_done1) begin
      if (REG_RULES[4] & s_not_sr)// 是 START 而非重复 START
        fbus_index      <= 9'd0; // 在 START 时重置索引，以便后续读取
      fbus_wr           <= 1'b0; // 开始清除写标志
    end
    
  // 注意：如果无 DDR 支持，以下逻辑将被优化掉
  wire [7:0] ddr_index  = {1'b0,fb_data[6:0]}; // 注意：不再有保留的 CMDs
  assign use_ddr        = ~|REG_RULES[7:6] | ((REG_RULES[7:6]==2'b01) & ~&fb_data[6:0]);
  wire [IDXMX:0] fb_idx = (bus_ddr&use_ddr) ? ddr_index[IDXMX:0] : fb_data[IDXMX:0]; 
  assign new_index      = (fb_idx>MAX_REG) ? 0 : fb_idx;
  wire [7:0] rm_idx     = REG_REMAP[({new_index[IDXMX:0],3'd7}) -: 8];
  assign map_index      = rm_idx[IDXMX:0];// 写入的第一个是索引
  assign remap_index    = &map_index ? new_index : map_index;
  assign tb_end         = (index_magic|ccc_magic) ? (mr_rdata_end|mr_rdata_none) : // 如果无 magic 寄存器则折叠
                          is_end[next_index];// 读取时有效
  assign tb_err         = tb_need1 & 
                          ((index_magic|ccc_magic) ? mr_rdata_none : ~is_valid[fbus_index]);
  assign next_index     = fbus_index+{{IDXMX{1'b0}},1'b1};
  // tx_valid 在 MR 时取决于 "none"，在寄存器时取决于有效性，如果 REG_RULES[8]=1
  assign tx_valid       = (index_magic|ccc_magic) ? ~mr_rdata_none : 
                          (~REG_RULES[8] | is_valid[fbus_index]); 
  // 抑制有助于在 to-bus 上获得可靠结果
  assign tb_data        = (index_magic|ccc_magic) ? mr_rdata : // 如果无 magic 寄存器则折叠
                          is_valid[fbus_index] ? tobus_data[fbus_index] : 8'hFF;
  wire   index_over;
  wire   [7:0] full_idx = bus_ddr ? ddr_index : fb_data; 
  generate if (IDXMX < 7) begin : index_check
    assign index_over   = |full_idx[7:(IDXMX+1)];
  end else begin
    assign index_over   = 1'b0;
  end endgenerate
  // wr_none 记住是否在允许的写入范围内
  always @ (posedge clk_SCL or negedge RSTn)
    if (!RSTn)
      wr_none   <= 1'b0;
    else if (bus_new1 & ~fb_new1) 
      wr_none   <= 1'b0;
    else if (fb_new1 & ~fb_is_ccc) begin
      if (bus_new1 & ~use_ddr) 
        wr_none <= 1'b0;
      else if (REG_RULES[0] & ~fbus_wr & ~is_end[fb_data])
        wr_none <= 1'b1;                // 无效开始（仅在运行开始时）
      else if (~fbus_wr & ((full_idx[IDXMX:0]>MAX_REG) | index_over))
        wr_none <= 1'b1;                // 无效索引 - 超出允许范围
      else if (~REG_RULES[1] & fbus_wr & is_end[next_index] & ~index_magic)
        wr_none <= 1'b1;                // 运行结束
      else if ((REG_RULES[7:6]==2'b10) & index_only)
        wr_none <= 1'b1;                // 仅在此写入索引
    end
  assign fb_err     = fb_new1 & wr_none;// 无效写入

  generate if (REG_RULES[7:6]==2'b10) begin : cmd_not_idx
    reg  idx_only_r;
    always @ (posedge clk_SCL or negedge RSTn)
      if (!RSTn)
        idx_only_r <= 1'b0;
      else if (bus_new1 & fb_new1)
        idx_only_r <= remap_index[IDXMX:0] == REG_DDRCMD_WRIDX;
      else if (bus_new1)
        idx_only_r <= 1'b0;
    assign index_only = idx_only_r;
  end else begin
    assign index_only = 1'b0;
  end endgenerate

  //
  // Magic 寄存器 - 读取侧已在上面处理
  //
  assign mr_idx           = fbus_index;
  assign mr_next_idx      = fb_is_ccc ? fbus_index : next_index;
  assign mr_dir           = bus_read; 
  assign mr_trans_w_done  = index_magic & fb_new1 & ~fb_is_ccc & ~(bnew_magic|bus_new1);
  assign mr_trans_r_done  = index_magic & tb_need1 & ~fb_is_ccc;
  assign mr_trans_ddrcmd  = index_magic & fb_new1 & ~fb_is_ccc & (bus_new1|bnew_magic);
  assign mr_trans_ccc_done= (ccc_magic|MAGIC_RULES[2]) & (fb_new1 | tb_need1) & fb_is_ccc;
  assign mr_wdata         = fb_data;
  generate if (REG_RULES[5]) begin : magic_matching
    if (~MAGIC_RULES[2]) begin : mr_no_idx
      reg is_magic1, is_magic2;
      always @ (posedge clk_SCL or negedge RSTn)
        if (!RSTn) 
          is_magic1 <= 1'b0;
        else if (fb_is_ccc & is_magic2)
          is_magic1 <= 1'b0;
        else if (bus_new1) begin
          // DDR 同时使用两者，否则如果不是读取，清除为下一个写入准备
          if (fb_new1 & use_ddr)
            is_magic1 <= 1'b1;
          else if (~bus_read)
            is_magic1 <= 1'b0;
        end else if (fb_new1) begin
          if (~fbus_wr) begin             // 正常写入索引
            // 注意：不考虑重映射 - 直接查看索引本身
            if (~bus_read | MAGIC_RULES[0])
              is_magic1 <= (full_idx & MAGIC_MASK) == MAGIC_MATCH;
          end
        end else if (bus_done1)
          if (REG_RULES[4] & s_not_sr & // 是 START 而非重复 START
                 ((MAGIC_MASK&8'd0) != MAGIC_MATCH))   // 将重置为 0 索引
            is_magic1 <= 1'b0;

      // 我们希望 SCL 下降沿时改变
      always @ (posedge clk_SCL or negedge RSTn)
        if (!RSTn) 
          is_magic2 <= 1'b0;
        else if (fb_is_ccc & is_magic2)
          is_magic2 <= 1'b0;              // CCC 不使用此路径
        else if (~fb_is_ccc & (is_magic1 ^ is_magic2))
          is_magic2 <= is_magic1;         // 正常延迟
        else if (~fb_is_ccc & fb_new1 & ~fbus_wr & bus_in_ddr)
          is_magic2 <= 1'b1;              // DDR 使用字节作为索引

      assign index_magic = is_magic2;
    end else begin
      assign index_magic = 1'b1;          // 无索引，因此发出所有写入
    end

    // CCC 是特殊情况，允许直接以及广播
    // 我们仅注册以便跳过 CCC 代码字节
    reg is_ccc2; // 如果不使用，后者将折叠
    always @ (posedge clk_SCL or negedge RSTn)
      if (!RSTn) 
        is_ccc2   <= 1'b0;
      else if (MAGIC_RULES[1] & fb_is_ccc & fbus_wr)
        is_ccc2   <= 1'b1;
      else
        is_ccc2   <= 1'b0;
    
    // Bus new 被注册，以便我们可以延迟半个周期脉冲
    reg is_bnew;
    always @ (posedge clk_SCL_n or negedge RSTn)
      if (!RSTn) 
        is_bnew <= 1'b0;
      else if (bus_new1)
        is_bnew <= 1'b1;
      else
        is_bnew <= 1'b0;
    assign ccc_magic   = is_ccc2;
    assign bnew_magic  = is_bnew;
  end else begin
    assign index_magic = 1'b0;          // 无 magic 寄存器 
    assign ccc_magic   = 1'b0;
    assign bnew_magic  = 1'b0;
  end
  endgenerate 

  //
  // 现在系统通知 - 通常是 CDC
  //
  // 我们必须指示开始 - 4 相握手
  SYNC_ASelfClr_S2C_Seq2 sync_start(.SCL(clk_SCL), .CLK(CLK), .RSTn(RSTn), 
                                    .local_set(bus_new1), .o_pulse(osync_started));
  always @ (posedge clk_SCL or negedge RSTn)
    if (!RSTn) 
      sys_read  <= 1'b0;
    else if (bus_new1) 
      sys_read  <= bus_read;
  assign o_read    = sys_read;

  reg in_trans;
  always @ (posedge clk_SCL or negedge RSTn)
    if (!RSTn) 
      in_trans  <= 1'b0;
    else if (bus_new1)
      in_trans  <= 1'b1;
    else if (bus_done1)
      in_trans  <= 1'b0;
  // 我们必须指示完成 - 4 相握手
  wire sys_done;
  SYNC_ASelfClr_LVL_S2C_Seq2 sync_done(.SCL(clk_SCL), .CLK(CLK), .RSTn(RSTn), 
                                       .local_set(bus_done1), .local_hold(bus_in_STOP), 
                                       .o_pulse(sys_done));
  assign osync_done = sys_done & in_trans;

  generate if (REG_RULES[4]) begin : detect_s
    // STOP 重置，因此我们忽略任何其他开始
    wire    rst_s_n = RSTn & ~bus_in_STOP;
    reg     is_s;
    always @ (posedge clk_SCL or negedge rst_s_n)
      if (~rst_s_n)
        is_s <= 1'b1;                   // 默认是第一个
      else if (bus_done1)
        is_s <= 1'b0;
    assign s_not_sr = is_s;
  end else
    assign s_not_sr = 1'b0;
  endgenerate

endmodule