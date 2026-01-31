/*--------------------------------------------------------------------
  此模块用于当 PIN_MODEL == `PINM_EXT_REG 时，将 SDA 输出控制信号
  （输出使能与输出值）从从机外设中分离，并移至靠近焊盘的位置。
  此机制用于解决路径时序问题。该模块接收组合逻辑 D 输入（在 SCL 边沿之前建立），
  通过将 pin_SCL_in 从附近的焊盘引入，从而闭合时钟输入到数据输出的环路。
  因此，SCL_in 到 SDA_out 是一条短路径，而驱动 SDA_out 选择的 D 输入
  来自较远的外设逻辑。
  -------------------------------------------------------------------- */

module i3c_pinm_reg_ext #(
  parameter  ENA_DDR   = 0             // 使能 HDR-DDR 模式
  )(
  // 系统或模块复位
  input               RSTn,             // 主复位
  // 来自焊盘的 SCL 信号，将在本地转换为时钟及其反相时钟
  input               i_pad_SCL,        // 直接来自焊盘
  // 来自 I3C 从机外设的信号
  input               pin_SDA_out,      // 外设输出的 SDA 值
  input               pin_SDA_oena,     // 下降沿输出使能（来自外设）
  input               pin_SDA_oena_rise0, // 上升沿输出使能（来自外设）
  input               pin_SDA_oena_rise1, // 强制起始条件
  input         [1:0] pin_SDA_oena_rise32, // HDR-DDR 模式下的上升沿输出使能（仅 HDR-DDR 使用）
  // 输出到 SDA 焊盘
  output              o_pad_SDA_oe,     // SDA 输出使能
  output              o_pad_SDA_out,    // 输出到 SDA（当 oe=1 时）
  // 扫描测试信号
  input               scan_single_clock, // 扫描模式下使用非反相的 scan_CLK
  input               scan_CLK
  );

  wire clk_SCL, clk_SCL_n;

  // 生成 SCL 时钟及其反相时钟，使用时钟安全选择模块
  // 可简化为更简单的形式，因为只需要下降沿
  CLOCK_SOURCE_MUX source_scl  (.use_scan_clock(scan_single_clock), .scan_clock(scan_CLK), 
                                .pin_clock(i_pad_SCL), .clock(clk_SCL));
  CLOCK_SOURCE_INV source_scl_n(.scan_no_inv(scan_single_clock), .good_clock(clk_SCL), 
                                .clock(clk_SCL_n));

  // 寄存预期的焊盘状态。变化前检查是为了降低触发器的翻转功耗
  // 以下两个寄存器异或后形成焊盘输出使能
  reg   SDA_oena_r;
  always @ (posedge clk_SCL_n or negedge RSTn) 
    if (!RSTn)
      SDA_oena_r <= 1'b0;
    else if (pin_SDA_oena != SDA_oena_r) // 使能检查非必需
      SDA_oena_r <= ~SDA_oena_r;

  // 寄存输出值（当 OE=1 时有效）。仅在下降沿变化，通常保持为 0（开漏）
  reg SDA_out_r;
  always @ (posedge clk_SCL_n or negedge RSTn) 
    if (!RSTn)
      SDA_out_r <= 1'b0;
    else if (pin_SDA_out != SDA_out_r)  // 使能检查非必需
      SDA_out_r <= ~SDA_out_r;

  // 输出使能逻辑：OE = XOR(SDA_oena_r, AND(rise_chg, SCL)) 的变体
  assign o_pad_SDA_oe   = pin_SDA_oena_rise1 |
                           (SDA_oena_r ^ (pin_SDA_oena_rise0 & i_pad_SCL));
  
  // HDR-DDR 模式下的异或项，非 HDR-DDR 时会被优化掉
  wire hdr_xor = (pin_SDA_oena_rise1 & SDA_out_r) |
                 (ENA_DDR & pin_SDA_oena_rise32[1] & i_pad_SCL &
                  (pin_SDA_oena_rise32[0] != SDA_out_r));
  
  assign o_pad_SDA_out  = SDA_out_r ^ hdr_xor;
  
endmodule