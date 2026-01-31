// CLOCK_SOURCE.v
// 模块: CLOCK_SOURCE_MUX 和 CLOCK_SOURCE_INV
// 功能: 时钟源多路选择器和反相器
//       注意：必须用时钟安全（无毛刺）单元替换此模块，用于多路复用和反相（或异或）。
//       PD 工具可以使用约束将输出视为时钟分布和 STA 的时钟。

module CLOCK_SOURCE_MUX(
  input    use_scan_clock,
  input    scan_clock,
  input    pin_clock,
  output   clock);

  assign clock = use_scan_clock ? scan_clock : pin_clock;
endmodule

module CLOCK_SOURCE_INV(
  input    scan_no_inv,
  input    good_clock,
  output   clock);

  assign clock = ~scan_no_inv ^ good_clock;
endmodule