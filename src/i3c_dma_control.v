// 设计信息
// 文件名称        : i3c_dma_control.v
// 描述            : MIPI I3C DMA 管理模块，用于 APB 接口（从设备和主设备）
//                    此模块包含 DMA 请求/确认处理，以支持 FIFO 的 DMA 加载/排空。
//                    由于缺乏标准，DMA 相当混乱。因此，我们必须分别适应不同类型。

`include "i3c_params.v"                 // 局部参数/常量

module i3c_dma_control #(
  parameter  DMA_TYPE         = 2'd0)   // 0=req/ack，1=trig，2=req/ack with done，3=req/ack with handshake
  (
  input               PRESETn,          // 系统复位
  input               PCLK,             // 寄存器同步的系统时钟
  input         [5:0] dma_ctrl,         // 来自寄存器的设置
  input               rx_trig,
  input               tx_trig,
  input         [1:0] rx_fullness,      // 仅关心是否达到 2 或更多
  input         [1:0] tx_avail,         // 仅关心是否有 2 或更多空间
  input               pread,            // APB 读取发生
  output              dma_req_tb, 
  output              dma_req_fb, 
  input               dma_req_tb_ack, 
  input               dma_req_fb_ack
  );

  // trigger 表示 PCLK 域中的一个脉冲，DMA 知道计数。
  // 否则请求/确认尽可能长（根据满度）
  wire                rx_is_empty, tx_is_full;
  wire                rx_is_last, tx_is_last;

  generate
  if (DMA_TYPE==`DMA_4PHASE) begin : dma_type_ctrl
    // 覆盖情况 1、2、3，但非情况 5
    // 1) 如果有数据要处理，驱动请求触发器。2) 收到确认时，清除请求。
    // 然后回到 1。
    // 注意：我们假设确认->请求=0 周期发生在数据阶段或之后，因此下一个周期的请求检查是安全的。
    reg dma_req_fb_hshake;
    reg dma_req_tb_hshake;
    reg fb_auto_cancel;
    // 注意：自动取消允许最后一个 RX 字节在单帧情况下到达 DMA。但是，
    // 如果 DMA 速度太慢无法跟上，则无法排空 RX FIFO。
    always @ (posedge PCLK or negedge PRESETn)
      if (!PRESETn) begin
        dma_req_fb_hshake <= 1'b0;
        fb_auto_cancel    <= 1'b0;
      end else if (dma_req_fb & ~|dma_ctrl[1:0] & ~fb_auto_cancel)
        dma_req_fb_hshake <= 1'b0;      // DMA 手动取消
      else if (dma_req_fb & dma_req_fb_ack)
        dma_req_fb_hshake <= 1'b0;      // 请求完成
      else if (|dma_ctrl[1:0] & ~dma_req_fb & ~dma_req_fb_ack & 
               ~rx_is_empty) begin      // 无触发，因为 4 相
        dma_req_fb_hshake <= 1'b1;      // 开始请求
        fb_auto_cancel    <= ~dma_ctrl[1];
      end
    always @ (posedge PCLK or negedge PRESETn)
      if (!PRESETn)
        dma_req_tb_hshake <= 1'b0;
      else if (dma_req_tb & ~|dma_ctrl[3:2])
        dma_req_tb_hshake <= 1'b0;      // DMA 取消
      else if (dma_req_tb & dma_req_tb_ack)
        dma_req_tb_hshake <= 1'b0;      // 请求完成
      else if (|dma_ctrl[3:2] & ~dma_req_tb & ~dma_req_tb_ack & 
               ~tx_is_full)             // 无触发，因为 4 相
        dma_req_tb_hshake <= 1'b1;      // 开始请求

     assign dma_req_fb = dma_req_fb_hshake;
     assign dma_req_tb = dma_req_tb_hshake;
  end else if (DMA_TYPE==`DMA_ACK1AFT) begin
    // 覆盖情况 4，但非情况 5 或 6
    // 只要非空/非满，我们就设置请求，每个确认持续 1 个周期。
    // 我们使用带有最后一个的确认来决定是否应取消请求，以便我们可以处理 DMA 中的任何流水线前瞻
    //（但不是激进的前瞻）。
    reg fb_start;
    reg tb_start;
    always @ (posedge PCLK or negedge PRESETn)
      if (!PRESETn) begin
        fb_start <= 1'b0;
      end else if (dma_req_fb & ~|dma_ctrl[1:0])
        fb_start <= 1'b0;               // DMA 取消
      else if (fb_start & dma_req_fb_ack & (rx_is_last | rx_is_empty))
        fb_start <= 1'b0;               // 请求完成
      else if (|dma_ctrl[1:0] & ~fb_start & ~dma_req_fb_ack & 
               (rx_trig | ~rx_is_empty))// 如果使用触发，是否应包含空？
        fb_start <= 1'b1;               // 开始请求
    always @ (posedge PCLK or negedge PRESETn)
      if (!PRESETn)
        tb_start <= 1'b0;
      else if (dma_req_tb & ~|dma_ctrl[3:2])
        tb_start <= 1'b0;               // DMA 取消
      else if (tb_start & dma_req_tb_ack & (tx_is_last | tx_is_full))
        tb_start <= 1'b0;               // 请求完成
      else if (|dma_ctrl[3:2] & ~tb_start & ~dma_req_tb_ack & 
               (tx_trig | ~tx_is_full)) // 如果使用触发，是否需要 is_full？
        tb_start <= 1'b1;               // 开始请求

    // 注意：以下组合逻辑作为保护
    assign dma_req_fb = fb_start & ~rx_is_empty;
    assign dma_req_tb = tb_start & ~tx_is_full;
  end else if (DMA_TYPE==`DMA_ACK1BEF) begin
    // 覆盖情况 4，但非情况 5 或 6
    // 只要非空/非满，我们就设置请求，每个确认持续 1 个周期。
    // 我们使用带有最后一个的确认来决定是否应取消请求，以便我们可以处理 DMA 中的任何流水线前瞻
    //（但不是激进的前瞻）。
    reg fb_start;
    reg tb_start;
    reg fb_hold;
    reg tb_hold;
    always @ (posedge PCLK or negedge PRESETn)
      if (!PRESETn) begin
        fb_start <= 1'b0;
        fb_hold  <= 1'b0;
      end else if (dma_req_fb & ~|dma_ctrl[1:0]) begin
        fb_start <= 1'b0;               // DMA 取消
        fb_hold  <= 1'b0;
      end else if (~fb_start & fb_hold & 
                   (pread | (~rx_is_last & ~rx_is_empty))) begin // 重新准备
        fb_start <= 1'b1;               // 开始请求
        fb_hold  <= 1'b0;
      end else if (fb_start & dma_req_fb_ack & (rx_is_last | rx_is_empty)) begin
        fb_start <= 1'b0;               // 请求完成
        fb_hold  <= 1'b1;               // 保持直到读取或有超过 1 个等待
      end else if (|dma_ctrl[1:0] & ~fb_start & ~dma_req_fb_ack & 
                   ~fb_hold & rx_trig) begin // 再次准备
        fb_start <= 1'b1;               // 开始请求
        fb_hold  <= 1'b0;
      end
    always @ (posedge PCLK or negedge PRESETn)
      if (!PRESETn) begin
        tb_start <= 1'b0;
        tb_hold  <= 1'b0;
      end else if (dma_req_tb & ~|dma_ctrl[3:2]) begin
        tb_start <= 1'b0;               // DMA 取消
        tb_hold  <= 1'b0;
      end else if (~tb_start & tb_hold & ~tx_is_last & 
                   ~tx_is_full) begin   // 当有空间时重新准备
        tb_start <= 1'b1;               // 开始请求
        tb_hold  <= 1'b0;
      end else if (tb_start & dma_req_tb_ack & (tx_is_last | tx_is_full)) begin
        tb_start <= 1'b0;               // 请求完成
        tb_hold  <= 1'b1;               // 保持直到有空间
      end else if (|dma_ctrl[3:2] & ~tb_start & ~dma_req_tb_ack & 
                   ~tb_hold & tx_trig) begin
        tb_start <= 1'b1;               // 开始请求
        tb_hold  <= 1'b0;
      end

    // 注意：以下组合逻辑作为保护
    assign dma_req_fb = fb_start & ~rx_is_empty;
    assign dma_req_tb = tb_start & ~tx_is_full;
  end
  endgenerate

  // 空和最后测试需要知道我们是以 2 字节还是 1 字节移动
  // 因此检查 dma_ctrl[5]（表示半字）。注意：空和满可能还有 1 字节剩余（等待或可用）
  assign rx_is_empty = dma_ctrl[5] ? ~rx_fullness[1] : ~|rx_fullness;
  assign tx_is_full  = dma_ctrl[5] ? ~tx_avail[1]    : ~|tx_avail;
  assign rx_is_last  = dma_ctrl[5] ? (rx_fullness==2'd2) : (rx_fullness==2'd1);
  assign tx_is_last  = dma_ctrl[5] ? (tx_avail==2'd2)    : (tx_avail==2'd1);

endmodule