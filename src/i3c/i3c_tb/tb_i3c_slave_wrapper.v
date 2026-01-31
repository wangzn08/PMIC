`timescale 1ns/1ps

module tb_i3c_slave_wrapper;
    // ==========================================
    // 信号定义 [cite: 1-45]
    // ==========================================
    reg        RSTn;
    reg        pin_SCL_in;
    reg        pin_SDA_in;
    wire       pin_SCL_out;
    wire       pin_SCL_oena;
    wire       pin_SDA_out;
    wire       pin_SDA_oena;

    // 基本配置参数
    wire       slv_enable = 1'b1;
    reg  [7:0] opt_static_addr = 8'h22;  // 静态地址 0x22 (7位地址 0x11) [cite: 6]
    reg        cf_SdrOK = 1'b1;
    reg        cf_DdrOK = 1'b0;
    reg        cf_SlvNack = 1'b0;
    reg  [7:0] cf_SetDA = 8'h00;
    reg        cf_s0ignore = 1'b0;

    // 事件相关
    reg  [2:0] event_pending = 3'b000;
    reg        force_sda = 1'b0;
    reg        ibi_has_byte = 1'b0;
    reg  [7:0] opt_ibi_byte = 8'h00;
    reg        ibi_timec = 1'b0;

    // 数据缓冲区接口
    reg        tb_data_valid = 1'b0;
    reg  [7:0] tb_datab = 8'h00;
    reg        tb_end = 1'b0;
    reg        fb_data_use = 1'b1;

    // 内部输出监控信号 [cite: 20-33]
    wire       int_start_seen;
    wire       int_da_matched;
    wire       int_sa_matched;
    wire       int_7e_matched;
    wire       int_in_STOP;
    wire [7:0] dyn_addr;
    wire       dyn_addr_chg;
    wire [2:0] dyn_chg_cause;
    wire [7:0] fb_datab;
    wire       fb_datab_done;
    wire       fb_datab_err;
    wire       tb_datab_ack;

    // 结果对比专用变量
    integer test_count = 0;
    integer error_count = 0;
    reg [7:0] master_data;

    // 时钟参数 [cite: 45-47]
    parameter SCL_CLK_PERIOD = 100;    // 10MHz SCL
    parameter SDA_SAMPLE_DELAY = 20;   // SDA采样延迟

    // ==========================================
    // 实例化待测模块 (DUT) 
    // ==========================================
    i3c_slave_wrapper #(
        .ENA_ID48B(1),
        .ID_48B(48'h0123456789AB),
        .ID_BCR(8'h01),
        .ID_DCR(8'h01),
        .ENA_SADDR(1),
        .SADDR_P(7'h22),
        .ENA_CCC_HANDLING(6'b000001),
        .MAX_RDLEN(256),
        .MAX_WRLEN(256),
        .PIN_MODEL(2) // COMBO
    ) dut (
        .RSTn(RSTn),
        .pin_SCL_in(pin_SCL_in),
        .pin_SCL_out(pin_SCL_out),
        .pin_SCL_oena(pin_SCL_oena),
        .pin_SDA_in(pin_SDA_in),
        .pin_SDA_out(pin_SDA_out),
        .pin_SDA_oena(pin_SDA_oena),
        .slv_enable(slv_enable),
        .int_start_seen(int_start_seen),
        .int_da_matched(int_da_matched),
        .int_sa_matched(int_sa_matched),
        .int_7e_matched(int_7e_matched),
        .int_in_STOP(int_in_STOP),
        .dyn_addr(dyn_addr),
        .opt_static_addr(opt_static_addr),
        .dyn_addr_chg(dyn_addr_chg),
        .dyn_chg_cause(dyn_chg_cause),
        .tb_data_valid(tb_data_valid),
        .tb_datab(tb_datab),
        .tb_end(tb_end),
        .tb_datab_ack(tb_datab_ack),
        .fb_data_use(fb_data_use),
        .fb_datab(fb_datab),
        .fb_datab_done(fb_datab_done),
        .fb_datab_err(fb_datab_err),
        .cf_IdInst(4'h0),
        .cf_IdRand(1'b0),
        .cf_Partno(32'h12345678),
        .cf_IdBcr(8'h01),
        .cf_IdDcr(8'h01),
        .cf_IdVid(15'h1234),
        .cf_MaxRd(12'h100),
        .cf_MaxWr(12'h100),
        .cf_SdrOK(cf_SdrOK),
        .cf_DdrOK(cf_DdrOK),
        .cf_SlvNack(cf_SlvNack),
        .cf_s0ignore(cf_s0ignore),
        .cf_SetDA(cf_SetDA),
        .scan_single_clock(1'b0),
        .scan_clk(1'b0),
        .scan_no_rst(1'b0),
        .scan_no_gates(1'b0)
    );

    // ==========================================
    // 时钟生成 [cite: 60]
    // ==========================================
    always #(SCL_CLK_PERIOD/2) pin_SCL_in = ~pin_SCL_in;

    // ==========================================
    // 测试任务 (Tasks) [cite: 77-120]
    // ==========================================
    
    task send_i2c_start;
        begin
            #SDA_SAMPLE_DELAY;
            pin_SDA_in = 1'b0;
            #SDA_SAMPLE_DELAY;
        end
    endtask

    task send_i2c_stop;
        begin
            #SDA_SAMPLE_DELAY;
            pin_SDA_in = 1'b1;
            #SDA_SAMPLE_DELAY;
        end
    endtask

    task send_byte;
        input [7:0] data;
        integer i;
        begin
            for (i = 7; i >= 0; i = i - 1) begin
                pin_SCL_in = 1'b0;
                #SDA_SAMPLE_DELAY;
                pin_SDA_in = data[i];
                #SDA_SAMPLE_DELAY;
                pin_SCL_in = 1'b1;
                #(SCL_CLK_PERIOD/2 - SDA_SAMPLE_DELAY*2);
            end
            pin_SCL_in = 1'b0;
            #SDA_SAMPLE_DELAY;
            pin_SDA_in = 1'b1; // 释放SDA等待ACK
            #SDA_SAMPLE_DELAY;
            pin_SCL_in = 1'b1; // 从机采样ACK位
            #(SCL_CLK_PERIOD/2);
            pin_SCL_in = 1'b0;
        end
    endtask

    task receive_byte_check;
        input [7:0] expected_val;
        output [7:0] data;
        integer i;
        begin
            for (i = 7; i >= 0; i = i - 1) begin
                pin_SCL_in = 1'b0;
                #SDA_SAMPLE_DELAY;
                pin_SCL_in = 1'b1;
                #SDA_SAMPLE_DELAY;
                data[i] = pin_SDA_out; 
                #(SCL_CLK_PERIOD/2 - SDA_SAMPLE_DELAY*2);
            end
            pin_SCL_in = 1'b0;
            #SDA_SAMPLE_DELAY;
            pin_SDA_in = 1'b0; // 主机发送ACK
            pin_SCL_in = 1'b1;
            #(SCL_CLK_PERIOD/2);
            pin_SCL_in = 1'b0;
            pin_SDA_in = 1'b1;

            if (data === expected_val)
                $display(">>> [PASS] Received: 0x%02X", data);
            else begin
                $display(">>> [FAIL] Received: 0x%02X, Expected: 0x%02X", data, expected_val);
                error_count = error_count + 1;
            end
        end
    endtask

    // ==========================================
    // 测试主程序 [cite: 61-76]
    // ==========================================
    initial begin
        // 初始化 [cite: 62-63]
        $display("=== I3C Slave Testbench Starting ===");
        $dumpfile("tb_i3c_slave_wrapper.vcd");
        $dumpvars(0, tb_i3c_slave_wrapper);
        
        RSTn = 1'b0;
        pin_SCL_in = 1'b1;
        pin_SDA_in = 1'b1;
        
        #100 RSTn = 1'b1; 
        $display("Reset released at time %t", $time);
        #200;

        // --- 测试 1: I2C 写操作 ---
        $display("\n[TEST 1] I2C Write to Static Address 0x22");
        test_count = test_count + 1;
        send_i2c_start();
        send_byte(8'h22); // 0x11 << 1 | 0
        send_byte(8'hAA);
        send_i2c_stop();
        #100;
        if (!int_sa_matched) begin
            $display(">>> [FAIL] Static address match not detected");
            error_count = error_count + 1;
        end

        // --- 测试 2: I2C 读操作 (结果比对) ---
        $display("\n[TEST 2] I2C Read from Static Address 0x23");
        test_count = test_count + 1;
        tb_data_valid = 1'b1;
        tb_datab = 8'h55; // 从机准备的数据
        send_i2c_start();
        send_byte(8'h23); // 0x11 << 1 | 1
        receive_byte_check(8'h55, master_data);
        send_i2c_stop();
        tb_data_valid = 1'b0;

        // --- 测试 3: ENTDAA 过程 ---
        $display("\n[TEST 3] I3C ENTDAA Broadcast");
        test_count = test_count + 1;
        send_i2c_start();
        send_byte(8'h7E); // 广播地址
        send_byte(8'h07); // ENTDAA CCC
        send_i2c_stop();
        #1000;
        if (dyn_addr == 8'h00) begin
            $display(">>> [FAIL] Dynamic address not assigned after ENTDAA");
            error_count = error_count + 1;
        end else begin
            $display(">>> [PASS] Dynamic address assigned: 0x%02X", dyn_addr);
        end

        // --- 最终汇总 ---
        $display("\n=======================================");
        $display("  TEST SUMMARY");
        $display("  Total Test Cases: %d", test_count);
        $display("  Total Errors:     %d", error_count);
        if (error_count == 0)
            $display("  FINAL STATUS:     PASSED");
        else
            $display("  FINAL STATUS:     FAILED");
        $display("=======================================");
        
        #1000 $finish;
    end

    // ==========================================
    // 监控输出 [cite: 121-131]
    // ==========================================
    always @(posedge fb_datab_done) begin
        $display("Time %t: Slave received data: 0x%02X (Error: %b)", $time, fb_datab, fb_datab_err);
    end

    always @(posedge dyn_addr_chg) begin
        $display("Time %t: Dynamic address updated to 0x%02X", $time, dyn_addr);
    end



    initial	begin
	    $fsdbDumpfile("tb_i3c_slave_wrapper");//这个是产生名为tb.fsdb的文件
	    $fsdbDumpvars;
end
endmodule
