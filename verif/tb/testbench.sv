module testbench();

    // TB clock
    bit clk = 0;
    always #5ps clk <= ~clk;

    // TB reset
    bit rst = 0;

    // DUT interface
    bit [31:0] pc;
    bit [31:0] instr;

    // DUT
    riscv32i_core dut(.clk(clk), .reset(rst), .pc(pc), .instr(instr));

    // load into instruction memory
    initial begin
        dut.instr_mem[0] = 32'h06600093;  // add x1, x2, x3
        dut.instr_mem[1] = 32'h40b28233;  // sub x4, x5, x6
        dut.instr_mem[2] = 32'h00042303;  // lw x7, 0(x8)
        dut.instr_mem[3] = 32'h00902423;  // sw x9, 4(x10)
        dut.instr_mem[4] = 32'h00a60693;  // addi x11, x12, 10
        dut.instr_mem[5] = 32'h00f707b3;  // and x13, x14, x15
        dut.instr_mem[6] = 32'h00188793;  // ori x16, x17, 1
        dut.instr_mem[7] = 32'hfff10ae3;  // beq x1, x2, -4
        dut.instr_mem[8] = 32'h008002ef;  // jal x5, 8
        dut.instr_mem[9] = 32'h00008067;  // jalr x0, x1, 0
    end

    
    always @(dut.regfile[1]) begin
        $display(dut.regfile[1]);
    end

    initial begin
        #500ns;
        $display("Ending test after 500ns");
        $finish();
    end
endmodule
