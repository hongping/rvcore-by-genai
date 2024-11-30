module riscv32i_core (
    input logic clk,
    input logic reset,
    output logic [31:0] pc,
    output logic [31:0] instr
);

    // Program Counter (PC) and next PC logic
    logic [31:0] next_pc;
    logic branch_taken;
    logic [31:0] branch_target;

    // Register file and memory interfaces
    logic [31:0] regfile [31:0];    // Register file
    logic [31:0] data_mem [0:255];  // Data memory (simplified)
    logic [31:0] instr_mem [0:255]; // Instruction memory (simplified)
    assign instr = instr_mem[pc[7:0] >> 2]; // Fetch instruction from memory

    // IF/ID pipeline registers
    logic [31:0] if_id_instr, if_id_pc;
    
    // ID/EX pipeline registers
    logic [31:0] id_ex_pc, id_ex_imm;
    logic [31:0] id_ex_rs1_data, id_ex_rs2_data;
    logic [4:0]  id_ex_rs1, id_ex_rs2, id_ex_rd;
    logic [6:0]  id_ex_opcode;
    logic [2:0]  id_ex_funct3;
    logic [6:0]  id_ex_funct7;

    // EX/MEM pipeline registers
    logic [31:0] ex_mem_alu_result, ex_mem_rs2_data;
    logic [4:0]  ex_mem_rd;
    logic ex_mem_mem_read, ex_mem_mem_write, ex_mem_reg_write;
    
    // MEM/WB pipeline registers
    logic [31:0] mem_wb_alu_result, mem_wb_mem_read_data;
    logic [4:0]  mem_wb_rd;
    logic mem_wb_reg_write;

    // PC logic
    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            pc <= 32'h0000_0000;
        end else begin
            pc <= next_pc;
        end
    end

    always_comb begin
        next_pc = branch_taken ? branch_target : pc + 4;
    end

    // IF/ID Pipeline Register
    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            if_id_instr <= 32'b0;
            if_id_pc <= 32'b0;
        end else begin
            if_id_instr <= instr;
            if_id_pc <= pc;
        end
    end

    // Instruction Decode (ID) Stage
    logic [4:0] rs1, rs2, rd;
    logic [6:0] opcode, funct7;
    logic [2:0] funct3;
    logic [31:0] imm;

    always_comb begin
        opcode = if_id_instr[6:0];
        rd     = if_id_instr[11:7];
        funct3 = if_id_instr[14:12];
        rs1    = if_id_instr[19:15];
        rs2    = if_id_instr[24:20];
        funct7 = if_id_instr[31:25];

        // Immediate extraction
        case (opcode)
            7'b0010011, 7'b0000011, 7'b1100111: imm = {{20{if_id_instr[31]}}, if_id_instr[31:20]}; // I-type
            7'b0100011: imm = {{20{if_id_instr[31]}}, if_id_instr[31:25], if_id_instr[11:7]};      // S-type
            7'b1100011: imm = {{20{if_id_instr[31]}}, if_id_instr[7], if_id_instr[30:25], if_id_instr[11:8], 1'b0}; // B-type
            7'b0110111, 7'b0010111: imm = {if_id_instr[31:12], 12'b0};                                 // U-type
            7'b1101111: imm = {{12{if_id_instr[31]}}, if_id_instr[19:12], if_id_instr[20], if_id_instr[30:21], 1'b0}; // J-type
            default: imm = 32'b0;
        endcase
    end

    // Register file read
    logic [31:0] rs1_data, rs2_data;
    assign rs1_data = regfile[rs1];
    assign rs2_data = regfile[rs2];

    // ID/EX Pipeline Register
    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            id_ex_pc <= 32'b0;
            id_ex_rs1 <= 5'b0;
            id_ex_rs2 <= 5'b0;
            id_ex_rd <= 5'b0;
            id_ex_rs1_data <= 32'b0;
            id_ex_rs2_data <= 32'b0;
            id_ex_imm <= 32'b0;
            id_ex_opcode <= 7'b0;
            id_ex_funct3 <= 3'b0;
            id_ex_funct7 <= 7'b0;
        end else begin
            id_ex_pc <= if_id_pc;
            id_ex_rs1 <= rs1;
            id_ex_rs2 <= rs2;
            id_ex_rd <= rd;
            id_ex_rs1_data <= rs1_data;
            id_ex_rs2_data <= rs2_data;
            id_ex_imm <= imm;
            id_ex_opcode <= opcode;
            id_ex_funct3 <= funct3;
            id_ex_funct7 <= funct7;
        end
    end

    // Execution (EX) Stage - ALU operations
    logic [31:0] alu_result;
    logic alu_branch;

    always_comb begin
        case (id_ex_opcode)
            7'b0110011: begin  // R-type
                case (id_ex_funct3)
                    3'b000: alu_result = (id_ex_funct7 == 7'b0100000) ? (id_ex_rs1_data - id_ex_rs2_data) : (id_ex_rs1_data + id_ex_rs2_data); // ADD/SUB
                    3'b111: alu_result = id_ex_rs1_data & id_ex_rs2_data;  // AND
                    3'b110: alu_result = id_ex_rs1_data | id_ex_rs2_data;  // OR
                    3'b100: alu_result = id_ex_rs1_data ^ id_ex_rs2_data;  // XOR
                    3'b001: alu_result = id_ex_rs1_data << id_ex_rs2_data[4:0];  // SLL
                    3'b101: alu_result = (id_ex_funct7 == 7'b0100000) ? (id_ex_rs1_data >>> id_ex_rs2_data[4:0]) : (id_ex_rs1_data >> id_ex_rs2_data[4:0]); // SRA/SRL
                    3'b010: alu_result = (id_ex_rs1_data < id_ex_rs2_data) ? 32'b1 : 32'b0; // SLT
                    default: alu_result = 32'b0;
                endcase
            end

            7'b0010011: begin  // I-type (e.g., ADDI)
                case (id_ex_funct3)
                    3'b000: alu_result = id_ex_rs1_data + id_ex_imm;  // ADDI
                    3'b111: alu_result = id_ex_rs1_data & id_ex_imm;  // ANDI
                    3'b110: alu_result = id_ex_rs1_data | id_ex_imm;  // ORI
                    default: alu_result = 32'b0;
                endcase
            end

            7'b1100011: begin  // B-type (e.g., BEQ)
                case (id_ex_funct3)
                    3'b000: alu_branch = (id_ex_rs1_data == id_ex_rs2_data); // BEQ
                    3'b001: alu_branch = (id_ex_rs1_data != id_ex_rs2_data); // BNE
                    3'b100: alu_branch = (id_ex_rs1_data < id_ex_rs2_data);  // BLT
                    3'b101: alu_branch = (id_ex_rs1_data >= id_ex_rs2_data); // BGE
                    default: alu_branch = 1'b0;
                endcase
                alu_result = id_ex_pc + id_ex_imm;
            end

            7'b1101111: begin  // JAL
                alu_result = id_ex_pc + id_ex_imm;
            end

            7'b1100111: begin  // JALR
                alu_result = (id_ex_rs1_data + id_ex_imm) & ~32'b1;
            end

            7'b0000011: begin  // Load (e.g., LW)
                alu_result = id_ex_rs1_data + id_ex_imm;
            end

            7'b0100011: begin  // Store (e.g., SW)
                alu_result = id_ex_rs1_data + id_ex_imm;
            end

            default: alu_result = 32'b0;
        endcase
    end

    // EX/MEM Pipeline Register
    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            ex_mem_alu_result <= 32'b0;
            ex_mem_rs2_data <= 32'b0;
            ex_mem_rd <= 5'b0;
            ex_mem_mem_read <= 1'b0;
            ex_mem_mem_write <= 1'b0;
            ex_mem_reg_write <= 1'b0;
        end else begin
            ex_mem_alu_result <= alu_result;
            ex_mem_rs2_data <= id_ex_rs2_data;
            ex_mem_rd <= id_ex_rd;
            ex_mem_mem_read <= (id_ex_opcode == 7'b0000011);  // LW
            ex_mem_mem_write <= (id_ex_opcode == 7'b0100011); // SW
            ex_mem_reg_write <= (id_ex_opcode == 7'b0110011 || id_ex_opcode == 7'b0010011 || id_ex_opcode == 7'b1101111 || id_ex_opcode == 7'b1100111 || id_ex_opcode == 7'b0000011);
        end
    end

    // Memory Access (MEM) Stage
    logic [31:0] mem_read_data;

    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            mem_read_data <= 32'b0;
        end else if (ex_mem_mem_read) begin
            mem_read_data <= data_mem[ex_mem_alu_result[7:0] >> 2];
        end else if (ex_mem_mem_write) begin
            data_mem[ex_mem_alu_result[7:0] >> 2] <= ex_mem_rs2_data;
        end
    end

    // MEM/WB Pipeline Register
    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            mem_wb_alu_result <= 32'b0;
            mem_wb_mem_read_data <= 32'b0;
            mem_wb_rd <= 5'b0;
            mem_wb_reg_write <= 1'b0;
        end else begin
            mem_wb_alu_result <= ex_mem_alu_result;
            mem_wb_mem_read_data <= mem_read_data;
            mem_wb_rd <= ex_mem_rd;
            mem_wb_reg_write <= ex_mem_reg_write;
        end
    end

    // Write-Back (WB) Stage
    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            regfile[mem_wb_rd] <= 32'b0;
        end else if (mem_wb_reg_write) begin
            regfile[mem_wb_rd] <= (ex_mem_mem_read) ? mem_wb_mem_read_data : mem_wb_alu_result;
        end
    end
endmodule
