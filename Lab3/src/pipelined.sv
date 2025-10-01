`timescale 1ns / 1ps
// <your student id>

/** [Prerequisite] Lab 2: alu, control, alu_control
 * This module is the pipelined MIPS processor in FIGURE 4.51
 * You can implement it by any style you want, as long as it passes testbench
 */

/* checkout FIGURE 4.51 */

module pipelined #(
    parameter integer TEXT_BYTES = 1024,        // size in bytes of instruction memory
    parameter integer TEXT_START = 'h00400000,  // start address of instruction memory
    parameter integer DATA_BYTES = 1024,        // size in bytes of data memory
    parameter integer DATA_START = 'h10008000   // start address of data memory
) (
    input clk,  // clock
    input rstn  // negative reset
);

    /* Instruction Memory */
    logic [31:0] instr_mem_address, instr_mem_instr;
    instr_mem #(
        .BYTES(TEXT_BYTES),
        .START(TEXT_START)
    ) instr_mem (
        .address(instr_mem_address),
        .instr  (instr_mem_instr)
    );

    /* Register Rile */
    logic [4:0] reg_file_read_reg_1, reg_file_read_reg_2, reg_file_write_reg;
    logic reg_file_reg_write;
    logic [31:0] reg_file_write_data, reg_file_read_data_1, reg_file_read_data_2;
    reg_file reg_file (
        .clk        (~clk),                  // only write when negative edge
        .rstn       (rstn),
        .read_reg_1 (reg_file_read_reg_1),
        .read_reg_2 (reg_file_read_reg_2),
        .reg_write  (reg_file_reg_write),
        .write_reg  (reg_file_write_reg),
        .write_data (reg_file_write_data),
        .read_data_1(reg_file_read_data_1),
        .read_data_2(reg_file_read_data_2)
    );

    /* ALU */
    logic [31:0] alu_a, alu_b, alu_result;
    logic [3:0] alu_ALU_ctl;
    logic alu_zero, alu_overflow;
    alu alu (
        .a       (alu_a),
        .b       (alu_b),
        .ALU_ctl (alu_ALU_ctl),
        .result  (alu_result),
        .zero    (alu_zero),
        .overflow(alu_overflow)
    );

    /* Data Memory */
    logic data_mem_mem_read, data_mem_mem_write;
    logic [31:0] data_mem_address, data_mem_write_data, data_mem_read_data;
    data_mem #(
        .BYTES(DATA_BYTES),
        .START(DATA_START)
    ) data_mem (
        .clk       (~clk),                 // only write when negative edge
        .mem_read  (data_mem_mem_read),
        .mem_write (data_mem_mem_write),
        .address   (data_mem_address),
        .write_data(data_mem_write_data),
        .read_data (data_mem_read_data)
    );

    /* ALU Control */
    logic [1:0] alu_control_alu_op;
    logic [5:0] alu_control_funct;
    logic [3:0] alu_control_operation;
    alu_control alu_control (
        .alu_op   (alu_control_alu_op),
        .funct    (alu_control_funct),
        .operation(alu_control_operation)
    );

    /* (Main) Control */
    logic [5:0] control_opcode;
    // Execution/address calculation stage control lines
    logic control_reg_dst, control_alu_src;
    logic [1:0] control_alu_op;
    // Memory access stage control lines
    logic control_branch, control_mem_read, control_mem_write;
    // Wire-back stage control lines
    logic control_reg_write, control_mem_to_reg;
    control control (
        .opcode    (control_opcode),
        .reg_dst   (control_reg_dst),
        .alu_src   (control_alu_src),
        .mem_to_reg(control_mem_to_reg),
        .reg_write (control_reg_write),
        .mem_read  (control_mem_read),
        .mem_write (control_mem_write),
        .branch    (control_branch),
        .alu_op    (control_alu_op)
    );

 
    typedef struct packed{
        logic reg_write;
        logic mem_to_reg;    
    } WB_s;
    typedef struct packed{
        logic reg_dst;
        logic alu_src;
        logic [1:0] alu_op;    
    } EX_s;
    typedef struct packed{
        logic branch;
        logic mem_read;
        logic mem_write;
    } MEM_s;
    typedef logic [31:0] PC_t;
    typedef logic [31:0] Operand_t;
    typedef logic [4:0] Reg_number_t;
    typedef struct packed{
        Operand_t data1;
        Operand_t data2;
    } RegFileRead_s;
    typedef struct packed{
        Reg_number_t instr_20_16;
        Reg_number_t instr_15_11;
    } MemInstr_s;
    
    /** [step 1] Instruction fetch (IF)
     * 1. We need a register to store PC (acts like pipeline register).
     * 2. Wire pc to instruction memory.
     * 3. Implement an adder to calculate PC+4. (combinational)
     *    Hint: use "+" operator.
     * 4. Update IF/ID pipeline registers, and reset them @(negedge rstn)
     *    a. fetched instruction
     *    b. PC+4
     *    Hint: What else should be done when reset?
     *    Hint: Update of PC can be handle later in MEM stage.
     */
    // 1.
    reg [31:0] pc;  // DO NOT change this line
    // 2.
    assign instr_mem_address = pc;
    // 3.
    PC_t pc_4;
    assign pc_4 = pc+ 32'd4;
    // 4.
    PC_t IF_ID_instr;
    PC_t IF_ID_pc_4;
     always_ff @(posedge clk or negedge rstn) begin
        if (!rstn) begin
           IF_ID_instr <= '0;  // a.
           IF_ID_pc_4  <= '0;  // b.
        end else begin        
           IF_ID_instr <= instr_mem_instr; // a.
           IF_ID_pc_4  <= pc_4;            // b.
        end
     end

    /** [step 2] Instruction decode and register file read (ID)
     * From top to down in FIGURE 4.51: (instr. refers to the instruction from IF/ID)
     * 1. Generate control signals of the instr. (as Lab 2)
     * 2. Read desired registers (from register file) in the instr.
     * 3. Calculate sign-extended immediate from the instr.
     * 4. Update ID/EX pipeline registers, and reset them @(negedge rstn)
     *    a. Control signals (WB, MEM, EX)
     *    b. ??? (something from IF/ID)
     *    c. Data read from register file
     *    d. Sign-extended immediate
     *    e. ??? & ??? (WB stage needs to know which reg to write)
     */
     EX_s ID_EX_ex;
     MEM_s ID_EX_mem;
     WB_s ID_EX_wb;
     PC_t ID_EX_pc_4;
     RegFileRead_s ID_EX_regfile;
     Operand_t ID_EX_sign_extended;
     MemInstr_s ID_EX_mem_instr;
     assign {control_opcode, reg_file_read_reg_1, reg_file_read_reg_2} = IF_ID_instr[31:16];
     always_ff @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            ID_EX_ex <= '{default: '0};
            ID_EX_mem <= '{default: '0};
            ID_EX_wb <= '{default: '0};
            ID_EX_pc_4 <= '0;
            ID_EX_regfile <= '{default: '0};
            ID_EX_sign_extended <= '0;
            ID_EX_mem_instr <= '{default: '0};
        end else begin
            ID_EX_ex <= '{
                reg_dst: control_reg_dst,
                alu_src: control_alu_src,
                alu_op: control_alu_op
            };
            ID_EX_mem <= '{    
                branch: control_branch,
                mem_read: control_mem_read,
                mem_write: control_mem_write        
            };
            ID_EX_wb <= '{
                reg_write: control_reg_write,
                mem_to_reg: control_mem_to_reg
            };
            ID_EX_pc_4 <= IF_ID_pc_4;
            ID_EX_regfile <= '{
                data1: reg_file_read_data_1,
                data2: reg_file_read_data_2
            };
            ID_EX_sign_extended <= {{16{IF_ID_instr[15]}}, IF_ID_instr[15:0]};
            ID_EX_mem_instr <= MemInstr_s'(IF_ID_instr[20:11]);
            
        end
     end
     
     
    /** [step 3] Execute or address calculation (EX)
     * From top to down in FIGURE 4.51
     * 1. Calculate branch target address from sign-extended immediate.
     * 2. Select correct operands of ALU like in Lab 2.
     * 3. Wire control signals to ALU control & ALU like in Lab 2.
     * 4. Select correct register to write.
     * 5. Update EX/MEM pipeline registers, and reset them @(negedge rstn)
     *    a. Control signals (WB, MEM)
     *    b. Branch target address
     *    c. ??? (What information dose MEM stage need to determine whether to branch?)
     *    d. ALU result
     *    e. ??? (What information does MEM stage need when executing Store?)
     *    f. ??? (WB stage needs to know which reg to write)
     */
    MEM_s EX_MEM_mem;
    WB_s EX_MEM_wb;
    PC_t EX_MEM_pc_sign_extended;
    logic EX_MEM_alu_zero;
    Operand_t EX_MEM_alu_result;
    Operand_t EX_MEM_reg_file_read_data_2;
    Reg_number_t EX_MEM_write_reg;
    always_comb begin
        alu_b = (ID_EX_ex.alu_src ==1'b0)?ID_EX_regfile.data2:ID_EX_sign_extended;
        alu_a=ID_EX_regfile.data1;
        alu_control_alu_op = ID_EX_ex.alu_op;
        alu_control_funct = ID_EX_sign_extended[5:0];
        alu_ALU_ctl =alu_control_operation;
    end  
    always_ff @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            EX_MEM_mem <= '{default: '0};
            EX_MEM_wb <= '{default: '0};
            EX_MEM_pc_sign_extended <= '0;
            EX_MEM_alu_zero <= '0;
            EX_MEM_alu_result <= '0;
            EX_MEM_reg_file_read_data_2 <= '0;
            EX_MEM_write_reg <= '0;
        end else begin
            EX_MEM_mem <= ID_EX_mem; //a
            EX_MEM_wb <= ID_EX_wb; //a
            EX_MEM_pc_sign_extended <= ID_EX_pc_4 + (ID_EX_sign_extended<<2); //b
            EX_MEM_alu_zero <= alu_zero; //c
            EX_MEM_alu_result <= alu_result; //d
            EX_MEM_reg_file_read_data_2 <= ID_EX_regfile.data2; //e
            EX_MEM_write_reg <= (ID_EX_ex.reg_dst==1'b0)?ID_EX_mem_instr.instr_20_16 : ID_EX_mem_instr.instr_15_11;//f
        end
     end
    /** [step 4] Memory access (MEM)
     * From top to down in FIGURE 4.51
     * 1. Decide whether to branch or not.
     * 2. Wire address & data to write
     * 3. Wire control signal of read/write
     * 4. Update MEM/WB pipeline registers, and reset them @(negedge rstn)
     *    a. Control signals (WB)
     *    b. ???
     *    c. ???
     *    d. ???
     * 5. Update PC.
     */
    logic pc_src;
    WB_s MEM_WB_wb;
    Operand_t MEM_WB_data_mem_read_data;
    Operand_t MEM_WB_alu_result;
    Reg_number_t MEM_WB_write_reg;
    always_comb begin
        pc_src = EX_MEM_mem.branch & EX_MEM_alu_zero;
        data_mem_address = EX_MEM_alu_result;
        data_mem_write_data = EX_MEM_reg_file_read_data_2;
        data_mem_mem_read = EX_MEM_mem.mem_read; 
        data_mem_mem_write = EX_MEM_mem.mem_write;
    end
    always_ff @(posedge clk or negedge rstn) begin
        if (!rstn) begin
           MEM_WB_wb <= '{default: '0};
           MEM_WB_data_mem_read_data <= '0;
           MEM_WB_alu_result <= '0;
           MEM_WB_write_reg <= '0;
        end else begin      
           MEM_WB_wb <= EX_MEM_wb; //a
           MEM_WB_data_mem_read_data <= data_mem_read_data;
           MEM_WB_alu_result <= EX_MEM_alu_result;
           MEM_WB_write_reg <= EX_MEM_write_reg;
        end
    end 
     
//    always @(posedge clk)
//        if (rstn) begin
//            pc <= (pc_src==1'b0) ? pc_4 : EX_MEM_pc_sign_extended;   // 5.
//        end
//    always @(negedge rstn) begin
//        pc <= 32'h00400000;
//    end
    always_ff @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            pc <= 32'h00400000;
        end else begin      
            pc <= (pc_src==1'b0) ? pc_4 : EX_MEM_pc_sign_extended;   // 5.
        end
     end 

    /** [step 5] Write-back (WB)
     * From top to down in FIGURE 4.51
     * 1. Wire RegWrite of register file.
     * 2. Select the data to write into register file.
     * 3. Select which register to write.
     */
     always_comb begin
        reg_file_reg_write = MEM_WB_wb.reg_write;
        reg_file_write_data = (MEM_WB_wb.mem_to_reg==1'b0)? MEM_WB_alu_result : MEM_WB_data_mem_read_data;
        reg_file_write_reg = MEM_WB_write_reg; 
     end

endmodule  // pipelined
