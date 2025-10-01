`timescale 1ns / 1ps
// 612001061

/** [Reading] 4.4 p.321-327
 * "Operation of the Datapath"
 * "Finalizing Control": double check your control.v !
 */
/** [Prerequisite] control.v
 * This module is the single-cycle MIPS processor in FIGURE 4.17
 * You can implement it by any style you want, but port `clk` & `rstn` must remain.
 */

/* checkout FIGURE 4.17 */
module single_cycle #(
    parameter integer TEXT_BYTES = 1024,        // size in bytes of instruction memory
    parameter integer TEXT_START = 'h00400000,  // start address of instruction memory
    parameter integer DATA_BYTES = 1024,        // size in bytes of data memory
    parameter integer DATA_START = 'h10008000   // start address of data memory
) (
    input clk,  // clock
    input rstn  // negative reset
);

    /* Instruction Memory */
    wire [31:0] instr_mem_address, instr_mem_instr;
    instr_mem #(
        .BYTES(TEXT_BYTES),
        .START(TEXT_START)
    ) instr_mem (
        .address(instr_mem_address),
        .instr  (instr_mem_instr)
    );

    /* Register Rile */
    wire [4:0] reg_file_read_reg_1, reg_file_read_reg_2, reg_file_write_reg;
    wire reg_file_reg_write;
    wire [31:0] reg_file_write_data, reg_file_read_data_1, reg_file_read_data_2;
    reg_file reg_file (
        .clk        (clk),
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
    wire [31:0] alu_result;
    reg [31:0] alu_a, alu_b;
    wire [3:0] alu_ALU_ctl;
    wire alu_zero, alu_overflow;
    alu alu (
        .a       (alu_a),
        .b       (alu_b),
        .ALU_ctl (alu_ALU_ctl),
        .result  (alu_result),
        .zero    (alu_zero),
        .overflow(alu_overflow)
    );

    /* Data Memory */
    wire data_mem_mem_read, data_mem_mem_write;
    wire [31:0] data_mem_address, data_mem_write_data, data_mem_read_data;
    data_mem #(
        .BYTES(DATA_BYTES),
        .START(DATA_START)
    ) data_mem (
        .clk       (clk),
        .mem_read  (data_mem_mem_read),
        .mem_write (data_mem_mem_write),
        .address   (data_mem_address),
        .write_data(data_mem_write_data),
        .read_data (data_mem_read_data)
    );

    /* ALU Control */
    wire [1:0] alu_control_alu_op;
    wire [5:0] alu_control_funct;
    wire [3:0] alu_control_operation;
    alu_control alu_control (
        .alu_op   (alu_control_alu_op),
        .funct    (alu_control_funct),
        .operation(alu_control_operation)
    );

    /* (Main) Control */  // named without `control_` prefix!
    wire [5:0] opcode;
    wire reg_dst, alu_src, mem_to_reg, reg_write, mem_read, mem_write, branch,jump;
    wire [1:0] alu_op;
    control control (
        .opcode    (opcode),
        .reg_dst   (reg_dst),
        .alu_src   (alu_src),
        .mem_to_reg(mem_to_reg),
        .reg_write (reg_write),
        .mem_read  (mem_read),
        .mem_write (mem_write),
        .branch    (branch),
        .alu_op    (alu_op),
        .jump       (jump)   
    );

    reg [31:0] pc;  // DO NOT change this line
    reg [31:0] pc_start;
    assign instr_mem_address = pc_start;
    wire [31:0] pc_step_4;
    assign pc_step_4 = pc_start + 32'd4;
    assign {opcode,reg_file_read_reg_1, reg_file_read_reg_2} = instr_mem_instr[31:16];
    wire is_imm;
    wire is_lui, is_ori;
    reg [31:0] sign_extended;
    assign is_lui = (opcode == 6'b001111);
    assign is_ori = (opcode ==6'b001101);
    assign is_imm = (is_lui | is_ori);
    always@(*)begin
        
       if (~is_imm)begin
           sign_extended = {{16{instr_mem_instr[15]}},instr_mem_instr[15:0]};
           alu_b = (alu_src ==1'b0)?reg_file_read_data_2:sign_extended;
           alu_a=reg_file_read_data_1; 
       end else if(is_ori)begin
           sign_extended = 32'b0;
           alu_a=reg_file_read_data_1; 
           alu_b = {16'b0, instr_mem_instr[15:0]};
       end else begin
           sign_extended = 32'b0;
           alu_a=32'b0;
           alu_b = 32'b0;
       end
    end
    assign reg_file_write_data =  is_lui? {instr_mem_instr[15:0],16'b0} : ((mem_to_reg==1'b0)?alu_result:data_mem_read_data);    
    assign reg_file_reg_write=reg_write;
    assign data_mem_mem_read=mem_read;
    assign data_mem_mem_write=mem_write;
    reg [31:0] pc_src; 
    always@(*)begin
       pc_src = ((branch&alu_zero)==1'b0)?pc_step_4:(pc_step_4 + (sign_extended<<2));
       pc = (jump==1'b0)?pc_src:{pc_step_4,instr_mem_instr[25:0],2'b0};
    end
    assign alu_control_alu_op=alu_op;   
    assign alu_control_funct= instr_mem_instr[5:0];
    assign alu_ALU_ctl=alu_control_operation;
    assign data_mem_address=alu_result;
    assign data_mem_write_data=alu_result;  
    assign reg_file_write_reg = (reg_dst==1'b0)?instr_mem_instr[20:16]:instr_mem_instr[15:11];
    always @(posedge clk or negedge rstn)
        if (~rstn) begin
            pc_start <= 'h00400000;
        end else begin
            pc_start <= pc;
    end
 
//    always @(negedge rstn) begin
//        ???
//    end

endmodule
