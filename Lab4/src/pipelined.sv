`timescale 1ns / 1ps
// <your student id>

/** [Prerequisite] pipelined (Lab 3), forwarding, hazard_detection
 * This module is the pipelined MIPS processor "similar to" FIGURE 4.60 (control hazard is not solved).
 * You can implement it by any style you want, as long as it passes testbench.
 */

module pipelined #(
    parameter integer TEXT_BYTES = 1024,        // size in bytes of instruction memory
    parameter integer TEXT_START = 'h00400000,  // start address of instruction memory
    parameter integer DATA_BYTES = 1024,        // size in bytes of data memory
    parameter integer DATA_START = 'h10008000   // start address of data memory
) (
    input clk,  // clock
    input rstn  // negative reset
);

    initial begin
       $dumpfile("test.vcd");
       $dumpvars;
    end
    /** [step 0] Copy from Lab 3
     * You should modify your pipelined processor from Lab 3, so copy to here first.
          Instruction Memory */
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
    //stall
//    logic control_mem_write_stall, control_reg_write_stall;
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
    
    /** Instruction fetch (IF)
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
        end else if (IF_ID_write) begin        
           IF_ID_instr <= instr_mem_instr; // a.
           IF_ID_pc_4  <= pc_4;            // b.
        end
     end

    /**  Instruction decode and register file read (ID)
     */
     EX_s ID_EX_ex;
     MEM_s ID_EX_mem;
     WB_s ID_EX_wb;
     PC_t ID_EX_pc_4;
     RegFileRead_s ID_EX_regfile;
     Operand_t ID_EX_sign_extended;
     MemInstr_s ID_EX_mem_instr;
     assign {control_opcode, reg_file_read_reg_1, reg_file_read_reg_2} = IF_ID_instr[31:16];
     //forwarding
     Reg_number_t ID_EX_rs;
     // R format : opcode:31-26 rs:25-21 rt:20-16 rd:15-11
     //branch
     logic pc_src;
     logic branch_equality;
     Operand_t ID_EX_pc_sign_extended;
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
            if(stall) begin
                ID_EX_ex <= '{default: '0};
                ID_EX_mem <= '{default: '0};
                ID_EX_wb <= '{default: '0};
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
            end

            ID_EX_pc_4 <= IF_ID_pc_4;
            ID_EX_regfile <= '{
                data1: reg_file_read_data_1,
                data2: reg_file_read_data_2
            };
            ID_EX_sign_extended <= {{16{IF_ID_instr[15]}}, IF_ID_instr[15:0]};
            
            ID_EX_mem_instr <= MemInstr_s'(IF_ID_instr[20:11]);
            //forwarding
            ID_EX_rs <= reg_file_read_reg_1;
            //branch
            ID_EX_pc_sign_extended <= ID_EX_pc_4 + (ID_EX_sign_extended<<2);
             
//             branch_equality <= ID_EX_regfile.data1 == ID_EX_regfile.data2;
          
         
//             pc_src <= ID_EX_mem.branch &  alu_zero;
        end
     end

    always_ff @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            pc <= 32'h00400000;
        end else if (pc_write) begin      
            pc <= (pc_src==1'b0) ? pc_4 : pc+({{16{IF_ID_instr[15]}}, IF_ID_instr[15:0]}<<2);   // 5.
        end
     end
    always_comb begin
         if (control_branch) begin
            if(stall) begin
                pc_src = forward_reg_file_write_data == forward_EX_MEM_alu_result;
            end else begin
                pc_src = reg_file_read_data_1 == reg_file_read_data_2;
            end
         end else begin
            pc_src = 1'b0;
         end  
    end

    /** Execute or address calculation (EX)
     */
    MEM_s EX_MEM_mem;
    WB_s EX_MEM_wb;
    PC_t EX_MEM_pc_sign_extended;
    logic EX_MEM_alu_zero;
    Operand_t EX_MEM_alu_result;
    Operand_t EX_MEM_reg_file_read_data_2;
    Reg_number_t EX_MEM_write_reg;
    Operand_t alu_b_no_forwarding;
    MemInstr_s EX_MEM_mem_instr;
    Reg_number_t EX_MEM_rs;
    always_comb begin
        alu_b_no_forwarding = (ID_EX_ex.alu_src ==1'b0)?ID_EX_regfile.data2:ID_EX_sign_extended;
//        alu_a_no_forwarding=ID_EX_regfile.data1;
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
//            alu_b_no_forwarding <= '0;
            EX_MEM_mem_instr <= '{default: '0};
            EX_MEM_rs <= '{default: '0};
        end else begin   
            EX_MEM_mem <= ID_EX_mem; //a
            EX_MEM_wb <= ID_EX_wb; //a
//                   EX_MEM_pc_sign_extended <= ID_EX_pc_4 + (ID_EX_sign_extended<<2); //b
            EX_MEM_alu_zero <= alu_zero; //c
            EX_MEM_alu_result <= alu_result; //d
            EX_MEM_reg_file_read_data_2 <= ID_EX_regfile.data2; //e
            EX_MEM_write_reg <= (ID_EX_ex.reg_dst==1'b0)?ID_EX_mem_instr.instr_20_16 : ID_EX_mem_instr.instr_15_11;//f
            EX_MEM_mem_instr <= ID_EX_mem_instr;
            EX_MEM_rs  <= ID_EX_rs;
        end
     end
    /** Memory access (MEM)
     */
//    logic  pc_src;
    WB_s MEM_WB_wb;
    Operand_t MEM_WB_data_mem_read_data;
    Operand_t MEM_WB_alu_result;
    Reg_number_t MEM_WB_write_reg;
//    logic MEM_WB_mem_read;
    always_comb begin
//        pc_src = EX_MEM_mem.branch & EX_MEM_alu_zero;
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
//           MEM_WB_mem_read<='0;
        end else begin      
           MEM_WB_wb <= EX_MEM_wb; //a
           MEM_WB_data_mem_read_data <= data_mem_read_data;
           MEM_WB_alu_result <= EX_MEM_alu_result;
           MEM_WB_write_reg <= EX_MEM_write_reg;
//           MEM_WB_mem_read <= data_mem_mem_read;
        end
    end 
    
    /** Write-back (WB)
     */
     always_comb begin
        reg_file_reg_write = MEM_WB_wb.reg_write;
        reg_file_write_data = (MEM_WB_wb.mem_to_reg==1'b0)? MEM_WB_alu_result : MEM_WB_data_mem_read_data;
        reg_file_write_reg = MEM_WB_write_reg; 
     end
    /** [step 2] Connect Forwarding unit
     * 1. add `ID_EX_rs` into ID/EX stage registers
     * 2. Use a mux to select correct ALU operands according to forward_A/B
     *    Hint don't forget that alu_b might be sign-extended immediate!
     */
    wire [1:0] forward_A, forward_B;
    wire forward_reg_data;
    logic[31:0] forward_reg_file_write_data,forward_EX_MEM_alu_result;
    forwarding forwarding (
        .ID_EX_rs        (ID_EX_rs),
        .ID_EX_rt        (ID_EX_mem_instr.instr_20_16),
        .EX_MEM_reg_write(EX_MEM_wb.reg_write),
        .EX_MEM_rd       (EX_MEM_write_reg),
        .MEM_WB_reg_write(MEM_WB_wb.reg_write),
        .MEM_WB_rd       (MEM_WB_write_reg),
        .is_branch   (ID_EX_mem.branch),
        .forward_A       (forward_A),
        .forward_B       (forward_B),
        .forward_reg_data  (forward_reg_data)

    );
    always_comb begin
        case(forward_A)
            2'b00: begin
                alu_a = ID_EX_regfile.data1;
            end
            2'b10: begin
                alu_a = EX_MEM_alu_result;
            end
            2'b01: begin
                alu_a = reg_file_write_data;
            end
            default: begin
                alu_a = 'x;
            end
        endcase
        case(forward_B)
            2'b00: begin
                alu_b = alu_b_no_forwarding;
            end
            2'b10: begin
                alu_b = EX_MEM_alu_result;
            end
            2'b01: begin
                alu_b = reg_file_write_data;
            end
            default: begin
                alu_b = 'x;
            end
        endcase
        case (forward_reg_data)
            1'b1: begin
                forward_reg_file_write_data = reg_file_write_data;
                forward_EX_MEM_alu_result = EX_MEM_alu_result;
            end
            1'b0: begin
                forward_reg_file_write_data = 'x;
                forward_EX_MEM_alu_result = 'x;
            end
        endcase
    end


    /** [step 4] Connect Hazard Detection unit
     * 1. use `pc_write` when updating PC
     * 2. use `IF_ID_write` when updating IF/ID stage registers
     * 3. use `stall` when updating ID/EX stage registers
     */
    hazard_detection hazard_detection (
        .ID_EX_mem_read(ID_EX_mem.mem_read),
        .ID_EX_rt      (ID_EX_mem_instr.instr_20_16),
        .IF_ID_rs      (IF_ID_instr[25:21]),
        .IF_ID_rt      (IF_ID_instr[20:16]),
        
        .EX_MEM_rt      (EX_MEM_mem_instr.instr_15_11),
        .EX_MEM_rs      (EX_MEM_rs),
        .ID_EX_rs       (ID_EX_rs),
//        .ID_EX_rd     (ID_EX_mem_instr.instr_15_11),
//        .b_eq          (branch_equality),
//        .MEM_WB_mem_read (MEM_WB_mem_read),
        .pc_src_branch        (control_branch),
        .EX_MEM_read   (EX_MEM_mem.mem_read),
        
        .pc_write      (pc_write),            // implicitly declared
        .IF_ID_write   (IF_ID_write),         // implicitly declared
        .stall         (stall)                // implicitly declared
    );
 

    
    /** [step 5] Control Hazard
     * This is the most difficult part since the textbook does not provide enough information.
     * By reading p.377-379 "Reducing the Delay of Branches",
     * we can disassemble this into the following steps:
     * 1. Move branch target address calculation & taken or not from EX to ID
     * 2. Move branch decision from MEM to ID
     * 3. Add forwarding for registers used in branch decision from EX/MEM
     * 4. Add stalling:
          branch read registers right after an ALU instruction writes it -> 1 stall
          branch read registers right after a load instruction writes it -> 2 stalls
     */
//    logic [1:0] counter_next, counter;
//    always_ff begin
//        if(!rst)begin
//            counter <= '0;
//        end else begin
            
//        end        
//    end
    
endmodule  // pipelined
