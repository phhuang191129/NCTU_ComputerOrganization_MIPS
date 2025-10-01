`timescale 1ns / 1ps
// <your student id>

/** [Reading] 4.4 p.318-321
 * "Designing the Main Control Unit"
 */
/** [Prerequisite] alu_control.v
 * This module is the Control unit in FIGURE 4.17
 * You can implement it by any style you want.
 */

/* checkout FIGURE 4.16/18 to understand each definition of control signals */
module control (
    input  [5:0] opcode,      // the opcode field of a instruction is [?:?]
    output reg   reg_dst,     // select register destination: rt(0), rd(1)
    output reg   alu_src,     // select 2nd operand of ALU: rt(0), sign-extended(1)
    output reg   mem_to_reg,  // select data write to register: ALU(0), memory(1)
    output reg   reg_write,   // enable write to register file
    output reg   mem_read,    // enable read form data memory
    output reg   mem_write,   // enable write to data memory
    output reg   branch,      // this is a branch instruction or not (work with alu.zero)
    output reg [1:0] alu_op,       // ALUOp passed to ALU Control unit
    output reg   jump
);

    /* implement "combinational" logic satisfying requirements in FIGURE 4.18 */
    /* You can check the "Green Card" to get the opcode/funct for each instruction. */
    
    always@(*)begin
        case(opcode)
            6'b000000: begin //R-format
                reg_dst = 1'b1;
                alu_src = 1'b0;
                mem_to_reg = 1'b0;
                reg_write = 1'b1;
                mem_read = 1'b0;
                mem_write = 1'b0;
                branch = 1'b0;
                alu_op = 2'b10;
                jump=1'b0;       
            end
            6'b100011: begin //lw
                reg_dst = 1'b0;
                alu_src = 1'b1;
                mem_to_reg = 1'b1;
                reg_write = 1'b1;
                mem_read = 1'b1;
                mem_write = 1'b0;
                branch = 1'b0;
                alu_op = 2'b00;
                jump=1'b0;       
            end
            6'b101011: begin //sw
                reg_dst = 1'bX; //x
                alu_src = 1'b1;
                mem_to_reg = 1'bX; //x
                reg_write = 1'b0;
                mem_read = 1'b0;
                mem_write = 1'b1;
                branch = 1'b0;
                alu_op = 2'b00;
                jump=1'b0;       
            end
            6'b000100: begin //beq
                reg_dst = 1'bX;
                alu_src = 1'b0;
                mem_to_reg = 1'bX;
                reg_write = 1'b0;
                mem_read = 1'b0;
                mem_write = 1'b0;
                branch = 1'b1;
                alu_op = 2'b01;
                jump=1'b0;       
            end
            6'b000010: begin //jump
                reg_dst = 1'bX;
                alu_src = 1'b0;
                mem_to_reg = 1'bX;
                reg_write = 1'b0;
                mem_read = 1'b0;
                mem_write = 1'b0;
                branch = 1'b0;
                alu_op = 2'b0;
                jump=1'b1;       
            end
            6'b001111: begin //lui
                reg_dst = 1'b0;
                alu_src = 1'b0;
                mem_to_reg = 1'b0;
                reg_write = 1'b1;
                mem_read = 1'b0;
                mem_write = 1'b0;
                branch = 1'b0;
                alu_op = 2'b11;
                jump=1'b0;    
            end
            6'b001101: begin //ori
                reg_dst = 1'b0;
                alu_src = 1'b0;
                mem_to_reg = 1'b0;
                reg_write = 1'b1;
                mem_read = 1'b0;
                mem_write = 1'b0;
                branch = 1'b0;
                alu_op = 2'b10;
                jump=1'b0;         
            end
            default:begin //Should not happen
                reg_dst = 1'b1;
                alu_src = 1'b0;
                mem_to_reg = 1'b0;
                reg_write = 1'b1;
                mem_read = 1'b0;
                mem_write = 1'b0;
                branch = 1'b0;
                alu_op = 2'b10;
                jump=1'b0;       
            end
        endcase
    end

endmodule
