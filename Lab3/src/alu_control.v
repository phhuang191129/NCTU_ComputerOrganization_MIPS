`timescale 1ns / 1ps
//612001061

/** [Reading] 4.4 p.316-318
 * "The ALU Control"
 */
/**
 * This module is the ALU control in FIGURE 4.17
 * You can implement it by any style you want.
 * There's a more hardware efficient design in Appendix D.
 */

/* checkout FIGURE 4.12/13 */
module alu_control (
    input  [1:0] alu_op,    // ALUOp
    input  [5:0] funct,     // Funct field
    output reg [3:0] operation  // Operation
);
    
    /* implement "combinational" logic satisfying requirements in FIGURE 4.12 */
    always @(*) begin
        case (alu_op)
            2'b00:   operation = 4'b0010;  //LW/SW
            2'b01:   operation = 4'b0110; // /beq  
            2'b10:   begin //R-format
                        case(funct)
                            6'b100000: operation = 4'b0010;
                            6'b100010: operation = 4'b0110;
                            6'b100100: operation = 4'b0000;
                            6'b100101: operation = 4'b0001;
                            6'b101010: operation = 4'b0111;
                            default: operation = 4'b0000;
                        endcase
                     end
            2'b11:   operation = 4'b0010; //addi

        endcase
    end
endmodule
