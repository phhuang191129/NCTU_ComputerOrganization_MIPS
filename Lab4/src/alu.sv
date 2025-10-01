`timescale 1ns / 1ps
// 612001061

/* checkout FIGURE C.5.12 */
/** [Prerequisite] complete bit_alu.v & msb_alu.v
 * We recommend you to design a 32-bit ALU with 1-bit ALU.
 * However, you can still implement ALU with more advanced feature in Verilog.
 * Feel free to code as long as the I/O ports remain the same shape.
 */
module alu (
    input  [31:0] a,        // 32 bits, source 1 (A)
    input  [31:0] b,        // 32 bits, source 2 (B)
    input  [ 3:0] ALU_ctl,  // 4 bits, ALU control input
    output logic[31:0] result,   // 32 bits, result
    output logic       zero,     // 1 bit, set to 1 when the output is 0
    output logic       overflow  // 1 bit, overflow
);

logic a_invert;
logic b_invert;
logic [1:0]operation;

assign a_invert = ALU_ctl[3];
assign b_invert = ALU_ctl[2];
assign operation = ALU_ctl[1:0];

logic [31:0] ai;
logic [31:0] bi;  
assign ai = {32{a_invert}}^a;
assign bi = {32{b_invert}}^b; 
logic [31:0] carry_in;

assign carry_in = b_invert ? 32'b1 : 32'b0;

always_comb begin
       case (operation)  // `case` is similar to `switch` in C
            2'b00:   begin
                result = ai & bi;
                overflow = 1'b0;        
            end
            2'b01:  begin
                result = ai | bi;
                overflow = 1'b0;        
            end 
            2'b10:  begin
                result = ai + bi + carry_in;
                overflow = (a[31] == bi[31]) & (a[31]^result[31]);        
            end
            2'b11:   begin
                result = $signed(a) < $signed(b);
                overflow = 1'b0;        
            end 
            default: begin
                result = 'x;
                overflow = 'x;        
            end   // should not happened
        endcase
        zero = result == '0;            
    end
endmodule
