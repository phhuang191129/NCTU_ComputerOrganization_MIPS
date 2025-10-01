`timescale 1ns / 1ps
// <your student id>

/* checkout FIGURE 4.7 */
module reg_file (
    input         clk,          // clock
    input         rstn,         // negative reset
    input  [ 4:0] read_reg_1,   // Read Register 1 (address)
    input  [ 4:0] read_reg_2,   // Read Register 2 (address)
    input         reg_write,    // RegWrite: write data when posedge clk
    input  [ 4:0] write_reg,    // Write Register (address)
    input  [31:0] write_data,   // Write Data
    output [31:0] read_data_1,  // Read Data 1
    output [31:0] read_data_2   // Read Data 2
);

    /* [step 1] How many bits per register? How many registers does MIPS have? */
    reg [31:0] registers[0:31];  // do not change its name
    wire [31:0] reg0; assign reg0 = registers[0];
    wire [31:0] reg1; assign reg1 = registers[1];
    wire [31:0] reg2; assign reg2 = registers[2];
    wire [31:0] reg3; assign reg3 = registers[3];
    wire [31:0] reg4; assign reg4 = registers[4];
    wire [31:0] reg5; assign reg5 = registers[5];
    wire [31:0] reg6; assign reg6 = registers[6];
    wire [31:0] reg7; assign reg7 = registers[7];
    wire [31:0] reg8; assign reg8 = registers[8];
    wire [31:0] reg9; assign reg9 = registers[9];
    wire [31:0] reg10; assign reg10 = registers[10];
    wire [31:0] reg11; assign reg11 = registers[11];
    wire [31:0] reg12; assign reg12 = registers[12];
    wire [31:0] reg13; assign reg13 = registers[13];
    wire [31:0] reg14; assign reg14 = registers[14];
    wire [31:0] reg15; assign reg15 = registers[15];
    wire [31:0] reg16; assign reg16 = registers[16];
    wire [31:0] reg17; assign reg17 = registers[17];
    wire [31:0] reg18; assign reg18 = registers[18];
    wire [31:0] reg19; assign reg19 = registers[19];
    wire [31:0] reg20; assign reg20 = registers[20];
    wire [31:0] reg21; assign reg21 = registers[21];
    wire [31:0] reg22; assign reg22 = registers[22];
    wire [31:0] reg23; assign reg23 = registers[23];
    wire [31:0] reg24; assign reg24 = registers[24];
    wire [31:0] reg25; assign reg25 = registers[25];
    wire [31:0] reg26; assign reg26 = registers[26];
    wire [31:0] reg27; assign reg27 = registers[27];
    wire [31:0] reg28; assign reg28 = registers[28];
    wire [31:0] reg29; assign reg29 = registers[29];
    wire [31:0] reg30; assign reg30 = registers[30];
    wire [31:0] reg31; assign reg31 = registers[31];

    /* [step 2] Read Registers */
    /* Remember to check whether register number is zero */
    assign read_data_1 = (read_reg_1==5'b0) ? 32'b0 : registers[read_reg_1];
    assign read_data_2 = (read_reg_2==5'b0) ? 32'b0 : registers[read_reg_2];
    /** Sequential Logic
     * `posedge clk` means that this block will execute when clk changes from 0 to 1 (positive edge trigger).
     * `negedge rstn` vice versa.
     * https://www.chipverify.com/verilog/verilog-always-block
     */
    /* [step 3] Write Registers */
    integer i;
    always @(posedge clk or negedge rstn)
        if (~rstn) begin  // make sure to check reset!
            for (i=0;i<32;i=i+1) begin
                registers[i]<=32'b0;
            end
        end else begin
            if (reg_write & (write_reg!=4'b0)) begin //
                registers[write_reg]<=write_data;
            end
        end

endmodule
