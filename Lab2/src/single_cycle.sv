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
package single_cycle_package;

	typedef enum logic [5:0] {
		opcode_e__Rtype         = 6'h00,
		opcode_e__Jump          = 6'h02,
		opcode_e__BranchOnEqual = 6'h04,
		opcode_e__OrImmediate   = 6'h0d,
		opcode_e__LoadUpperImm  = 6'h0f,
		opcode_e__LoadWord      = 6'h23,
		opcode_e__StoreWord     = 6'h2b
	} opcode_e;

	typedef enum logic [5:0] {
		funct_e__Add         = 6'h20,
		funct_e__Sub         = 6'h22,
		funct_e__And         = 6'h24,
		funct_e__Or          = 6'h25,
		funct_e__SetLessThan = 6'h2a
	} funct_e;

	typedef logic [4:0] register_i;
	typedef logic [4:0] shamt_i;
	typedef logic [25:0] address_i;
	typedef logic [15:0] imm_i;
	typedef logic [31:0] instruction_t;

	typedef union packed {
		struct packed {
			opcode_e opcode;
			union packed {
				struct packed {
					register_i rs;
					register_i rt;
					register_i rd;
					shamt_i shamt;
					funct_e funct;
				} rtype;
				struct packed {
					register_i rs;
					register_i rt;
					imm_i immediate;
				} itype;
				struct packed {
					address_i address;
				} jtype;
				address_i flatten;
			} _;
		} _;
		instruction_t flatten;
	} instruction_u;

	typedef struct packed {
		logic [31:0] a;
		logic [31:0] b;
		logic [3:0] ctl;
	} alu_in_s;

	typedef struct packed {
		logic [31:0] result;
		logic zero;
		logic overflow;
	} alu_out_s;

	typedef struct packed {
		logic [4:0] raddr1;
		logic [4:0] raddr2;
		logic [4:0] waddr;
		logic [31:0] wdata;
		logic write_en;
	} reg_file_in_s;

	typedef struct packed {
		logic [31:0] rdata1;
		logic [31:0] rdata2;
	} reg_file_out_s;

	typedef struct {
		logic       reg_dst;     // select register destination: rt(0), rd(1)
		logic       alu_src;     // select 2nd operand of ALU: rt(0), sign-extended(1)
		logic       mem_to_reg;  // select data write to register: ALU(0), memory(1)
		logic       reg_write;   // enable write to register file
		logic       mem_read;    // enable read form data memory
		logic       mem_write;   // enable write to data memory
		logic       branch;      // this is a branch instruction or not (work with alu.zero)
		logic [3:0] alu_op;      // internal opcode for alu unit
		logic       jump;        // ?
	} inst_decode_out;

	function inst_decode_out DecodeInstruction;
		input opcode_e opcode;
		input funct_e funct;
	begin
		DecodeInstruction = {default : '0};
		case(opcode)
			opcode_e__Rtype: begin
				DecodeInstruction.reg_dst = 1'b1;
				DecodeInstruction.reg_write = 1'b1;
				case(funct)
					funct_e__Add        : begin DecodeInstruction.alu_op = 4'b0010; end
					funct_e__Sub        : begin DecodeInstruction.alu_op = 4'b0110; end
					funct_e__And        : begin DecodeInstruction.alu_op = 4'b0000; end
					funct_e__Or         : begin DecodeInstruction.alu_op = 4'b0001; end
					funct_e__SetLessThan: begin DecodeInstruction.alu_op = 4'b0111; end
					default: begin end
				endcase
			end
			opcode_e__LoadUpperImm,
			opcode_e__OrImmediate: begin
				DecodeInstruction.alu_src = 1'b1;
				DecodeInstruction.reg_write = 1'b1;
				DecodeInstruction.alu_op = 4'b0001;
			end
			opcode_e__LoadWord: begin
				DecodeInstruction.alu_src = 1'b1;
				DecodeInstruction.mem_to_reg = 1'b1;
				DecodeInstruction.reg_write = 1'b1;
				DecodeInstruction.mem_read = 1'b1;
				DecodeInstruction.alu_op = 4'b0010;
			end
			opcode_e__StoreWord: begin
				DecodeInstruction.alu_src = 1'b1;
				DecodeInstruction.mem_write = 1'b1;
				DecodeInstruction.alu_op = 4'b0010;
			end
			opcode_e__BranchOnEqual: begin
				DecodeInstruction.branch = 1'b1;
			end
			opcode_e__Jump: begin
				DecodeInstruction.jump = 1'b1;
			end
			default: begin
			end
		endcase
	end endfunction
endpackage

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
	import single_cycle_package::*;

	/////////////////////////////////////////
	// Do not modify from here
	/////////////////////////////////////////

	/* Instruction Memory */
	logic [31:0] instr_mem_address, instr_mem_instr;
	instr_mem #(
		.BYTES(TEXT_BYTES),
		.START(TEXT_START)
	) instr_mem (
		.address(instr_mem_address),
		.instr  (instr_mem_instr)
	);

	/* Register File */
	reg_file_in_s rf_in;
	reg_file_out_s rf_out;
	reg_file reg_file (
		.clk        (clk),
		.rstn       (rstn),
		.read_reg_1 (rf_in.raddr1),
		.read_reg_2 (rf_in.raddr2),
		.reg_write  (rf_in.write_en),
		.write_reg  (rf_in.waddr),
		.write_data (rf_in.wdata),
		.read_data_1(rf_out.rdata1),
		.read_data_2(rf_out.rdata2)
	);

	/* ALU */
	alu_in_s alu_in;
	alu_out_s alu_out;
	alu u_alu (
		.a(alu_in.a),
		.b(alu_in.b),
		.ALU_ctl(alu_in.ctl),
		.result(alu_out.result),
		.zero(alu_out.zero),
		.overflow(alu_out.overflow)
	);

	/* Data Memory */
	logic data_mem_mem_read, data_mem_mem_write;
	logic [31:0] data_mem_address, data_mem_write_data, data_mem_read_data;
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

	logic [31:0] pc;

	/////////////////////////////////////////
	// Do not modify to here
	/////////////////////////////////////////

	// registers
	logic [31:0] pc_r;
	logic [31:0] pc_w;

	// fetch and decode instruction from instruction memory
	instruction_u inst;
	inst_decode_out inst_decoded;
	logic [31:0] inst_imm_zeroext;
	logic [31:0] inst_imm_signext;
	assign instr_mem_address = pc_r;
	assign inst.flatten = instr_mem_instr;
	assign inst_decoded = DecodeInstruction(inst._.opcode, inst._._.rtype.funct);

	always_comb begin
		inst_imm_zeroext = 32'(inst._._.itype.immediate);
		inst_imm_signext = {{16{inst._._.itype.immediate[15]}}, inst._._.itype.immediate};
	end

	// wire pc_w to what TA need
	assign pc = pc_w;

	// TODO: shall handle JMP or BEQ
	assign pc_w = pc_r + 'd4;

	always_comb begin
		alu_in.a = rf_out.rdata1;
		alu_in.b = inst_decoded.alu_src ? inst_imm_zeroext : rf_out.rdata2;
		alu_in.ctl = inst_decoded.alu_op;
	end

	always_comb begin
		rf_in.raddr1 = inst._._.rtype.rs;
		rf_in.raddr2 = inst._._.rtype.rt;
		rf_in.write_en = inst_decoded.reg_write;
		rf_in.waddr = inst_decoded.reg_dst ? inst._._.rtype.rd : inst._._.rtype.rt;

		data_mem_mem_read = inst_decoded.mem_read;
		data_mem_mem_write = inst_decoded.mem_write;
		data_mem_address = rf_out.rdata1 + inst_imm_signext;
	end

	logic [31:0] alu_out_selected;
	always_comb begin
		alu_out_selected = (inst._.opcode == opcode_e__LoadUpperImm) ? (inst_imm_zeroext<<16) : alu_out.result;
		rf_in.wdata = inst_decoded.mem_to_reg ? data_mem_read_data : alu_out_selected;
		data_mem_write_data = alu_out_selected;
	end

	always_ff@(posedge clk or negedge rstn) begin
		if (!rstn) begin
			pc_r <= TEXT_START;
		end else begin
			pc_r <= pc_w;
		end
	end

endmodule
