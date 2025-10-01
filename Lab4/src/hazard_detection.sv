`timescale 1ns / 1ps
// <your student id>

/** [Reading] 4.7 p.372-375
 * Understand when and how to detect stalling caused by data hazards.
 * When read a reg right after it was load from memory,
 * it is impossible to solve the hazard just by forwarding.
 */

/* checkout FIGURE 4.59 to understand why a stall is needed */
/* checkout FIGURE 4.60 for how this unit should be connected */
module hazard_detection (
    input        ID_EX_mem_read,
    input  [4:0] ID_EX_rt,
    input  [4:0] IF_ID_rs,
    input  [4:0] IF_ID_rt,
    
    input  [4:0] EX_MEM_rt,
    input  [4:0] EX_MEM_rs,
//    input  [4:0] ID_EX_rd,
    input  [4:0] ID_EX_rs,
//    input  logic[31:0] b_eq,
//    input logic MEM_WB_mem_read,
    input logic pc_src_branch,
    input logic EX_MEM_read,
    
    output logic pc_write,        // only update PC when this is set
    output logic IF_ID_write,     // only update IF/ID stage registers when this is set
    output logic stall            // insert a stall (bubble) in ID/EX when this is set
);

    /** [step 3] Stalling
     * 1. calculate stall by equation from textbook.
     * 2. Should pc be written when stall?
     * 3. Should IF/ID stage registers be updated when stall?
     */
    logic load_stall,branch_stall_once, branch_stall_twice;
    logic imm_in_prev_immi_instr,imm_in_2nd_load,imm_in_prev_r_instr;
    always_comb begin
        imm_in_prev_immi_instr = ((ID_EX_rt == IF_ID_rs) | (ID_EX_rt == IF_ID_rt));
        load_stall = (ID_EX_mem_read & imm_in_prev_immi_instr);
        branch_stall_once = (((IF_ID_rs==ID_EX_rs)|(IF_ID_rt==ID_EX_rs) |(IF_ID_rs==ID_EX_rt)|(IF_ID_rt==ID_EX_rt))& ID_EX_mem_read &pc_src_branch);
        branch_stall_twice = (((IF_ID_rs == EX_MEM_rs)|(IF_ID_rt==EX_MEM_rs)|(IF_ID_rs==EX_MEM_rt)|(IF_ID_rt==EX_MEM_rt)) &ID_EX_mem_read &pc_src_branch);
        
//        imm_in_2nd_load =  (((EX_MEM_rt == IF_ID_rs) | (EX_MEM_rt == IF_ID_rt)) &  EX_MEM_read;
//        imm_in_prev_r_instr = ((ID_EX_rd == IF_ID_rs) | (ID_EX_rd == IF_ID_rt));
//        if?the?destination?register? feld?of?the?load?in?the?EX?stage?matches?either?source?register?of?the?instruction
//        load_stall= (ID_EX_mem_read & imm_in_prev_immi_instr);
//        branch_stall_once = (pc_src_branch & ((imm_in_prev_r_instr &ID_EX_mem_read) | imm_in_2nd_load));
//        branch_stall_twice = (pc_src_branch & (load_stall | imm_in_2nd_load));
       if (load_stall | branch_stall_once | branch_stall_twice) begin
            stall =1'b1;
        end else begin
            stall = 1'b0;
        end
    end
     
    assign pc_write = !stall;
    assign IF_ID_write = !stall;


endmodule
