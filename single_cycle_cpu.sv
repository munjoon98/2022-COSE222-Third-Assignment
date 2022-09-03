/* ********************************************
 *	COSE222 Lab #3
 *
 *	Module: top design of the single-cycle CPU (single_cycle_cpu.sv)
 *  - Top design of the single-cycle CPU
 *
 *  Author: Gunjae Koo (gunjaekoo@korea.ac.kr)
 *
 * ********************************************
 */

`timescale 1ns/1ps

module single_cycle_cpu
#(  parameter IMEM_DEPTH = 1024,    // imem depth (default: 1024 entries = 4 KB)
              IMEM_ADDR_WIDTH = 10,
              REG_WIDTH = 64,
              DMEM_DEPTH = 1024,    // dmem depth (default: 1024 entries = 8 KB)
              DMEM_ADDR_WIDTH = 10 )
(
    input           clk,            // System clock
    input           reset_b         // Asychronous negative reset
);

    // Wires for datapath elements
    logic   [IMEM_ADDR_WIDTH-1:0]   imem_addr;
    logic   [31:0]  inst;   // instructions = an output of ????

    logic   [4:0]   rs1, rs2, rd;    // register numbers
    logic   [REG_WIDTH-1:0] rd_din;
    logic           reg_write;
    logic   [REG_WIDTH-1:0] rs1_dout, rs2_dout;

    logic   [REG_WIDTH-1:0] alu_in1, alu_in2;
    logic   [3:0]   alu_control;    // ALU control signal
    logic   [REG_WIDTH-1:0] alu_result;
    logic           alu_zero;
    logic           alu_sign;

    logic   [DMEM_ADDR_WIDTH-1:0]    dmem_addr;
    logic   [63:0]  dmem_din, dmem_dout;
    logic           mem_read, mem_write;

    // -------------------------------------------------------------------
    /* Main control unit:
     * Main control unit generates control signals for datapath elements
     * The control signals are determined by decoding instructions
     * Generating control signals using opcode = inst[6:0]
     */
    logic   [6:0]   opcode;
    logic   [3:0]   branch;
    logic           alu_src, mem_to_reg;
    logic   [1:0]   alu_op;
    logic   [2:0]   funct3;
    //logic         mem_read, mem_write, reg_write; // declared above
    // Note for Lab #3
    // The branch control signal has 4-bits since this processor supports beq, bne, blt, and bge
    // Each bit of the branch control signal represents the corresponding branch instruction
    // i.e., branch[0] = beq, branch[1] = bne, branch[2] = blt, branch[3] = bge

    // COMPLETE THE MAIN CONTROL UNIT HERE
    assign opcode = inst[6:0];
    assign funct3 = inst[14:12];
    assign branch[0] = (opcode==7'b1100011 && funct3==3'b000) ? 1'b1 : 1'b0;
    assign branch[1] = (opcode==7'b1100011 && funct3==3'b001) ? 1'b1 : 1'b0;
    assign branch[2] = (opcode==7'b1100011 && funct3==3'b100) ? 1'b1 : 1'b0;
    assign branch[3] = (opcode==7'b1100011 && funct3==3'b101) ? 1'b1 : 1'b0;

    assign mem_read = (opcode==7'b0000011) ? 1'b1:1'b0;    // ld
    assign mem_write = (opcode==7'b0100011) ? 1'b1:1'b0;   // sd
    assign mem_to_reg = (opcode==7'b0000011) ? 1'b1:1'b0;
    assign reg_write = (opcode==7'b0110011 || opcode==7'b0010011 || opcode==7'b0000011) ? 1'b1 : 1'b0; // ld, r-type, or i-type
    assign alu_src = (opcode==7'b0100011 || opcode==7'b0010011 || opcode==7'b0000011) ? 1'b1 : 1'b0;   // ld, sd, or i-type

    assign alu_op[0] = (opcode==7'b1100011) ? 1'b1 : 1'b0;
    assign alu_op[1] = (opcode==7'b0110011 || opcode==7'b0010011) ? 1'b1:1'b0;    // r-type or i-type


    // --------------------------------------------------------------------

    // --------------------------------------------------------------------
    /* ALU control unit:
     * ALU control unit generate alu_control signal which selects ALU operations
     * Generating control signals using alu_op, funct7, and funct3 fileds
     */
    logic   [6:0]   funct7;
    //logic   [2:0]   funct3;   // declared above

    // COMPLETE THE ALU CONTROL UNIT HERE
    assign funct7=inst[31:25];
    always_comb begin
        case (alu_op)
            2'b00: alu_control = 4'b0010;
            2'b01: alu_control = 4'b0110;
	    2'b10: 
		if(funct3==3'b000) begin
		 if(funct7==7'b0100000) alu_control=4'b0110; //sub
		 else alu_control=4'b0010; //add
		end
		else if(funct3==3'b100) alu_control=4'b1001; //xor
		else if(funct3==3'b110) alu_control=4'b0001; //or
		else if(funct3==3'b111) alu_control=4'b0000; //and
            default: alu_control = 4'b0010; 
	endcase
    end


    // ---------------------------------------------------------------------


    // ---------------------------------------------------------------------
    /* Immediate generator:
     * Generating immediate value from inst[31:0]
     * We require immediate type data for load, store, i-type, and branch instructions
     */
    logic   [63:0]  imm64;
    logic   [63:0]  imm64_branch;  // imm64 left shifted by 1
    logic   [11:0]  imm12;  // 12-bit immediate value extracted from inst

    // COMPLETE IMMEDIATE GENERATOR HERE
    always_comb begin
      case(opcode)
	7'b0000011:imm12=inst[31:20];
	7'b0010011:imm12=inst[31:20];
	7'b0100011:imm12={inst[31:25],inst[11:7]};
	7'b1100011:imm12={inst[31],inst[7],inst[30:25],inst[11:8]};
	default: imm12=12'b0;
      endcase
    end
	
    assign imm64={{52{imm12[11]}},imm12[11:0]};
    assign imm64_branch={imm64,1'b0};


    // ----------------------------------------------------------------------

    // Program counter
    logic   [63:0]  pc_curr, pc_next;
    logic           pc_next_sel;    // selection signal for pc_next
    logic   [63:0]  pc_next_plus4, pc_next_branch;


    assign pc_next_plus4 = pc_curr+64'd4;    // FILL THIS

    always_ff @ (posedge clk or negedge reset_b) begin
        if (~reset_b) begin
            pc_curr <= 'b0;
        end else begin
            pc_curr <= pc_next;        // FILL THIS
        end
    end


    // MUXes:
    // COMPLETE MUXES HERE
    // PC_NEXT
    assign pc_next_sel = (branch[0]==1'b1 && alu_result==0) || (branch[1]==1'b1 && alu_result!=0) || (branch[2]==1'b1 && alu_sign==1'b1) || (branch[3]==1'b1 && alu_sign==1'b0) ? 1'b1 : 1'b0;      // FILL THIS
    assign pc_next = (pc_next_sel) ? pc_next_branch: pc_next_plus4; // if branch is taken, pc_next_sel=1'b1
    assign pc_next_branch = pc_curr+imm64_branch;   // FILL THIS

    // ALU inputs
    assign alu_in1 = rs1_dout;
    assign alu_in2 = (alu_src==1'b1) ? imm64 : rs2_dout;

    // RF din
    assign rd_din = (mem_to_reg==1'b1) ? dmem_dout : alu_result;

    // COMPLETE CONNECTIONS HERE
    // imem
    assign imem_addr = pc_curr;

    // regfile
    assign rs1 = inst[19:15];
    assign rs2 = (opcode==7'b0000011 || opcode==7'b0010011) ? 1'b0 : inst[24:20];
    assign rd =  (opcode==7'b0100011 || opcode==7'b1100011) ? 1'b0 : inst[11:7];

    // dmem
    assign dmem_addr = rs1_dout+imm64;
    assign dmem_din = rs2_dout;
 

    // -----------------------------------------------------------------------
    /* Instantiation of datapath elements
     * All input/output ports should be connected
     */
    
    // IMEM
    imem #(
        .IMEM_DEPTH         (IMEM_DEPTH),
        .IMEM_ADDR_WIDTH    (IMEM_ADDR_WIDTH)
    ) u_imem_0 (
        .addr               ( imem_addr   ),
        .dout               (       inst        )
    );

    // REGFILE
	regfile #(
		.REG_WIDTH (REG_WIDTH)
	) u_regfile_0(
		.clk (clk),
		.rs1 (rs1),
		.rs2 (rs2),
		.rd (rd),
		.rd_din (rd_din),
		.reg_write (reg_write),
		.rs1_dout (rs1_dout),
		.rs2_dout (rs2_dout)
	);
    // ALU
	alu #(
		.REG_WIDTH (REG_WIDTH)
	) u_alu_0 (
		.in1(alu_in1),
		.in2(alu_in2),
		.alu_control(alu_control),
		.result(alu_result),
		.zero(alu_zero),
		.sign(alu_sign)
	);
    // DMEM
	dmem #(
		.DMEM_DEPTH(DMEM_DEPTH),
		.DMEM_ADDR_WIDTH(DMEM_ADDR_WIDTH)
	) u_dmem_0(
		.clk(clk),
		.addr(dmem_addr),
		.din(dmem_din),
		.mem_read(mem_read),
		.mem_write(mem_write),
		.dout(dmem_dout)
	);
endmodule
