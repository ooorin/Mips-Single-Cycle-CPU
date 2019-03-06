`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2018/04/02 16:00:03
// Design Name: 
// Module Name: test
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////
parameter wid = 32;

module ALU(
	input logic [2:0] aluCtrl,
	input logic [wid - 1:0] a, [wid - 1:0] b,
	
	output logic [wid - 1:0] result,
	output logic cf, ovf, zf, nf
    );

	logic [wid:0] r;

	always_comb
	begin
		case(aluCtrl)
			3'b000: r = a & b;
			3'b001: r = a | b;
			3'b010: r = a + b;
			3'b110: r = a - b;
			3'b111: r = a < b ? 32'hFFFFFFFF : 32'b0; // SLT
			default: r = r;
		endcase
	end

	assign result = r[wid - 1:0];
	assign cf = r[wid];
	assign ovf = r[wid];
	assign zf = (r == 0);
	assign nf = r[wid - 1];

endmodule

module PC( 
	input logic clk,
	input logic rst,
	input logic [wid - 1:0] pc1,

	output logic [wid - 1:0] pc2
	);

	logic [wid - 1:0] pc;

	always_ff @(posedge clk)
	begin
		if (rst) pc <= 0;
		else pc <= pc1;
	end

	assign pc2 = pc;

endmodule

module IMEM(
	input logic [5:0] pc,

	output logic [wid - 1:0] instr // length of instruction
	);

	logic [wid - 1:0] rom [63:0];

	initial
		begin
			$readmemh("C:/Users/10441/Desktop/project_1/project_1.srcs/sources_1/new/iMemFile2.dat", rom); // read instruction_memory file (hex) readmemb-binary
		end
	
	assign instr = rom[pc];

endmodule

module DMEM(
	input logic clk, rst, writeEn,
	input logic [wid - 1:0] addr,
	input logic [wid - 1:0] writeData, // write data

	output logic [wid - 1:0] readData // read data
	);

	logic [wid - 1:0] ram [63:0];

	integer i;

	/*initial
	begin
		for (i = 0; i < 64; i = i + 1)
			ram[i] = 32'b0;
	end
	*/
	always_ff @(posedge clk)
	begin
		if (rst)
		begin
			for (i = 0; i < 64; i = i + 1)
			ram[i] = 32'b0;
		end
		else if (writeEn) ram[addr[31:2]] <= writeData;
	end

	assign readData = ram[addr[31:2]];

endmodule

module REGFILE(
	input logic clk,
	input logic rst,
	input logic regWriteEn,

	input logic [4:0] regWriteAddr,
	input logic [wid - 1:0] regWriteData,

	input logic [4:0] rsAddr,
	input logic [4:0] rtAddr,

	output logic [wid - 1:0] rsData,
	output logic [wid - 1:0] rtData
	);

	logic [wid - 1:0] rf [wid - 1:0];

	integer i;
	
	always_ff @(posedge clk)
	begin
		if (rst)
		begin
			for (i = 0; i < 32; i = i + 1)
				rf[i] = 32'b0;
		end
		else if (regWriteEn) rf[regWriteAddr] <= regWriteData;
	end

	assign rsData = (rsAddr != 0) ? rf[rsAddr] : 0;
	assign rtData = (rtAddr != 0) ? rf[rtAddr] : 0;

endmodule

module SIGNEXT(
	input logic [wid / 2 - 1:0] in,

	output logic [wid - 1:0] out
	);

	assign out = {{(wid / 2){in[wid / 2 - 1]}}, in};

endmodule

module MUX2 #(parameter w = 32)
	(
	input logic sw,
	input logic [w - 1:0] in1, in2,

	output logic [w - 1:0] out
	);

	logic [w - 1:0] y;

	always_comb
	begin
		if (sw == 1) y = in2;
		else y = in1;
	end

	assign out = y;

endmodule

module ADDER(
	input logic [wid - 1:0]a, b,

	output logic [wid - 1:0] sum
	);

	assign sum = a + b;

endmodule
/*
module SHIFT(
	input logic lr, // 0-left, 1-right
	input logic al, // a-algorthm-1, l-logic-0
	input logic [wid - 1:0] in,
	input logic [4:0] sp, // step

	output logic [wid - 1:0] out
	);

	logic [wid - 1:0] y;
	logic [4:0] i;
	
	always_comb
	begin
		y = in;
		if (lr == 0)
		begin
			for (i = 0; i < sp; i = i + 1)
				y = {y[wid - 2:0], al};
		end
		else
		begin
			for (i = 0; i < sp; i = i + 1)
				y = {al, y[wid - 1:1]};
		end
	end

	assign out = y;

endmodule
*/
module SHIFT(
	input logic [wid - 1:0] in,

	output logic [wid - 1:0] out
	);

	logic [wid - 1:0] y;

	assign y = {in[wid - 3:0], 2'b00};
	assign out = y;

endmodule

module MAINDEC(
	input logic [5:0] op,

	output logic memToReg, memWrite,
	output logic branch0, branch1, aluSrc,
	output logic regDst, regWrite,
	output logic jump,
	output logic [2:0]aluOp // 000 +, 001 -, 010 &, 011 |, 100 Rtype, 101 slt
	);

	logic [10:0] controls;

	assign {regWrite, regDst, aluSrc, branch0, branch1, memWrite, memToReg, jump, aluOp} = controls;

	always_comb
	begin
		case(op)
			6'b000000: controls = 11'b11000000100; // Rtype
			6'b100011: controls = 11'b10100010000; // LW read mem
			6'b101011: controls = 11'b00100100000; // SW write mem
			6'b000100: controls = 11'b00010000001; // BEQ
			6'b000101: controls = 11'b00001000001; // BNE
			6'b001000: controls = 11'b10100000000; // ADDI
			6'b001100: controls = 11'b10100000010; // ANDI
			6'b001101: controls = 11'b10100000011; // ORI
			6'b001010: controls = 11'b10100000101; // SLTI
			6'b000010: controls = 11'b00000001000; // J
			default:   controls = 11'bxxxxxxxxxxx;
		endcase
	end

endmodule

module ALUDEC(
	input logic [5:0] func,
	input logic [2:0] aluOp,

	output logic [2:0] aluCtrl
	);

	always_comb
	begin
		case(aluOp)
			3'b000: aluCtrl = 3'b010;
			3'b001: aluCtrl = 3'b110;
			3'b010: aluCtrl = 3'b000;
			3'b011: aluCtrl = 3'b001;
			3'b101: aluCtrl = 3'b111;
			default: 
				case(func)
					6'b000000: aluCtrl = 3'b010; // NOP
					6'b100000: aluCtrl = 3'b010; // ADD
					6'b100010: aluCtrl = 3'b110; // SUB
					6'b100100: aluCtrl = 3'b000; // AND
					6'b100101: aluCtrl = 3'b001; // OR
					6'b101010: aluCtrl = 3'b111; // SLT
					default:   aluCtrl = 3'bxxx;
				endcase
		endcase
	end

endmodule

module CONTROLLER(
	input logic [5:0] op, func,
	input logic zero,

	output logic memToReg, memWrite,
	output logic pcSrc, aluSrc,
	output logic regDst, regWrite,
	output logic jump,
	output logic [2:0] aluCtrl
	);

	logic branch0, branch1;
	logic [2:0]aluOp;

	MAINDEC mD(op, memToReg, memWrite, branch0, branch1, aluSrc, regDst, regWrite, jump, aluOp);
	ALUDEC aD(func, aluOp, aluCtrl);

	assign pcSrc = (branch0 & zero) | (branch1 & (~zero));

endmodule

module DATAPATH(
	input logic clk, rst,
	input logic memToReg, pcSrc,
	input logic aluSrc, regDst,
	input logic regWrite, jump,
	input logic [2:0] aluCtrl,
	input logic [wid - 1:0] instr,
	input logic [wid - 1:0] readData,

	output logic cf, ovf, zf, nf,
	output logic [wid - 1:0] pc,
	output logic [wid - 1:0] aluOut, writeMemData
	);

	logic [4:0] writeReg;
	logic [wid - 1:0] pcNext, pcNextBr, pcPlus4, pcBranch;
	logic [wid - 1:0] sigImm, sigImmSh;
	logic [wid - 1:0] srcA, srcB;
	logic [wid - 1:0] writeRegData;

	// pc next
	PC pcReg(clk, rst, pcNext, pc);
	ADDER pcAdd1(pc, 32'b100, pcPlus4);
	//SHIFT immSh(1'b0, 1'b0, sigImm, 5'b00010, sigImmSh);
	SHIFT immSh(sigImm, sigImmSh);
	ADDER pcAdd2(pcPlus4, sigImmSh, pcBranch);
	MUX2 pcBrMux(pcSrc, pcPlus4, pcBranch, pcNextBr);
	MUX2 pcMux(jump, pcNextBr, {pcPlus4[31:28], instr[25:0], 2'b00}, pcNext);
	
	// register file
	REGFILE rf(clk, rst, regWrite, writeReg, writeRegData, instr[25:21], instr[20:16], srcA, writeMemData); // writeMemData rt
	MUX2 #(5) wrMux(regDst, instr[20:16], instr[15:11], writeReg); // rt/rd -> writeReg
	MUX2 resMux(memToReg, aluOut, readData, writeRegData);
	SIGNEXT sgExt(instr[15:0], sigImm);

	// ALU
	MUX2 srcBrMux(aluSrc, writeMemData, sigImm, srcB); // rt/imm -> rt
	ALU alu(aluCtrl, srcA, srcB, aluOut, cf, ovf, zf, nf);

endmodule 

module MIPS(
	input logic clk, rst,
	input logic [wid - 1:0] instr,
	input logic [wid - 1:0] readData,

	output logic [wid - 1:0] pc,
	output logic memWrite,
	output logic [wid - 1:0] aluOut, writeMemData
	);

	logic memToReg, pcSrc, cf, ovf, zf, nf, aluSrc, regDst, regWrite, jump;
	logic [2:0] aluCtrl;

	CONTROLLER ctrl(instr[31:26], instr[5:0], zf,
					memToReg, memWrite, pcSrc,
					aluSrc, regDst, regWrite, jump,
					aluCtrl);

	DATAPATH dpath(clk, rst, memToReg, pcSrc, 
				   aluSrc, regDst, regWrite, jump,
				   aluCtrl, instr, readData,
				   cf, ovf, zf, nf, pc,
				   aluOut, writeMemData);

endmodule

module TOP(
	input logic clk, rst,

	output logic [wid - 1:0] writeMemAddr, writeMemData,
	output logic memWrite
	);

	logic [wid - 1:0] pc, instr, readData;

	MIPS mips(clk, rst, instr, readData, pc, memWrite, writeMemAddr, writeMemData);
	IMEM imem(pc[7:2], instr);
	DMEM dmem(clk, rst, memWrite, writeMemAddr, writeMemData, readData);

endmodule
/*
module TOP(
	input logic clk, rst,

	output logic [wid - 1:0] writeMemAddr, writeMemData,
	output logic memWrite,

	output logic [7:0] pcShow, rdDatShow, wMemAdrShow, wMemDatShow, // hhh
	output logic memWriteShow // hhh
	);

	logic [wid - 1:0] pc, instr, readData;

	MIPS mips(clk, rst, instr, readData, pc, memWrite, writeMemAddr, writeMemData);
	IMEM imem(pc[7:2], instr);
	DMEM dmem(clk, rst, memWrite, writeMemAddr, writeMemData, readData);

	assign pcShow = pc[7:0]; // hhh
	assign rdDatShow = readData[7:0]; // hhh
	assign wMemAdrShow = writeMemAddr[7:0]; // hhh
	assign wMemDatShow = writeMemData[7:0]; // hhh
	assign memWriteShow = memWrite; // hhh

endmodule

// test
module clkDiv(
	input logic clk,

	output logic clk190,
	output logic clk48,
	output logic clk1_4
	);

	logic [27:0] q;

	initial
	begin
		q = 0;
	end

	always_ff @(posedge clk)
	begin
		q <= q + 1;
	end

	assign clk190 = q[18];
	assign clk48 = q[20];
	assign clk1_4 = q[26];

endmodule

module forShow(
	input logic clk, rst,
	input logic k,

	output logic [6:0] a2g,
	output logic [7:0] enA2g,
	output logic dot
	);

	logic clk190, clk48, clk1_4;

	clkDiv div(clk, clk190, clk48, clk1_4);

	logic [wid - 1:0] writeMemAddr, writeMemData;
	logic memWrite;
	logic [31:0] sw; 

	assign clk1_4_k = clk1_4 & k;

	TOP top(clk1_4_k, rst, writeMemAddr, writeMemData, memWrite, 
			sw[31:24], sw[23:16], sw[15:8], sw[7:0], dot);

	logic [2:0] i;
	logic [20:0] fre;

	initial
	begin
		i = 0;
		fre = 0;
	end

	always_ff @(posedge clk)
	begin
		fre <= fre + 1;
	end

	assign i = fre[19:17];

	always_comb
	begin
		if (i == 3'b000)
		begin
			enA2g = 8'b1111_1110;
			case (sw[3:0])
			4'b0000: a2g <= 7'b1000000;
			4'b0001: a2g <= 7'b1111001;
			4'b0010: a2g <= 7'b0100100;
			4'b0011: a2g <= 7'b0110000;
			4'b0100: a2g <= 7'b0011001;
			4'b0101: a2g <= 7'b0010010;
			4'b0110: a2g <= 7'b0000010;
			4'b0111: a2g <= 7'b1111000;
			4'b1000: a2g <= 7'b0000000;
			4'b1001: a2g <= 7'b0010000;
			4'b1010: a2g <= 7'b0001000;
			4'b1011: a2g <= 7'b0000011;
			4'b1100: a2g <= 7'b1000110;
			4'b1101: a2g <= 7'b0100001;
			4'b1110: a2g <= 7'b0001110;
			4'b1111: a2g <= 7'b0001110;
			default: a2g <= 7'b1000000;
			endcase
		end
		else if (i == 3'b001)
		begin
			enA2g = 8'b1111_1101;
			case (sw[7:4])
			4'b0000: a2g <= 7'b1000000;
			4'b0001: a2g <= 7'b1111001;
			4'b0010: a2g <= 7'b0100100;
			4'b0011: a2g <= 7'b0110000;
			4'b0100: a2g <= 7'b0011001;
			4'b0101: a2g <= 7'b0010010;
			4'b0110: a2g <= 7'b0000010;
			4'b0111: a2g <= 7'b1111000;
			4'b1000: a2g <= 7'b0000000;
			4'b1001: a2g <= 7'b0010000;
			4'b1010: a2g <= 7'b0001000;
			4'b1011: a2g <= 7'b0000011;
			4'b1100: a2g <= 7'b1000110;
			4'b1101: a2g <= 7'b0100001;
			4'b1110: a2g <= 7'b0001110;
			4'b1111: a2g <= 7'b0001110;
			default: a2g <= 7'b1000000;
			endcase
		end
		else if (i == 3'b010)
		begin
			enA2g = 8'b1111_1011;
			case (sw[11:8])
			4'b0000: a2g <= 7'b1000000;
			4'b0001: a2g <= 7'b1111001;
			4'b0010: a2g <= 7'b0100100;
			4'b0011: a2g <= 7'b0110000;
			4'b0100: a2g <= 7'b0011001;
			4'b0101: a2g <= 7'b0010010;
			4'b0110: a2g <= 7'b0000010;
			4'b0111: a2g <= 7'b1111000;
			4'b1000: a2g <= 7'b0000000;
			4'b1001: a2g <= 7'b0010000;
			4'b1010: a2g <= 7'b0001000;
			4'b1011: a2g <= 7'b0000011;
			4'b1100: a2g <= 7'b1000110;
			4'b1101: a2g <= 7'b0100001;
			4'b1110: a2g <= 7'b0001110;
			4'b1111: a2g <= 7'b0001110;
			default: a2g <= 7'b1000000;
			endcase
		end
		else if (i == 3'b011)
		begin
			enA2g = 8'b1111_0111;
			case (sw[15:12])
			4'b0000: a2g <= 7'b1000000;
			4'b0001: a2g <= 7'b1111001;
			4'b0010: a2g <= 7'b0100100;
			4'b0011: a2g <= 7'b0110000;
			4'b0100: a2g <= 7'b0011001;
			4'b0101: a2g <= 7'b0010010;
			4'b0110: a2g <= 7'b0000010;
			4'b0111: a2g <= 7'b1111000;
			4'b1000: a2g <= 7'b0000000;
			4'b1001: a2g <= 7'b0010000;
			4'b1010: a2g <= 7'b0001000;
			4'b1011: a2g <= 7'b0000011;
			4'b1100: a2g <= 7'b1000110;
			4'b1101: a2g <= 7'b0100001;
			4'b1110: a2g <= 7'b0001110;
			4'b1111: a2g <= 7'b0001110;
			default: a2g <= 7'b1000000;
			endcase
		end
		else if (i == 3'b100)
		begin
			enA2g = 8'b1110_1111;
			case (sw[19:16])
			4'b0000: a2g <= 7'b1000000;
			4'b0001: a2g <= 7'b1111001;
			4'b0010: a2g <= 7'b0100100;
			4'b0011: a2g <= 7'b0110000;
			4'b0100: a2g <= 7'b0011001;
			4'b0101: a2g <= 7'b0010010;
			4'b0110: a2g <= 7'b0000010;
			4'b0111: a2g <= 7'b1111000;
			4'b1000: a2g <= 7'b0000000;
			4'b1001: a2g <= 7'b0010000;
			4'b1010: a2g <= 7'b0001000;
			4'b1011: a2g <= 7'b0000011;
			4'b1100: a2g <= 7'b1000110;
			4'b1101: a2g <= 7'b0100001;
			4'b1110: a2g <= 7'b0001110;
			4'b1111: a2g <= 7'b0001110;
			default: a2g <= 7'b1000000;
			endcase
		end
		else if (i == 3'b101)
		begin
			enA2g = 8'b1101_1111;
			case (sw[23:20])
			4'b0000: a2g <= 7'b1000000;
			4'b0001: a2g <= 7'b1111001;
			4'b0010: a2g <= 7'b0100100;
			4'b0011: a2g <= 7'b0110000;
			4'b0100: a2g <= 7'b0011001;
			4'b0101: a2g <= 7'b0010010;
			4'b0110: a2g <= 7'b0000010;
			4'b0111: a2g <= 7'b1111000;
			4'b1000: a2g <= 7'b0000000;
			4'b1001: a2g <= 7'b0010000;
			4'b1010: a2g <= 7'b0001000;
			4'b1011: a2g <= 7'b0000011;
			4'b1100: a2g <= 7'b1000110;
			4'b1101: a2g <= 7'b0100001;
			4'b1110: a2g <= 7'b0001110;
			4'b1111: a2g <= 7'b0001110;
			default: a2g <= 7'b1000000;
			endcase
		end
		else if (i == 3'b110)
		begin
			enA2g = 8'b1011_1111;
			case (sw[27:24])
			4'b0000: a2g <= 7'b1000000;
			4'b0001: a2g <= 7'b1111001;
			4'b0010: a2g <= 7'b0100100;
			4'b0011: a2g <= 7'b0110000;
			4'b0100: a2g <= 7'b0011001;
			4'b0101: a2g <= 7'b0010010;
			4'b0110: a2g <= 7'b0000010;
			4'b0111: a2g <= 7'b1111000;
			4'b1000: a2g <= 7'b0000000;
			4'b1001: a2g <= 7'b0010000;
			4'b1010: a2g <= 7'b0001000;
			4'b1011: a2g <= 7'b0000011;
			4'b1100: a2g <= 7'b1000110;
			4'b1101: a2g <= 7'b0100001;
			4'b1110: a2g <= 7'b0001110;
			4'b1111: a2g <= 7'b0001110;
			default: a2g <= 7'b1000000;
			endcase
		end
		else if (i == 3'b111)
		begin
			enA2g = 8'b0111_1111;
			case (sw[31:28])
			4'b0000: a2g <= 7'b1000000;
			4'b0001: a2g <= 7'b1111001;
			4'b0010: a2g <= 7'b0100100;
			4'b0011: a2g <= 7'b0110000;
			4'b0100: a2g <= 7'b0011001;
			4'b0101: a2g <= 7'b0010010;
			4'b0110: a2g <= 7'b0000010;
			4'b0111: a2g <= 7'b1111000;
			4'b1000: a2g <= 7'b0000000;
			4'b1001: a2g <= 7'b0010000;
			4'b1010: a2g <= 7'b0001000;
			4'b1011: a2g <= 7'b0000011;
			4'b1100: a2g <= 7'b1000110;
			4'b1101: a2g <= 7'b0100001;
			4'b1110: a2g <= 7'b0001110;
			4'b1111: a2g <= 7'b0001110;
			default: a2g <= 7'b1000000;
			endcase
		end
	end

endmodule

module forShowTop(
	input logic CLK100MHZ, SW[1:0],

	output logic [6:0] A2G,
	output logic [7:0] AN,
	output logic DP
	);
	
	logic d;

	forShow show(CLK100MHZ, SW[0], SW[1], A2G[6:0], AN[7:0], d);

	assign DP = ~d;

endmodule
*/