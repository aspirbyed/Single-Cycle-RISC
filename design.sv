// ---------------------------------------------------------------------------------
 
// Verilog code for instruction memory
 module instrmemory
 (  
    input           [15:0]          pc,  
    output wire     [15:0]          instruction  
 );  
   //wire [3:0] rom_addr = pc[4:1];  
 
   	reg [15:0] rom[255:0];  
   	initial  
    begin // $1 = 2; $2 =3; $3 = 0; $4 = 4; $5 = 5;
      rom[0] = 16'b0000001010011000;    //  $3 = $1 + $2 = 5;
      rom[1] = 16'b0000011001010000;    //  $2 = $3 + $1 = 7;
      rom[2] = 16'b0000001011010000;    //  $2 = $1 + $3 = 7;
      rom[3] = 16'b1000000000000010;    //  jump by 2 instr.;
      rom[4] = 16'b0101001011000000;    //  sw $3(=5) to [$1]+000000;
      rom[5] = 16'b0100001110000000;    //  lw [$1]+000000 to $6; 
      rom[6] = 16'b0000110011101000;    //  $5 = $6 + $3 = 10;
      end  
   assign instruction = rom[pc[4:1]];  
endmodule 

 // ---------------------------------------------------------------------------------

// Submodule: Data memory in Verilog 
module datamemory (input clk, we, 
                   input [15:0] a, wd, 
                   output [15:0] rd); 

  integer i;
  reg [15:0] RAM[255:0];
  initial begin  
      for(i=0;i<256;i=i+1)  
          RAM[i] <= 16'd0;  
  end
  assign rd=RAM[a[8:1]]; // word aligned 
    always @ (posedge clk) 
        if (we) 
          RAM[a[8:1]]=wd; 
endmodule

// ---------------------------------------------------------------------------------
 
// Verilog code for register file
 module regfile  
 (  
      input                      clk,  
      input                      rst,  
      // write port  
      input                      reg_write_en,  
      input          [2:0]       reg_write_dest,  
      input          [15:0]      reg_write_data,  
      //read port 1  
      input          [2:0]       reg_read_addr_1,  
      output         [15:0]      reg_read_data_1,  
      //read port 2  
      input          [2:0]       reg_read_addr_2,  
      output         [15:0]      reg_read_data_2
 );  
      reg            [15:0]      reg_array [7:0];  
      
      always @ (posedge clk or posedge rst) begin 
        $display("reg_read_addr_1 : %d reg_read_addr_2 : %d",reg_read_addr_1,reg_read_addr_2); 
        $display("reg_array[reg_read_addr_1] : %d reg_array[reg_read_addr_2] : %d",reg_array[reg_read_addr_1],reg_array[reg_read_addr_2]); 
           if(rst) begin  
                reg_array[0] <= 16'b0;  
             reg_array[1] <= 16'd2;  
             reg_array[2] <= 16'd3;  
                reg_array[3] <= 16'b0;  
             reg_array[4] <= 16'd4;  
             reg_array[5] <= 16'd5;  
                reg_array[6] <= 16'b0;  
                reg_array[7] <= 16'b0;       
           end  
           else begin  
                if(reg_write_en) begin 
                  $display("Write register got updated %d",reg_write_dest); 
                     reg_array[reg_write_dest] <= reg_write_data;  
                end  
           end  
           
      end  
    assign reg_read_data_1 = (reg_read_addr_1 == 0)? 16'b0 : reg_array[reg_read_addr_1];  
	assign reg_read_data_2 = (reg_read_addr_2 == 0)? 16'b0 : reg_array[reg_read_addr_2];
 endmodule
 
// ---------------------------------------------------------------------------------
  
  module alu(       
      input          [15:0]     a,          //src1  
      input          [15:0]     b,          //src2  
      input          [2:0]      alucontrol,     //function sel  
      output reg     [15:0]     result,          //result       
      output reg                zero  
   );  
 always @(*)
 begin   
      case(alucontrol)  
      3'b000: result = a + b; // add    
      3'b010: result = ~(a & b); // nand
      3'b100: result = a + b; // lw regB + immediate ig  
      3'b101: result = a + b; // sw regB + immediate ig   
      3'b110: begin
      		result = 16'd0;
      		zero = 1;
      end // branch equal to instruction
      3'b111: result = 16'd0; // jal instruction
        default: result = {16{1'b0}};
      endcase 
 end   
 endmodule  

  // ---------------------------------------------------------------------------------
  
  // Submodule: ALU Control Unit in Verilog 
 module aludec(aluop, alucontrol);  
 
 input           [3:0] aluop;  
 output reg      [2:0] alucontrol;  
    
 always @(aluop)  
 casex (aluop)  
  4'b0000: alucontrol=3'b000;  
  4'b0010: alucontrol=3'b010;  
  4'b0100: alucontrol=3'b100;  
  4'b0101: alucontrol=3'b101;  
  4'b1100: alucontrol=3'b110;  
  4'b1000: alucontrol=3'b111;      
  endcase
 endmodule
  
  // ---------------------------------------------------------------------------------
  
  // Main Decoder
  module maindec
  (
  	input      [3:0] op, 
  	output reg          regdst,
  	output reg          jump,  
  	output reg          branch,
  	//output           memread,
    output reg          memtoreg, 
    output     [3:0] aluop,
    output reg          memwrite, 
    output reg          alusrc, 
    output reg          regwrite 
    ); 
	
     assign aluop = op;

    always @ (*) 
    case(op) 
        4'b0000: begin //ADD
        	regdst = 1;
        	jump = 0; 
        	branch = 0;
        	//memread = 0;
        	memtoreg = 0;
        	memwrite = 0;
        	alusrc = 0;
        	regwrite = 1;
        end
        4'b0010: begin //NAND
        	regdst = 1;
        	jump = 0; 
        	branch = 0;
        	//memread = 0;
        	memtoreg = 0;
        	memwrite = 0;
        	alusrc = 0;
        	regwrite = 1;
        end 
        4'b0100: begin //LW
        	regdst = 0;
        	jump = 0; 
        	branch = 0;
        	//memread = 1;
        	memtoreg = 1;
        	memwrite = 0;
        	alusrc = 1;
        	regwrite = 1;
        end
        4'b0101: begin //SW
        	regdst = 0;
        	jump = 0; 
        	branch = 0;
        	//memread = 0;
        	memtoreg = 0;
        	memwrite = 1;
        	alusrc = 1;
        	regwrite = 0;
        end
        4'b1100: begin //BEQ
        	regdst = 0;
        	jump = 0; 
        	branch = 1;
        	//memread = 0;
        	memtoreg = 0;
        	memwrite = 0;
        	alusrc = 0;
        	regwrite = 0;
        end 
        4'b1000: begin //JAL
        	regdst = 0;
        	jump = 1; 
        	branch = 0;
        	//memread = 0;
        	memtoreg = 0;
        	memwrite = 0;
        	alusrc = 0;
        	regwrite = 0;
        end 
        default: begin //???
        	regdst = 1'bx;
        	jump = 1'bx; 
        	branch = 1'bx;
        	//memread = 1'bx;
        	memtoreg = 1'bx;
        	memwrite = 1'bx;
        	alusrc = 1'bx;
        	regwrite = 1'bx;
        end 
    endcase 
  endmodule

  // ---------------------------------------------------------------------------------
  
  // Submodule: Controller
  module controller 
  (
  	input      [3:0] op, 
    input            zero, 
    output           memtoreg, 
    //output           memread, 
    output 			 memwrite, 
    output           pcsrc, 
    output 			 alusrc, 
    output           regdst, 
    output           regwrite, 
    output           jump, 
    output     [2:0] alucontrol
  ); 

     wire [3:0] aluop; 
     wire branch; 

    maindec md (.op(op), .regdst(regdst), .jump(jump), .branch(branch), .memtoreg(memtoreg), 
     			.memwrite(memwrite), .alusrc(alusrc), .regwrite(regwrite), .aluop(aluop)); 
     aludec ad (.aluop(aluop), .alucontrol(alucontrol)); 
     
     assign pcsrc = branch & zero; 
  endmodule
  
   // ---------------------------------------------------------------------------------
   
   // Datapath
   module datapath (input clk, reset, 
                input memtoreg, pcsrc, 
                input alusrc, regdst, 
                input regwrite, jump, 
                input [2:0] alucontrol, 
                output zero, 
                output [15:0] pc, 
                input [15:0] instr, 
                output [15:0] aluout, writedata, 
                input [15:0] readdata); 

    wire [2:0] writereg; 
    wire [15:0] pcnext, pcnextbr, pcplus2, pcbranch; 
    wire [15:0] signimm, signimmsh; 
    wire [15:0] signtar, signtarsh;
    wire [15:0] pcjal;  
    wire [15:0] srca, srcb; 
    wire [15:0] result; 
	
	// pc = pcnext
	// pcplus2 = pc + 2
	// signimm = signextend to 16 bits instr[5:0]
	// signimmsh = shift left 1 of signimmsh
	// pcbranch = pcplus2 + signimmsh
	// signtar = signextend to 16 bits instr[9:0]
	// signtarsh = shift left 1 of signtarsh
	// pcjal = pcplus2 + signtarsh
	// if(pcsrc == 0) 
	// 		pcnextbr = pcplus2
	// else 
	// 		pcnextbr = pcbranch
	// if(jump == 0) 
	// 		pcnext = pcnextbr
	// else
	// 		pcnext = pcjal

// next PC logic 
    flopr #(16) pcreg(clk, reset, pcnext, pc); 
    adder pcadd1 (pc, 16'd2, pcplus2); 
    
    signextb se1(instr[5:0], signimm);
    sl1 immsh1(signimm, signimmsh); 
    adder pcadd2(pcplus2, signimmsh, pcbranch); //pcbrach
    
    signextj se2(instr[9:0], signtar);
    sl1 immsh2(signtar, signtarsh); 
    adder pcadd3(pcplus2, signtarsh, pcjal); //pcjal
    
    mux2 #(16) pcbrmux(pcplus2, pcbranch, pcsrc, pcnextbr); 
    mux2 #(16) pcmux(pcnextbr, pcjal,jump, pcnext); 

// register file logic 
     regfile rf(clk, reset, regwrite, writereg, result, instr[11:9], srca, instr[8:6], writedata); 
     mux2 #(3) wrmux(instr[8:6], instr[5:3], 
    regdst, writereg); 
    mux2 #(16) resmux(aluout, readdata, 
    memtoreg, result);  

// ALU logic 
    mux2 #(16) srcbmux(writedata, signimm, alusrc,srcb); 
    alu alu(srca, srcb, alucontrol, aluout, zero); 
endmodule
  
  // ---------------------------------------------------------------------------------
  
  // MIPS sub-module
  module mips(input clk, reset, 
            output [15:0] pc, 
            input [15:0] instr, 
            output memwrite, 
            output [15:0] aluout, writedata, 
            input [15:0] readdata); 

   wire memtoreg, pcsrc, alusrc, regdst, regwrite, jump, zero; 
   wire [2:0] alucontrol; 

   controller c(.op(instr[15:12]), .zero(zero), .regdst(regdst), .jump(jump), .pcsrc(pcsrc), .memtoreg(memtoreg), 
   				.alucontrol(alucontrol), .memwrite(memwrite), .alusrc(alusrc), .regwrite(regwrite));

   datapath dp(.clk(clk), .reset(reset), .memtoreg(memtoreg), .pcsrc(pcsrc), .alusrc(alusrc), .regdst(regdst),
    			.regwrite(regwrite), .jump(jump), .alucontrol(alucontrol), .zero(zero), .pc(pc), .instr(instr), 
    			.aluout(aluout), .writedata(writedata), .readdata(readdata)); 

  endmodule
 
  //---------------------------------------------------------------------------------
  
  // Sign extension module fro branch
module signextb (input [5:0] a,output [15:0] y); 
    assign y={{10{a[5]}}, a}; 
  endmodule
  
  //---------------------------------------------------------------------------------
  
  // Sign extension module for jal
module signextj (input [9:0] a,output [15:0] y); 
    assign y={{7{a[9]}}, a}; 
  endmodule 
  
  //---------------------------------------------------------------------------------
  
  // D Flip-Flop with variable width
  module flopr # (parameter WIDTH = 8)(input clk, reset,input [WIDTH-1:0] d, output reg [WIDTH-1:0] q); 
    always @ (posedge clk or posedge reset) 
        if (reset) q<=0; 
        else q <= d; 
  endmodule
  
  //---------------------------------------------------------------------------------
  
  // Shift left by 1
  module sl1 (input [15:0] a, 
	output [15:0] y); 
    assign y = {a[14:0], 1'b0}; 
  endmodule
  
  //---------------------------------------------------------------------------------
  
  // Adder
  module adder (input [15:0] a, b,output [15:0] y); 
    assign y = a + b; 
  endmodule
  
  //---------------------------------------------------------------------------------
  
  // Mux 2:1
  module mux2 # (parameter WIDTH = 8)(input [WIDTH-1:0] d0, d1,input s,output [WIDTH-1:0] y); 
	assign y = s ? d1 : d0; 
  endmodule
  
  //---------------------------------------------------------------------------------
  
  // Top Level Module
  module top 
  (
    input clk, reset, 
    output [15:0] writedata, dataadr, 
    output memwrite
  ); 
    wire [15:0] pc, instr, readdata; 

    // instantiate processor and memories 
    mips mips (clk, reset, pc, instr, memwrite, dataadr, writedata, readdata); 
    instrmemory imem (pc, instr);
    datamemory dmem (clk, memwrite, dataadr, writedata,readdata); 
  endmodule
  
  //---------------------------------------------------------------------------------
