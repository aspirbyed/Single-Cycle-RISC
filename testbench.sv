 module tb_mips16;  
      // Inputs  
      reg clk;  
      reg reset;  
      // Outputs  
   wire [15:0] writedata;  
      wire [15:0] dataadr;
   wire memwrite;
      // Instantiate the Unit Under Test (UUT)  
      top uut (  
           .clk(clk),   
           .reset(reset),   
           .writedata(writedata),
        .dataadr(dataadr),
        .memwrite(memwrite)
      );  
      initial begin  
           clk = 0;  
           forever #10 clk = ~clk;  
      end  
      initial begin  
           // Initialize Inputs  
           //$monitor ("register 3=%d, register 4=%d", reg3,reg4);  
        $dumpfile("tb_mips16.vcd");
        $dumpvars(0,tb_mips16);
           reset <= 1;  
           #40;  
           // Wait 40 ns for global reset to finish  
		   reset <= 0;  
        
           //$dumpfile("tb_mips16.vcd");
  	       //$dumpvars(0, tb_mips16);

           #100 $display("FIN de la simulacion");
           $finish;
            
      end  
   
 endmodule
/*
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
*/
