module LED(clk,rst,L);

input clk,rst;
output [3:0]L;

//50MHz
parameter T1S=27'd50000000;
// 改变定义的参数T1S的大小，更改每位LED灯亮的时间

reg [3:0]led;
reg [31:0]C1;

always @(posedge clk,negedge rst)

if(!rst)   
	begin
		led[3:0]<=4'b1000; 
		C1<=32'd0; 
	end

else if(C1==T1S)         
	begin
		led<={led[0],
		led[3:1]}; 
		C1<=32'd0; 
	end
    
else 
	begin 
		led<=led; 
		C1<=C1+1'b1; 
	end

assign L = ~led;

endmodule