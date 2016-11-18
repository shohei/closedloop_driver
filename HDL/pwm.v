module pwm(
	CLK,RST,SW,MD1,MD2
    );

input CLK;
input RST;
input [7:0] SW;
output MD1;
output MD2;

reg [7:0] sw_reg_1d;
reg [7:0] sw_reg_2d;
reg [10:0] pwm_base_reg;
reg [10:0] pwm_duty_reg;
reg pwm_out_reg;

wire pwmbp;
wire pwm_out;

assign pwmbp = pwm_base_reg[10];
assign pwm_out = (pwm_duty_reg[10:3] == sw_reg_2d) ? 1'b0 : 1'b1;
assign MD1 = pwm_out_reg;
assign MD2 = 1'b1;


always @(posedge CLK) begin
	sw_reg_1d <= SW;
	sw_reg_2d <= sw_reg_1d;
end

always @(posedge CLK) begin
	if(pwmbp == 1'b1) begin
		pwm_base_reg <= 11'h000;
	end
	else if(RST)
		pwm_base_reg <= 11'h0;
	else begin
		pwm_base_reg <= pwm_base_reg + 1'b1;
	end
end


always @(posedge CLK) begin
	if(pwmbp == 1'b1) begin
		pwm_duty_reg <= 11'h000;
	end
	else if(RST)
		pwm_duty_reg <= 11'h0;
	else if(pwm_out==1'b1)  begin
		pwm_duty_reg <= pwm_duty_reg + 1'b1;
	end
end


always @(posedge CLK) begin
	pwm_out_reg <= pwm_out;
end



endmodule
