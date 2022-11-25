`timescale 1ns/10ps
module IOTDF( clk, rst, in_en, iot_in, fn_sel, busy, valid, iot_out);

input          clk;
input          rst;
input          in_en;
input  [7:0]   iot_in;
input  [2:0]   fn_sel;
output         busy;
output         valid;
output [127:0] iot_out;

//parameter
parameter IDLE = 2'b00;
parameter FET = 2'b01;
parameter WB = 2'b10;

parameter iot_in_WIDTH = 8;
parameter LOW_EXT = 128'h6FFF_FFFF_FFFF_FFFF_FFFF_FFFF_FFFF_FFFF;
parameter HIGH_EXT = 128'hAFFF_FFFF_FFFF_FFFF_FFFF_FFFF_FFFF_FFFF;
parameter LOW_EXC = 128'h7FFF_FFFF_FFFF_FFFF_FFFF_FFFF_FFFF_FFFF;
parameter HIGH_EXC = 128'hBFFF_FFFF_FFFF_FFFF_FFFF_FFFF_FFFF_FFFF;


//reg & wire
reg [ 1 : 0 ] state, state_n;
reg busy_r, valid_r;
reg [ 130 : 0 ] iot_out_r;
reg [ 3 : 0 ] cnt;
reg [ 2 : 0 ] p_cnt;
wire [ 6 : 0 ] digit_start;
//wire [ 6 : 0 ] digit_end;
reg [ 130 : 0 ] tmp;

//F1 F2
wire [127:0] max_min_data;
//F3
wire [130:0] avg_data;
//F4 F5
wire [127:0] ext_exc_data;
//F6 F7
reg peak_standard_stored;
reg [127:0] peak_data;


//output assignment
assign busy = busy_r;
assign valid = valid_r;
assign iot_out = (cnt==0 && p_cnt==0) ? ((fn_sel==1 || fn_sel==2) ? iot_out_r : ((fn_sel==3) ? avg_data[127:0] : 
((fn_sel==6 || fn_sel==7) ? peak_data : ((fn_sel==4 || fn_sel==5) ? ext_exc_data : 0)))) : 0;
assign digit_start = 127 - (cnt<<3);
//assign digit_end = 120 - (cnt<<3);
//assign digit_end = digit_start - 7;

//F1 F2 assignment
assign max_min_data = (cnt==0 && p_cnt==1 && state==FET) ? tmp[127:0] :
(((fn_sel==1 || fn_sel==6) ? ((cnt==0 && tmp[127:0]>iot_out_r[127:0]) ? tmp[127:0] : iot_out_r[127:0] ) : 
((cnt==0 && tmp[127:0]<iot_out_r[127:0]) ? tmp[127:0] : iot_out_r[127:0] )));

//F3 assignment
assign avg_data = (p_cnt==0 && cnt==0) ? iot_out_r>>3 : 0;

//F4 F5 assignment
assign ext_exc_data = (cnt==0) ? ((fn_sel==3'd4) ? ((HIGH_EXT>tmp[127:0] && tmp[127:0]>LOW_EXT) ? tmp[127:0] : 0 ) : 
((tmp[127:0]>HIGH_EXC || LOW_EXC>tmp[127:0]) ? tmp[127:0] : 0 ) ) : 0;


//FSM
always@(*)begin
    case(state)
        IDLE: state_n = FET;
        FET: state_n = ( cnt==4'd15 ) ? WB : FET;
        WB: state_n = FET;
        default: state_n = IDLE;
    endcase
end

//Sequential
always@(posedge clk)begin
    if(rst)begin
        busy_r <= 0;
        valid_r <= 0;
        iot_out_r <= 0;
        state <= IDLE;
        tmp <= 0;
        cnt <= 0;
        p_cnt <= 0;
        peak_standard_stored <= 0;
        peak_data <= 0;
    end
    else begin
        state <= state_n;
        case (state)
           FET:begin
               tmp[digit_start -: iot_in_WIDTH] <= iot_in;
               cnt <= (cnt==4'd15) ? 0 : cnt + 1'b1;
               busy_r <= (cnt==4'd14) ? 1'b1 : 1'b0;
               valid_r <= 0;
               
               if(fn_sel==3'b011)
               iot_out_r <= (cnt==0 && p_cnt==0)? 0 : iot_out_r;
               else
               iot_out_r <= (cnt==0) ? max_min_data : iot_out_r;
           end 
           WB:begin
               if(fn_sel==3'b001||fn_sel==3'b010)begin
                   iot_out_r[127:0] <= max_min_data;
                   p_cnt <= (p_cnt==3'd7) ? 0 : p_cnt + 1'b1;
                   valid_r <= p_cnt==3'd7 ? 1'b1 : 1'b0;
                   tmp <= (p_cnt==3'd7) ? 0 : tmp;
               end
               if(fn_sel==3'b011)begin
                   iot_out_r <= iot_out_r + tmp;
                   p_cnt <= (p_cnt==3'd7) ? 0 : p_cnt + 1'b1;
                   valid_r <= p_cnt==3'd7 ? 1'b1 : 1'b0;
                   tmp <= (p_cnt==3'd7) ? 0 : tmp;
               end
               if(fn_sel==3'b100||fn_sel==3'b101)begin
                   valid_r <= ext_exc_data>0 ? 1'b1 : 1'b0;
                   tmp <= p_cnt==3'd7 ? 0 : tmp;
               end
               if(fn_sel==3'b110)begin
                   iot_out_r[127:0] <= max_min_data;
                   p_cnt <= (p_cnt==3'd7) ? 0 : p_cnt + 1'b1;
                   if(peak_standard_stored==0 && p_cnt==3'd7)
                   valid_r <= 1'b1;
                   else
                   valid_r <= (p_cnt==7 && max_min_data>peak_data) ? 1'b1 : 1'b0;

                   peak_data <= (p_cnt==7 && max_min_data>peak_data) ? max_min_data : peak_data;
                   tmp <= (p_cnt==3'd7) ? 0 : tmp;
                   if(p_cnt==3'd7)
                   peak_standard_stored <= 1'b1;
               end
               if(fn_sel==3'b111)begin
                   iot_out_r[127:0] <= max_min_data;
                   p_cnt <= (p_cnt==3'd7) ? 0 : p_cnt + 1'b1;
                   if(peak_standard_stored==0 && p_cnt==3'd7)
                       valid_r <= 1'b1;
                   else
                   valid_r <= (p_cnt==7 && max_min_data<peak_data) ? 1'b1 : 1'b0;
                   
                   peak_data <= (p_cnt==3'd7 && peak_standard_stored==0) ? max_min_data : 
                   ((p_cnt==7 && max_min_data<peak_data) ? max_min_data : peak_data);
                   tmp <= (p_cnt==3'd7) ? 0 : tmp;
                   if(p_cnt==3'd7)
                   peak_standard_stored <= 1'b1;
               end
           end
        endcase
    end
end
endmodule
