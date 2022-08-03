
module agc_controller #(
	parameter TIMER_CNT_W = 32,
	parameter AGC_TMR_Value_1ms = 8000,
	parameter AGC_TMR_Value_10ms = 80000,
	parameter AGC_TMR_Value_100ms = 800000,
	parameter AGC_TMR_Value_10us = 80,
	parameter CONFIG_WORD_W = 11,
	parameter CONFIG_WORD_UPBOUND = 11'h1ff,
	parameter CONFIG_WORD_LOWBOUND = 11'h0,
	parameter UNDER_ATTACK_NUM = 5
	
)(
	input wire	clk										,
	input wire	rst_n									,
    //interface for ScanChain
    input  wire	SCAN_MODE_IN	    ,
    input  wire	SCAN_CLK		    ,
    input  wire SCAN_RSTN		    ,    
	//analog input
	input wire  SQUARE_WAVE_I							,
	input wire	SQUARE_WAVE_Q							,
	input wire	VCOMPOUT_OV_I							,
	input wire	VCOMPOUT_UV_I							,
	input wire	VCOMPOUT_OV_Q							,
	input wire	VCOMPOUT_UV_Q							,
	//config word input
	input wire	AGC_Check_En							,
	input wire	[CONFIG_WORD_W-1:0]AGC_Plause_UThresh	,
	input wire	[CONFIG_WORD_W-1:0]AGC_Plause_LThresh	,
	input wire 	AGC_Dis									,
	input wire  AGC_Strategy_Sel						,
	input wire	[01:0]AGC_Tmr_Sel						,
	input wire	[CONFIG_WORD_W-1:0]AFE_Gain_Config		,
	//output
	input wire	AFE_Gain_Config_Modify					,
	output reg  [CONFIG_WORD_W-1:0]AFE_Gain_Active		,
	//output reg	AFE_Gain_Active_Modify					,
	output wire AGC_Plause_Error						,
	output wire Comparator_Out_Exception
);

reg	AFE_Gain_Active_Modify					;
reg [1:0]amp_state;
reg [3:0]s_attacker;
reg [2:0]under_attack_counter;
reg [TIMER_CNT_W-1:0]attacker_timer;

parameter AMP_UNKNOWN	= 2'b00;
parameter AMP_NORMAL	= 2'b01;
parameter AMP_UNDER 	= 2'b10;
parameter AMP_OVER 		= 2'b11;

parameter S_UNKNOWN		= 3'b000;
parameter S_PRE_DETECT 	= 3'b001;
parameter S_DETECT		= 3'b010;
parameter S_OVER		= 3'b011;
parameter S_UNDER		= 3'b100;
parameter S_NORMAL      = 3'b101;

parameter IDLE 			= 4'b0000;
parameter ATTACK_UNDER	= 4'b0001;
parameter ATTACK_OVER 	= 4'b0010;
parameter DELAY_PRE 	= 4'b0011;
parameter DELAY			= 4'b0100;
parameter DETECT_UNDER	= 4'b0101;
parameter DETECT_OVER	= 4'b0110;
parameter ATTACK_END_PRE= 4'b0111;
parameter ATTACK_END	= 4'b1000;


/*************************************************************************************/
//consistant time map
/*************************************************************************************/

//AGC_TMR_Value
reg [TIMER_CNT_W-1:0] AGC_TMR_Value;
always@(*) begin
	case(AGC_Tmr_Sel)
		2'b00	: AGC_TMR_Value = AGC_TMR_Value_1ms;
		2'b01	: AGC_TMR_Value = AGC_TMR_Value_10ms;
		2'b10	: AGC_TMR_Value = AGC_TMR_Value_100ms;
		2'b11	: AGC_TMR_Value = AGC_TMR_Value_10us;
		default	: AGC_TMR_Value = AGC_TMR_Value_10us;
	endcase
end

/*************************************************************************************/
//plause detect
/*************************************************************************************/

//plause_flag
wire  plause_flag;
assign plause_flag = (AFE_Gain_Active <= AGC_Plause_UThresh) && (AFE_Gain_Active >= AGC_Plause_LThresh);

//AGC_Plause_Error
assign AGC_Plause_Error = (~AGC_Dis) & AGC_Check_En & (~plause_flag);



/*************************************************************************************/
//analog signal synchronization
/*************************************************************************************/
wire SQUARE_WAVE_I_SYN;
sync2 #(.WIDE(1)) u_sync1(
	.clk			(clk			),
	.rst_n		    (rst_n		    ),
	.SCAN_MODE_IN	(SCAN_MODE_IN   ),
	.SCAN_CLK	    (SCAN_CLK	    ),
	.SCAN_RSTN	    (SCAN_RSTN	    ),
	.in				(SQUARE_WAVE_I	),
	.out			(SQUARE_WAVE_I_SYN)
);

wire SQUARE_WAVE_Q_SYN;
sync2 #(.WIDE(1)) u_sync2(
	.clk			(clk			),
	.rst_n		    (rst_n		    ),
	.SCAN_MODE_IN	(SCAN_MODE_IN   ),
	.SCAN_CLK	    (SCAN_CLK	    ),
	.SCAN_RSTN	    (SCAN_RSTN	    ),
	.in				(SQUARE_WAVE_Q	),
	.out			(SQUARE_WAVE_Q_SYN)
);

wire VCOMPOUT_OV_I_SYN;
sync2 #(.WIDE(1)) u_sync3(
	.clk			(clk			),
	.rst_n		    (rst_n		    ),
	.SCAN_MODE_IN	(SCAN_MODE_IN   ),
	.SCAN_CLK	    (SCAN_CLK	    ),
	.SCAN_RSTN	    (SCAN_RSTN	    ),
	.in				(VCOMPOUT_OV_I	),
	.out			(VCOMPOUT_OV_I_SYN)
);

wire VCOMPOUT_UV_I_SYN;
sync2 #(.WIDE(1)) u_sync4(
	.clk			(clk			),
	.rst_n		    (rst_n		    ),
	.SCAN_MODE_IN	(SCAN_MODE_IN   ),
	.SCAN_CLK	    (SCAN_CLK	    ),
	.SCAN_RSTN	    (SCAN_RSTN	    ),
	.in				(VCOMPOUT_UV_I	),
	.out			(VCOMPOUT_UV_I_SYN)
);

wire VCOMPOUT_OV_Q_SYN;
sync2 #(.WIDE(1)) u_sync5(
	.clk			(clk			),
	.rst_n		    (rst_n		    ),
	.SCAN_MODE_IN	(SCAN_MODE_IN   ),
	.SCAN_CLK	    (SCAN_CLK	    ),
	.SCAN_RSTN	    (SCAN_RSTN	    ),
	.in				(VCOMPOUT_OV_Q	),
	.out			(VCOMPOUT_OV_Q_SYN)
);

wire VCOMPOUT_UV_Q_SYN;
sync2 #(.WIDE(1)) u_sync6(
	.clk			(clk			),
	.rst_n		    (rst_n		    ),
	.SCAN_MODE_IN	(SCAN_MODE_IN   ),
	.SCAN_CLK	    (SCAN_CLK	    ),
	.SCAN_RSTN	    (SCAN_RSTN	    ),
	.in				(VCOMPOUT_UV_Q	),
	.out			(VCOMPOUT_UV_Q_SYN)
);

/*************************************************************************************/
//I & Q channel time windows generate
/*************************************************************************************/
wire WinPulseI;
reg WinPulseI_delay;
wire WinPulseI_edge_p;
wire WinPulseI_edge_n;

assign WinPulseI = SQUARE_WAVE_I_SYN & (~SQUARE_WAVE_Q_SYN);

assign WinPulseI_edge_p = WinPulseI & (~WinPulseI_delay);

assign WinPulseI_edge_n = (~WinPulseI) & WinPulseI_delay;

//for ScanChain
wire	clk_muxed = SCAN_MODE_IN ? SCAN_CLK : clk;
wire	rstn_muxed = SCAN_MODE_IN ? SCAN_RSTN : rst_n;	

always@(posedge clk_muxed or negedge rstn_muxed)begin
	if(~rstn_muxed)
		WinPulseI_delay <= 1'b0;
	else
		WinPulseI_delay <= WinPulseI;
end


wire WinPulseQ;
reg WinPulseQ_delay;
wire WinPulseQ_edge_p;
wire WinPulseQ_edge_n;

assign WinPulseQ = SQUARE_WAVE_I_SYN & SQUARE_WAVE_Q_SYN;

assign WinPulseQ_edge_p = WinPulseQ & (~WinPulseQ_delay);

assign WinPulseQ_edge_n = (~WinPulseQ) & WinPulseQ_delay;

always@(posedge clk_muxed or negedge rstn_muxed)begin
	if(~rstn_muxed)
		WinPulseQ_delay <= 1'b0;
	else
		WinPulseQ_delay <= WinPulseQ;
end
/*************************************************************************************/
//comparator output pattern decode
/*************************************************************************************/

wire norm_pattern_i;
wire norm_pattern_q;
wire under_pattern_i_q;
wire over_pattern_i;
wire over_pattern_q;
wire exception_pattern0;
wire exception_pattern1;
wire exception_pattern2;

//norm patterns
assign norm_pattern_i 	= (~VCOMPOUT_OV_I_SYN)&(VCOMPOUT_UV_I_SYN)&(~VCOMPOUT_OV_Q_SYN)&(~VCOMPOUT_UV_Q_SYN);//0100
assign norm_pattern_q 	= (~VCOMPOUT_OV_I_SYN)&(~VCOMPOUT_UV_I_SYN)&(~VCOMPOUT_OV_Q_SYN)&(VCOMPOUT_UV_Q_SYN);//0001
//under patterns
assign under_pattern_i_q = (~VCOMPOUT_OV_I_SYN)&(~VCOMPOUT_UV_I_SYN)&(~VCOMPOUT_OV_Q_SYN)&(~VCOMPOUT_UV_Q_SYN);//0000
//over patterns
assign over_pattern_i 	= ((VCOMPOUT_OV_I_SYN)&(VCOMPOUT_UV_I_SYN)&(~VCOMPOUT_OV_Q_SYN)&(~VCOMPOUT_UV_Q_SYN)) | ((VCOMPOUT_OV_I_SYN)&(VCOMPOUT_UV_I_SYN)&(VCOMPOUT_OV_Q_SYN)&(VCOMPOUT_UV_Q_SYN)) | ((VCOMPOUT_OV_I_SYN)&(VCOMPOUT_UV_I_SYN)&(~VCOMPOUT_OV_Q_SYN)&(VCOMPOUT_UV_Q_SYN));//1100 1111 1101
assign over_pattern_q 	= (~VCOMPOUT_OV_I_SYN)&(~VCOMPOUT_UV_I_SYN)&(VCOMPOUT_OV_Q_SYN)&(VCOMPOUT_UV_Q_SYN) | ((~VCOMPOUT_OV_I_SYN)&(VCOMPOUT_UV_I_SYN)&(VCOMPOUT_OV_Q_SYN)&(VCOMPOUT_UV_Q_SYN)) | ((~VCOMPOUT_OV_I_SYN)&(VCOMPOUT_UV_I_SYN)&(~VCOMPOUT_OV_Q_SYN)&(VCOMPOUT_UV_Q_SYN));//0011 0111 0101
//exception patterns
assign exception_pattern0 = ((VCOMPOUT_OV_I_SYN)&(~VCOMPOUT_UV_I_SYN)&(~VCOMPOUT_OV_Q_SYN)&(~VCOMPOUT_UV_Q_SYN)) | ((VCOMPOUT_OV_I_SYN)&(~VCOMPOUT_UV_I_SYN)&(~VCOMPOUT_OV_Q_SYN)&(VCOMPOUT_UV_Q_SYN)) | ((VCOMPOUT_OV_I_SYN)&(~VCOMPOUT_UV_I_SYN)&(VCOMPOUT_OV_Q_SYN)&(VCOMPOUT_UV_Q_SYN));//1000 1001 1011
assign exception_pattern1 = ((~VCOMPOUT_OV_I_SYN)&(~VCOMPOUT_UV_I_SYN)&(VCOMPOUT_OV_Q_SYN)&(~VCOMPOUT_UV_Q_SYN)) | ((~VCOMPOUT_OV_I_SYN)&(VCOMPOUT_UV_I_SYN)&(VCOMPOUT_OV_Q_SYN)&(~VCOMPOUT_UV_Q_SYN)) | ((VCOMPOUT_OV_I_SYN)&(VCOMPOUT_UV_I_SYN)&(VCOMPOUT_OV_Q_SYN)&(~VCOMPOUT_UV_Q_SYN));//0010 0110 1110
assign exception_pattern2 = (VCOMPOUT_OV_I_SYN)&(~VCOMPOUT_UV_I_SYN)&(VCOMPOUT_OV_Q_SYN)&(~VCOMPOUT_UV_Q_SYN);//1010

assign Comparator_Out_Exception = exception_pattern0 | exception_pattern1 | exception_pattern2;
/*************************************************************************************/
//I or Q channel amplitude state detecter1
/*************************************************************************************/
always@(posedge clk_muxed or negedge rstn_muxed) begin
	if(~rstn_muxed)
		amp_state <= AMP_UNKNOWN;
	else if(AFE_Gain_Config_Modify)
		amp_state <= AMP_UNKNOWN;
	else if ((!AGC_Dis) && (WinPulseI_edge_p || WinPulseQ_edge_p)) begin
		if(norm_pattern_i|norm_pattern_q)
			amp_state <= AMP_NORMAL;
		else if(under_pattern_i_q)
			amp_state <= AMP_UNDER;
		else if(over_pattern_i|over_pattern_q)
			amp_state <= AMP_OVER;
		else
			amp_state <= AMP_UNKNOWN;
	end else;
end


/*************************************************************************************/
//I or Q channel amplitude state detecter2
/*************************************************************************************/


reg [2:0]s_detecter2;
reg [TIMER_CNT_W-1:0]s_detecter2_timer;
reg flag_normal_pattern;
reg flag_under_pattern;
reg flag_over_pattern;

wire s_detecter2_S_OVER;
reg s_detecter2_S_OVER_delay;
wire s_detecter2_S_OVER_edge_p;

wire s_detecter2_S_UNDER;
reg s_detecter2_S_UNDER_delay;
wire s_detecter2_S_UNDER_edge_p;

always@(posedge clk_muxed or negedge rstn_muxed)begin
	if(~rstn_muxed)
		s_detecter2 <= S_UNKNOWN;
	else if(AFE_Gain_Config_Modify)
		s_detecter2 <= S_UNKNOWN;
	else begin
		case(s_detecter2)
	   		/*0*/S_UNKNOWN: begin
				s_detecter2 <= S_PRE_DETECT;
			end
			/*1*/S_PRE_DETECT: begin
				s_detecter2 <= S_DETECT;
			end
            /*2*/S_DETECT: begin
				if(over_pattern_i|over_pattern_q)
					s_detecter2 <= S_OVER;
				else if(s_detecter2_timer == AGC_TMR_Value)
					if(flag_normal_pattern)
						s_detecter2 <= S_NORMAL;
					else if(~flag_normal_pattern & flag_under_pattern)
						s_detecter2 <= S_UNDER;
					else;
				else
					s_detecter2 <= S_DETECT;
			end
            /*3*/S_OVER: begin
				if(s_attacker == ATTACK_END_PRE)
					s_detecter2 <= S_UNKNOWN;
				else
					s_detecter2 <= S_OVER;
			end
            /*4*/S_UNDER: begin
				if(s_attacker == ATTACK_END_PRE)
					s_detecter2 <= S_UNKNOWN;
				else
					s_detecter2 <= S_UNDER;
			end
            /*5*/S_NORMAL: begin
				s_detecter2 <= S_UNKNOWN;
			end
			default:s_detecter2 <= S_UNKNOWN;
		endcase
	end
end

always@(posedge clk_muxed or negedge rstn_muxed)begin
	if(~rstn_muxed)
		s_detecter2_timer <= 'h0;
	else if(AFE_Gain_Config_Modify)
		s_detecter2_timer <= 'h0;
	else if(s_detecter2 == S_PRE_DETECT)
		s_detecter2_timer <= 'h0;
	else
		s_detecter2_timer <= s_detecter2_timer + 1;
end


always@(posedge clk_muxed or negedge rstn_muxed)begin
	if(~rstn_muxed)
		flag_normal_pattern <= 1'b0;
	else if(AFE_Gain_Config_Modify)
		flag_normal_pattern <= 1'b0;
	else if(s_detecter2 == S_PRE_DETECT)
		flag_normal_pattern <= 1'b0;
	else if((s_detecter2 == S_DETECT) && (norm_pattern_i|norm_pattern_q == 1'b1))
		flag_normal_pattern <= 1'b1;
	else;
end


always@(posedge clk_muxed or negedge rstn_muxed)begin
	if(~rstn_muxed)
		flag_under_pattern <= 1'b0;
	else if(AFE_Gain_Config_Modify)
		flag_under_pattern <= 1'b0;
	else if(s_detecter2 == S_PRE_DETECT)
		flag_under_pattern <= 1'b0;
	else if((s_detecter2 == S_DETECT) && (under_pattern_i_q == 1'b1))
		flag_under_pattern <= 1'b1;
	else;
end


always@(posedge clk_muxed or negedge rstn_muxed)begin
	if(~rstn_muxed)
		flag_over_pattern <= 1'b0;
	else if(AFE_Gain_Config_Modify)
		flag_over_pattern <= 1'b0;
	else if(s_detecter2 == S_PRE_DETECT)
		flag_over_pattern <= 1'b0;
	else if((s_detecter2 == S_DETECT) && (over_pattern_i|over_pattern_q == 1'b1))
		flag_over_pattern <= 1'b1;
	else;
end



assign s_detecter2_S_OVER = (s_detecter2 == S_OVER);

always@(posedge clk_muxed or negedge rstn_muxed)begin
	if(~rstn_muxed)
		s_detecter2_S_OVER_delay <= 1'b0;
	else
		s_detecter2_S_OVER_delay <= s_detecter2_S_OVER;
end

assign s_detecter2_S_OVER_edge_p = s_detecter2_S_OVER & (~s_detecter2_S_OVER_delay);

assign s_detecter2_S_UNDER = (s_detecter2 == S_UNDER);

always@(posedge clk_muxed or negedge rstn_muxed)begin
	if(~rstn_muxed)
		s_detecter2_S_UNDER_delay <= 1'b0;
	else
		s_detecter2_S_UNDER_delay <= s_detecter2_S_UNDER;
end

assign s_detecter2_S_UNDER_edge_p = s_detecter2_S_UNDER & (~s_detecter2_S_UNDER_delay);

/*************************************************************************************/
//Strategy Switch
/*************************************************************************************/
wire s_attacker_reset_condition = AGC_Strategy_Sel? (s_detecter2_S_OVER_edge_p | s_detecter2_S_UNDER_edge_p) : (WinPulseI_edge_p | WinPulseQ_edge_p) ;
wire s_attacker_force_end = AGC_Strategy_Sel? 1'b0 : WinPulseI_edge_n;
wire s_attacker_detect_under = AGC_Strategy_Sel? (s_detecter2 == S_UNDER) : (amp_state == AMP_UNDER);
wire s_attacker_detect_over = AGC_Strategy_Sel? (s_detecter2 == S_OVER) : (amp_state == AMP_OVER);
wire s_attacker_detect_normal = AGC_Strategy_Sel? (s_detecter2 == S_NORMAL) : (amp_state == AMP_NORMAL);


/*************************************************************************************/
//I or Q channel amplitude state attacker
/*************************************************************************************/
always@(posedge clk_muxed or negedge rstn_muxed)begin
	if(~rstn_muxed)
		s_attacker <= IDLE;
	else if(AFE_Gain_Config_Modify)
		s_attacker <= IDLE;
	else if(s_attacker_reset_condition)
		s_attacker <= IDLE;
	else if(s_attacker_force_end)
		s_attacker <= ATTACK_END_PRE;
	else if(!AGC_Dis)
		case(s_attacker)
			/*0*/IDLE:begin
				if(s_attacker_detect_under)
					s_attacker <= ATTACK_UNDER;
				else if(s_attacker_detect_over)
					s_attacker <= ATTACK_OVER;
				else if(s_attacker_detect_normal)
					s_attacker <= IDLE;
				else;
			end
			/*1*/ATTACK_UNDER:s_attacker <= DELAY_PRE;
			/*2*/ATTACK_OVER:s_attacker <= DELAY_PRE;
			/*3*/DELAY_PRE:s_attacker <= DELAY;
			/*4*/DELAY:begin
				if(attacker_timer == AGC_TMR_Value_10us)
					if(s_attacker_detect_under)
						s_attacker <= DETECT_UNDER;
					else if(s_attacker_detect_over)
						s_attacker <= DETECT_OVER;
					else;
				else
					s_attacker <= DELAY;
			end
			/*5*/DETECT_UNDER:begin
				if(under_attack_counter>=UNDER_ATTACK_NUM)
					s_attacker <= ATTACK_END_PRE;
				else if(norm_pattern_i|norm_pattern_q)
					s_attacker <= ATTACK_END_PRE;
				else if(under_pattern_i_q)
					s_attacker <= ATTACK_UNDER;
				else if(over_pattern_i|over_pattern_q)
					s_attacker <= ATTACK_END_PRE;
				else;
			end
			/*6*/DETECT_OVER:begin
				if(norm_pattern_i|norm_pattern_q)
					s_attacker <= ATTACK_END_PRE;
				else if(under_pattern_i_q)
					s_attacker <= ATTACK_END_PRE;
				else if(over_pattern_i|over_pattern_q)
					s_attacker <= ATTACK_OVER;
				else;
			end
			/*7*/ATTACK_END_PRE:begin
				s_attacker <= ATTACK_END;
			end
			/*8*/ATTACK_END: s_attacker <= ATTACK_END;
			default:s_attacker <= IDLE;
		endcase
	else;
end

//attacker for under state counter
always@(posedge clk_muxed or negedge rstn_muxed)begin
	if(~rstn_muxed)
		under_attack_counter <= 'h0;
	else if(AFE_Gain_Config_Modify)
		under_attack_counter <= 'h0;
	else if(s_attacker == IDLE)
		under_attack_counter <= 'h0;
	else if(s_attacker == ATTACK_UNDER)
		under_attack_counter <= under_attack_counter + 1;
	else;
end


//attacker delay timer
always@(posedge clk_muxed or negedge rstn_muxed)begin
	if(~rstn_muxed)
		attacker_timer <= 'h0;
	else if(AFE_Gain_Config_Modify)
		attacker_timer <= 'h0;
	else if(s_attacker == DELAY_PRE)
		attacker_timer <= 'h0;
	else if(s_attacker == DELAY)
		attacker_timer <= attacker_timer + 1;
	else;
end

/*************************************************************************************/
//Gain configure word output
/*************************************************************************************/

//AFE_Gain_Active/AFE_Gain_Active_Modify
always@(posedge clk_muxed or negedge rstn_muxed) begin
	if(~rstn_muxed) begin
		AFE_Gain_Active <= 'h0;
		AFE_Gain_Active_Modify <= 1'b0;
	end
	else if(AFE_Gain_Config_Modify) begin
		AFE_Gain_Active <= AFE_Gain_Config;
		AFE_Gain_Active_Modify <= 1'b1;
	end
	else if( !AGC_Dis && (s_attacker == ATTACK_UNDER) && plause_flag)
		if(AFE_Gain_Active == CONFIG_WORD_UPBOUND)begin
			AFE_Gain_Active <= CONFIG_WORD_UPBOUND;
			AFE_Gain_Active_Modify <= 1'b0;
		end
		else begin
			AFE_Gain_Active <= AFE_Gain_Active + 1;
			AFE_Gain_Active_Modify <= 1'b1;
		end
	else if( !AGC_Dis && (s_attacker == ATTACK_OVER) && plause_flag)
		if(AFE_Gain_Active == CONFIG_WORD_LOWBOUND)begin
			AFE_Gain_Active <= CONFIG_WORD_LOWBOUND;
			AFE_Gain_Active_Modify <= 1'b0;
		end
		else begin
			AFE_Gain_Active <= AFE_Gain_Active - 1;
			AFE_Gain_Active_Modify <= 1'b1;
		end
	else begin
		AFE_Gain_Active <= AFE_Gain_Active;
		AFE_Gain_Active_Modify <= 1'b0;
	end
end

endmodule
