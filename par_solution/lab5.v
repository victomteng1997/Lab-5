`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    22:51:37 04/11/2013 C
// Design Name: 
// Module Name:    labkit 
// Project Name: 
// Target Devices: 
// Tool versions: 
// Description: 
//
// Dependencies: 
//
// Revision: 
// Revision 0.01 - File Created
// Additional Comments: 
//
//////////////////////////////////////////////////////////////////////////////////
module labkit(
    input clk_100mhz,
    input [7:0] switch,
	 
    input btn_up,       // buttons, depress = high
    input btn_enter,
    input btn_left,
    input btn_down,
    input btn_right,
	 
    output [7:0] seg,   //output 0->6 = seg A->G ACTIVE LOW, 
                        //output 7 = decimal point, all active low
								
    output [3:0] dig,   //selects digits 0-3, ACTIVE LOW
    output [7:0] led,   // 1 turns on leds
	 
    output [2:0] vgared,
    output [2:0] vgagreen,
    output [2:1] vgablue,
    output hsync,
    output vsync,
	 
    inout [7:0] ja,
    inout [7:0] jb,
    inout [7:0] jc,
    input [7:0] jd,
    inout [19:0] exp_io_n,
    inout [19:0] exp_io_p
    );

    // all unused outputs must be assigned
//    assign vgared = 3'b111;
//    assign vgagreen = 3'b111;
//    assign vgablue = 2'b11;
//    assign hsync = 1'b1;
//    assign vsync = 1'b1;
	 
// next three lines turns the 7 seg display completely off
    assign seg = 7'b111_1111; 	//output 0->6 = seg A->G ACTIVE LOW
    assign dp = 1'b1; 				//decimal point ACTIVE LOW
    assign dig = 4'hF;  			//selectives digits 0-3, ACTIVE LOW
 
 
  
//////////////////////////////////////////////////////////////////
// dcm_all is a general purpose digital clock manager. It is used
// to create clocks at desired frequncies and phases.
//
   wire clk_65mhz;
	wire clk_100mhz_buf;  // 100mhz buffered clock, not used
		
   dcm_all_v2 #(.DCM_DIVIDE(20), .DCM_MULTIPLY(13))
     	my_clocks(
				.CLK(clk_100mhz),
				.CLKSYS(clk_100mhz_buf),
//				.CLK25(CLK25),
				.CLK_out(clk_65mhz)
	);
//
//////////////////////////////////////////////////////////////////

   wire pixel_clk = clk_65mhz;  // clock for 1024 x 768 60hz resolution  

   assign led = switch;  // provide feedback

   wire [10:0] hcount;
   wire [9:0]  vcount;   
	wire blank, hblank;
		
	wire [7:0] pixel; //,paddle_pix,ball;
	
	//assign pixel = paddle_pix | ball;
	
	assign vgared = blank ? pixel[7:5] : 3'b0;
	assign vgagreen = blank ? pixel[4:2] : 3'b0;
	assign vgablue = blank ? pixel[1:0] : 2'b0;
	
	debounce  db_up(.reset(reset), .clock(pixel_clk), .noisy(btn_up), .clean(up));			 
	debounce  db_down(.reset(reset), .clock(pixel_clk), .noisy(btn_down), .clean(down));  
	debounce  db_enter(.reset(1'b0), .clock(pixel_clk), .noisy(btn_enter), .clean(enter)); 
	
   assign reset = enter;
	
   vga_general video_driver(.pixel_clk(pixel_clk),.hcount(hcount), .vcount(vcount),
		.hsync(hsync1),.vsync(vsync1), .blank(blank1), .hblank(hblank1));

   wire [7:0] pong_pixel;
	
	reg [7:0] rgb;
	reg hs, vs, b;
   
	wire border = (hcount==0 | hcount==1023 | vcount==0 | vcount==767);
	
   always @(posedge pixel_clk) begin
//     case (switch[7:6])
     case (switch[1:0])
		2'b00: begin   // send the pong pixels through
		         hs <= phsync;
					vs <= pvsync;
					b <= pblank;
					rgb <= pong_pixel;
				 end
		2'b01:	 begin  // 1 pixel outline of visible area (white)
		         hs <= hsync1;
					vs <= vsync1;
					b <= blank1;
					rgb <= {{8{border}}};
				 end
		2'b10:  begin  // color bars
		         hs <= hsync1;
					vs <= vsync1;
					b <= blank1;
					rgb <= {{3{hcount[8]}}, {3{hcount[7]}}, {2{hcount[6]}}};				
             end
		2'b11: begin   // send the pong pixels through
		         hs <= phsync;
					vs <= pvsync;
					b <= pblank;
					rgb <= pong_pixel;
				 end
		 endcase
	  end
				 
               
		
   assign pixel = rgb; //{{3{hcount[8]}}, {3{hcount[7]}}, {2{hcount[6]}}};	//rgb;
	assign blank = b;
	assign vsync = vs;
	assign hsync = hs;	
	
   pong_game psolution(.pixel_clk(pixel_clk), .reset(reset), .up(up), .down(down), .pspeed(switch[7:4]),
	    .hcount(hcount), .vcount(vcount), .hsync(hsync1), .vsync(vsync1), .blank(blank1),
		 .phsync(phsync), .pvsync(pvsync), .pblank(pblank), .pixel(pong_pixel));	
	
endmodule

////////////////////////////////////////////////////////////////////////////////
//
// pong_game: the game itself!
//
////////////////////////////////////////////////////////////////////////////////

module pong_game (
   input pixel_clk,	// 65MHz clock
   input reset,		// 1 to initialize module
   input up,		// 1 when paddle should move up
   input down,  	// 1 when paddle should move down
   input [3:0] pspeed,  // puck speed in pixels/tick 
   input [10:0] hcount,	// horizontal index of current pixel (0..1023)
   input [9:0]  vcount, // vertical index of current pixel (0..767)
   input hsync,		// XVGA horizontal sync signal (active low)
   input vsync,		// XVGA vertical sync signal (active low)
   input blank,		// XVGA blanking (1 means output black pixel)
 	
   output phsync,	// pong game's horizontal sync
   output pvsync,	// pong game's vertical sync
   output pblank,	// pong game's blanking
   output [7:0] pixel	// pong game's pixel  // r=7:5, g=4:2, b=1:0 
   );

   wire [2:0] checkerboard;
	
 ////////////////////////////////////////////////////////////////
 // REPLACE ME! The code below just generates a color checkerboard
 // using 64 pixel by 64 pixel squares.
   
   assign phsync = hsync;
   assign pvsync = vsync;
   assign pblank = blank;
   assign checkerboard = hcount[8:6] + vcount[8:6];

   // here we use three bits from hcount and vcount to generate the \
   // checkerboard

   assign pixel = {{8{checkerboard[2]}}, {8{checkerboard[1]}}, {8{checkerboard[0]}}} ;
//
////////////////////////////////////////////////////////////////

	
endmodule

// Switch Debounce Module
// use your system clock for the clock input
// to produce a synchronous, debounced output
module debounce #(parameter DELAY=400000)   // .01 sec with a 49Mhz clock
	        (input reset, clock, noisy,
	         output reg clean);

   reg [19:0] count;
   reg new;

   always @(posedge clock)
     if (reset)
       begin
	  count <= 0;
	  new <= noisy;
	  clean <= noisy;
       end
     else if (noisy != new)
       begin
	  new <= noisy;
	  count <= 0;
       end
     else if (count == DELAY)
       clean <= new;
     else
       count <= count+1;
      
endmodule

//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: Gim P. Hom 3/22/2007
// 
// Create Date:    17:51:37 03/11/2007 
// Design Name: 
// Module Name:    vga_general 
// Project Name: 
// Revision 0.01 - File Created
// Additional Comments: 
//
//////////////////////////////////////////////////////////////////////////////////
module vga_general(
    input pixel_clk, 
    output reg [10:0] hcount,
    output reg [9:0] vcount,
    output blank,
    output hblank,	 
    output  hsync, 
    output  vsync
);	 
/*   //reg  hblank=1;
	assign vclock = pixel_clk;
	
   // horizontal: 1344 pixels total
   // display 1024 pixels per line
   reg vblank;
   wire hsyncon,hsyncoff,hreset,hblankon;
   assign hblankon = (hcount == 1023);    
   assign hsyncon = (hcount == 1047);
   assign hsyncoff = (hcount == 1183);
   assign hreset = (hcount == 1343);

   // vertical: 806 lines total
   // display 768 lines
   wire vsyncon,vsyncoff,vreset,vblankon;
   assign vblankon = hreset & (vcount == 767);    
   assign vsyncon = hreset & (vcount == 776);
   assign vsyncoff = hreset & (vcount == 782);
   assign vreset = hreset & (vcount == 805);

   // sync and blanking
   wire next_hblank,next_vblank;
   assign next_hblank = hreset ? 0 : hblankon ? 1 : hblank;
   assign next_vblank = vreset ? 0 : vblankon ? 1 : vblank;
   always @(posedge vclock) begin
      hcount <= hreset ? 0 : hcount + 1;
      hblank <= next_hblank;
      hsync <= hsyncon ? 0 : hsyncoff ? 1 : hsync;  // active low

      vcount <= hreset ? (vreset ? 0 : vcount + 1) : vcount;
      vblank <= next_vblank;
      vsync <= vsyncon ? 0 : vsyncoff ? 1 : vsync;  // active low

      blank <= next_vblank | (next_hblank & ~hreset);
   end*/

	 wire vblank;
	 
 
	 // 1024 x 768 @ 60hz;  alternate resolution = 640x480 75hz
	 parameter hfp = 24;  //24;  // 16;  
	 parameter hsy = 136;  //136; // 96;
	 parameter hbp = 160;  //160; // 48;
	 
	 parameter vfp = 3;  //3;  //  11;
	 parameter vsy = 6;   //6;  //  2;
	 parameter vbp = 29;  //29; //  32;
	 
	 parameter hsize = 1023; //1023; //639;  // there are 640 pixels counting 0
	 parameter vsize = 767; //767; //479;  // similarly there are 480 lines counting line 0
	 
	 wire  h_end = (hcount == (hsize + hfp + hsy + hbp));
	 wire  v_end = (vcount == (vsize + vfp + vsy + vbp));
	 
	 assign hsync = ((hcount < hsize + hfp) || (hcount > hsize + hfp + hsy));
	 assign vsync = ((vcount < vsize + vfp) || (vcount > vsize + vfp + vsy));
	 
	 assign hblank = (hcount <= hsize);
	 assign vblank = (vcount <= vsize);
	 
	 assign blank = hblank && vblank;
	 
	 always @(posedge pixel_clk)
	    begin
		 hcount <= h_end ? 0 : hcount + 1;
		 vcount <= h_end ? (v_end ? 0 : vcount + 1) : vcount;  
		 end
		 
endmodule

//////////////////////////////////////////////////////////////////////////////////////////
// Company: Digilent Inc 2011
// Engineer: Michelle Yu  
// Create Date:    08/26/2011
// Module Name:    dcm_all
// Project Name:     PmodPS2_Demo
// Target Devices: Nexys3 
// Tool version:     ISE 14.2
// Description: This file contains the design for a dcm that generates a 25MHz and a 
//                40MHz clock from a 100MHz clock.
//
// Revision: 
// Revision 0.01 - File Created
// Revision 1.00 - Converted from VHDL to Verilog (Josh Sackos)
// Revision 2.00 - removed CLK25, add parameters for divide/multiply
//////////////////////////////////////////////////////////////////////////////////////////

// =======================================================================================
//                                 Define Module
// =======================================================================================
module dcm_all_v2  #(parameter DCM_DIVIDE = 4,
                               DCM_MULTIPLY = 2)
     (
      CLK,
//    RST,
      CLKSYS,
//      CLK25,
      CLK_out
);

// =======================================================================================
//                               Port Declarations
// =======================================================================================

         input   CLK;
//         input   RST;
         output  CLKSYS;
//         output  CLK25;
         output  CLK_out;

// =======================================================================================
//                        Parameters, Registers, and Wires
// =======================================================================================   

         // Output registers
         wire CLKSYS;
         wire CLK25;
         wire CLK_out;

         // architecture of dcm_all entity
         wire    GND = 1'b0;
         wire    CLKSYSint;
         wire    CLKSYSbuf;
         
         assign CLKSYS = CLKSYSbuf;

// =======================================================================================
//                                 Implementation
// =======================================================================================   

         // buffer system clock and wire to dcm feedback
         BUFG BUFG_clksys(
                  .O(CLKSYSbuf),
                  .I(CLKSYSint)
         );

         // Instantiation of the DCM device primitive.
         // Feedback is not used.
         // Clock multiplier is 2
         // Clock divider is 5
         // 100MHz * 2/5 = 40MHz   
         // The following generics are only necessary if you wish to change the default behavior.
         DCM #(
                  .CLK_FEEDBACK("1X"),
                  .CLKDV_DIVIDE(4.0),                   //  Divide by: 1.5,2.0,2.5,3.0,3.5,4.0,4.5,5.0,5.5,6.0,6.5
                                                        //             7.0,7.5,8.0,9.0,10.0,11.0,12.0,13.0,14.0,15.0 or 16.0
                  .CLKFX_DIVIDE(DCM_DIVIDE),            //  Can be any interger from 2 to 32
                  .CLKFX_MULTIPLY(DCM_MULTIPLY),        //  Can be any integer from 2 to 32
                  .CLKIN_DIVIDE_BY_2("FALSE"),          //  TRUE/FALSE to enable CLKIN divide by two feature
                  .CLKIN_PERIOD(10000.0),               //  Specify period of input clock (ps)
                  .CLKOUT_PHASE_SHIFT("NONE"),          //  Specify phase shift of NONE, FIXED or VARIABLE
                  .DESKEW_ADJUST("SYSTEM_SYNCHRONOUS"), //  SOURCE_SYNCHRONOUS, SYSTEM_SYNCHRONOUS or
                                                        //        an integer from 0 to 15
                  .DFS_FREQUENCY_MODE("LOW"),           //  HIGH or LOW frequency mode for frequency synthesis
                  .DLL_FREQUENCY_MODE("LOW"),           //  HIGH or LOW frequency mode for DLL
                  .DUTY_CYCLE_CORRECTION("TRUE"),       //  Duty cycle correction, TRUE or FALSE
                  .FACTORY_JF(16'hC080),                //  FACTORY JF Values
                  .PHASE_SHIFT(0),                      //  Amount of fixed phase shift from -255 to 255
                  .STARTUP_WAIT("FALSE")                //  Delay configuration DONE until DCM LOCK, TRUE/FALSE
         )
         DCM_inst(
                  .CLK0(CLKSYSint),                     // 0 degree DCM CLK ouptput
                  .CLK180(),                            // 180 degree DCM CLK output
                  .CLK270(),                            // 270 degree DCM CLK output
                  .CLK2X(),                             // 2X DCM CLK output
                  .CLK2X180(),                          // 2X, 180 degree DCM CLK out
                  .CLK90(),                             // 90 degree DCM CLK output
                  .CLKDV(), //(CLK25),                  // Divided DCM CLK out (CLKDV_DIVIDE)
                  .CLKFX(CLK_out),                      // DCM CLK synthesis out (M/D)
                  .CLKFX180(),                          // 180 degree CLK synthesis out
                  .LOCKED(),                            // DCM LOCK status output
                  .PSDONE(),                            // Dynamic phase adjust done output
                  .STATUS(),                            // 8-bit DCM status bits output
                  .CLKFB(CLKSYSbuf),                    // DCM clock feedback
                  .CLKIN(CLK),                          // Clock input (from IBUFG, BUFG or DCM)
                  .PSCLK(GND),                          // Dynamic phase adjust clock input
                  .PSEN(GND),                           // Dynamic phase adjust enable input
                  .PSINCDEC(GND),                       // Dynamic phase adjust increment/decrement
                  .DSSEN(1'b0),
                  .RST (1'b0)  //(RST)                  // DCM asynchronous reset input
         );
   
endmodule
