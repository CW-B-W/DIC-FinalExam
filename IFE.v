
`timescale 1ns/10ps

module IFE(clk,reset,busy,ready,iaddr,idata,data_rd,data_wr,addr,wen,sel);

input					clk;
input					reset;
output	reg				busy;	
input					ready;	
output	reg [13:0]		iaddr;
input	[7:0]			idata;	
input	[7:0]			data_rd;
output	reg [7:0]		data_wr;
output	reg [13:0]		addr;
output	reg 			wen;
input 	[1:0]			sel;

reg [7:0] mat[8:0];
reg [6:0] mat_rd_idx;

reg  signed [7:0] x_center;
reg  signed [7:0] y_center;
// wire signed [2:0] dx = mat_rd_idx / 4'd3 - 4'd1;
// wire signed [2:0] dy = mat_rd_idx % 4'd3 - 4'd1;
reg  signed [2:0] dx;
reg  signed [2:0] dy;
wire signed [7:0] x = x_center + dx;
wire signed [7:0] y = y_center + dy;

reg [20:0] sum;
reg [20:0] mean;
reg [7:0]  maxv;

parameter S_IDLE   = 0;
parameter S_RD_REQ = 1;
parameter S_RD_RES = 2;
parameter S_WR     = 3;
reg [7:0] state;
reg [7:0] n_state;

always@(*) begin
	case (sel)
		2'd0: begin
			dx = mat_rd_idx / 4'd3 - 4'd1;
		end

		2'd1: begin
			dx = mat_rd_idx / 4'd5 - 4'd2;
		end

		2'd2: begin
			dx = mat_rd_idx / 4'd3 - 4'd1;
		end

		2'd3: begin
			dx = 0;
		end

		default: begin
			$stop;
		end
	endcase
end

always@(*) begin
	case (sel)
		2'd0: begin
			dy = mat_rd_idx % 4'd3 - 4'd1;
		end

		2'd1: begin
			dy = mat_rd_idx % 4'd5 - 4'd2;
		end

		2'd2: begin
			dy = mat_rd_idx % 4'd3 - 4'd1;
		end

		2'd3: begin
			dy = 0;
		end

		default: begin
			$stop;
		end

	endcase
end

always@(posedge clk, posedge reset) begin
    if (reset)
        state <= S_IDLE;
    else
        state <= n_state;
end

always@(posedge clk) begin
    case (sel)
    	// 3x3 mean filter
    	2'd0: begin
    		case (state)
    			S_IDLE: begin
    				busy       <= 0;
    				x_center   <= 0;
    				y_center   <= 0;
    				mat_rd_idx <= 0;
    				sum        <= 0;
    			end

    			S_RD_REQ: begin
    				wen  <= 0;
		    		busy <= 1;
		    		if (!(x < 0 || x >= 128 || y < 0 || y >= 128)) begin
		    			iaddr <= (y << 7) | x;
	    			end
    			end

    			S_RD_RES: begin
    				if (!(x < 0 || x >= 128 || y < 0 || y >= 128))
    					sum <= sum + idata;
    				else
    					sum <= sum;

    				if (mat_rd_idx == 8) begin
    					mat_rd_idx <= 0;
    				end
    				else begin
    					mat_rd_idx <= mat_rd_idx + 1;
    				end
    			end

    			S_WR: begin
    				addr    <= (y_center << 7) | x_center;
	    			wen     <= 1;
	    			data_wr <= sum / 9;

	    			if (x_center == 127) begin
	    				x_center <= 0;
	    				y_center <= y_center + 1;
	    			end
	    			else begin
	    				x_center <= x_center + 1;
	    			end
	    			sum <= 0;
    			end
    		endcase
    	end

    	// 5x5 mean filter
    	2'd1: begin
    		case (state)
    			S_IDLE: begin
    				busy       <= 0;
    				x_center   <= 0;
    				y_center   <= 0;
    				mat_rd_idx <= 0;
    				sum        <= 0;
    			end

    			S_RD_REQ: begin
    				wen  <= 0;
		    		busy <= 1;
		    		if (!(x < 0 || x >= 128 || y < 0 || y >= 128)) begin
		    			iaddr <= (y << 7) | x;
	    			end
    			end

    			S_RD_RES: begin
    				if (!(x < 0 || x >= 128 || y < 0 || y >= 128))
    					sum <= sum + idata;
    				else
    					sum <= sum;

    				if (mat_rd_idx == 24) begin
    					mat_rd_idx <= 0;
    				end
    				else begin
    					mat_rd_idx <= mat_rd_idx + 1;
    				end
    			end

    			S_WR: begin
    				addr    <= (y_center << 7) | x_center;
	    			wen     <= 1;
	    			data_wr <= sum / 25;

	    			if (x_center == 127) begin
	    				x_center <= 0;
	    				y_center <= y_center + 1;
	    			end
	    			else begin
	    				x_center <= x_center + 1;
	    			end
	    			sum <= 0;
    			end
    		endcase
    	end

    	// 3x3 max filter
    	2'd2: begin
    		case (state)
    			S_IDLE: begin
    				busy       <= 0;
    				x_center   <= 0;
    				y_center   <= 0;
    				mat_rd_idx <= 0;
    				maxv       <= 0;
    			end

    			S_RD_REQ: begin
    				wen  <= 0;
		    		busy <= 1;
		    		if (!(x < 0 || x >= 128 || y < 0 || y >= 128)) begin
		    			iaddr <= (y << 7) | x;
	    			end
    			end

    			S_RD_RES: begin
    				if (!(x < 0 || x >= 128 || y < 0 || y >= 128))
    					maxv <= idata > maxv ? idata : maxv;
    				else
    					maxv <= 0 > maxv ? 0 : maxv;

    				if (mat_rd_idx == 8) begin
    					mat_rd_idx <= 0;
    				end
    				else begin
    					mat_rd_idx <= mat_rd_idx + 1;
    				end
    			end

    			S_WR: begin
    				addr    <= (y_center << 7) | x_center;
	    			wen     <= 1;
	    			data_wr <= maxv;

	    			if (x_center == 127) begin
	    				x_center <= 0;
	    				y_center <= y_center + 1;
	    			end
	    			else begin
	    				x_center <= x_center + 1;
	    			end
	    			maxv <= 0;
    			end
    		endcase
    	end

    	// threshold
    	2'd3: begin
    		case (state)
    			S_IDLE: begin
    				busy       <= 0;
    				x_center   <= 0;
    				y_center   <= 0;
    				mat_rd_idx <= 0;
    			end

    			S_RD_REQ: begin
		    		wen  <= 0;
		    		busy <= 1;
		    		if (!(x < 0 || x >= 128 || y < 0 || y >= 128)) begin
		    			iaddr <= (y << 7) | x;
	    			end
	    		end

	    		S_RD_RES: begin
	    			mat[0] <= idata;
	    		end

	    		S_WR: begin
	    			addr <= (y_center << 7) | x_center;
	    			wen  <= 1;
	    			if (mat[0] >= 127) begin
	    				data_wr <= mat[0];
	    			end
	    			else begin
	    				data_wr <= 0;
	    			end

	    			if (x_center == 127) begin
	    				x_center <= 0;
	    				y_center <= y_center + 1;
	    			end
	    			else begin
	    				x_center <= x_center + 1;
	    			end
	    		end
	    	endcase
    	end
    endcase
end


// next state logic
always@(*) begin
    n_state = S_IDLE;

    case (sel)
    	2'd0: begin
    		case (state)
    			S_IDLE: begin
    				if (reset) begin
    					n_state = S_IDLE;
    				end
    				else begin
    					n_state = S_RD_REQ;
    				end
    			end

    			S_RD_REQ: begin
    				n_state = S_RD_RES;
    			end

    			S_RD_RES: begin
    				if (mat_rd_idx == 8)
    					n_state = S_WR;
    				else
    					n_state = S_RD_REQ;
    			end

    			S_WR: begin
    				if (x_center == 127 && y_center == 127)
    					n_state = S_IDLE;
    				else
    					n_state = S_RD_REQ;
    			end

    			default: begin
    			end
    		endcase
    	end

    	2'd1: begin
    		case (state)
    			S_IDLE: begin
    				if (reset) begin
    					n_state = S_IDLE;
    				end
    				else begin
    					n_state = S_RD_REQ;
    				end
    			end

    			S_RD_REQ: begin
    				n_state = S_RD_RES;
    			end

    			S_RD_RES: begin
    				if (mat_rd_idx == 24)
    					n_state = S_WR;
    				else
    					n_state = S_RD_REQ;
    			end

    			S_WR: begin
    				if (x_center == 127 && y_center == 127)
    					n_state = S_IDLE;
    				else
    					n_state = S_RD_REQ;
    			end

    			default: begin
    			end
    		endcase
    	end

    	2'd2: begin
    		case (state)
    			S_IDLE: begin
    				if (reset) begin
    					n_state = S_IDLE;
    				end
    				else begin
    					n_state = S_RD_REQ;
    				end
    			end

    			S_RD_REQ: begin
    				n_state = S_RD_RES;
    			end

    			S_RD_RES: begin
    				if (mat_rd_idx == 8)
    					n_state = S_WR;
    				else
    					n_state = S_RD_REQ;
    			end

    			S_WR: begin
    				if (x_center == 127 && y_center == 127)
    					n_state = S_IDLE;
    				else
    					n_state = S_RD_REQ;
    			end

    			default: begin
    			end
    		endcase
    	end

    	2'd3: begin
    		case (state)
    			S_IDLE: begin
    				if (reset) begin
    					n_state = S_IDLE;
    				end
    				else begin
    					n_state = S_RD_REQ;
    				end
    			end

    			S_RD_REQ: begin
    				n_state = S_RD_RES;
    			end

    			S_RD_RES: begin
    				n_state = S_WR;
    			end

    			S_WR: begin
    				if (x_center == 127 && y_center == 127)
    					n_state = S_IDLE;
    				else
    					n_state = S_RD_REQ;
    			end

    			default: begin
    			end
    		endcase
    	end
    endcase
end

endmodule

