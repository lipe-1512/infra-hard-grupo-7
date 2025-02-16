module mux_optype (
    input [31:0] in1, in2, in3, in4, in5,
    input [2:0] sel,
    output [31:0] out
);

    assign out = (sel == 000) ? in1:
                 (sel == 001) ? in2:
                 (sel == 010) ? in3:
                 (sel == 011) ? in4:
                 (sel == 100) ? in1:
                 32'bx;
    
endmodule