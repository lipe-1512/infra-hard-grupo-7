module mux_equal (
    input in1, sel,
    output out
);
    wire in2;
    assign in2 = ~in1; 
    assign out = (sel == 0) ? in1 : in2;

endmodule