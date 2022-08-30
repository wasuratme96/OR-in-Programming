var 0..10: x;
var int : obj = (x-7)*(x-7);

constraint x mod 4 = 0;

solve minimize obj;
output ["x = \(x);\nobj = \(obj);\n"];