% Use this editor as a MiniZinc scratch book
solve maximize 6*F + 10*L + 8*Z + 40*J;  

int: budget = 10000;
constraint 13*F + 21*L + 17*Z + 100*J <= budget;

var 0..1000: F; % F = [0, 1000]
var 0..400: L; % L = [0, 400]
var 0..500: Z; % Z = [0, 500]
var 0..150: J;

output ["F = \(F), L = \(L), Z = \(Z), J = \(J)\n"];