array[int] of int: readings;
int: start;
int: l = length(readings);
set of int: TIME0 = 0..l;
set of int: TIME = 0..l;

array[TIME0] of var 25..30: temp;
array[TIME] of var 25..30: tp;
array[TIME] of var ACTION: choice;

constraint forall(t in TIME)(tp[t] = temp[t]);

enum ACTION = { heat, strongly_heat, cool, strongly_cool, do_nothing};
array[ACTION] of int: adjust = [+1, +4, -2, -5, 0]; 
array[ACTION] of int: c = [1, 5, 3 , 9 ,0];


constraint temp[0] = start;
constraint forall(t in 1..l)(temp[t] = (readings[t] + temp[t-1]) div 2
                                       + adjust[choice[t]]);

var int: cost = sum(t in TIME)(c[choice[t]]);

solve minimize cost;

output ["temp = \(tp);\nchoice = \(choice);\ncost = \(cost);"];