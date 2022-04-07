begin_version
3.FOND
end_version
begin_metric
0
end_metric
5
begin_variable
var0
-1
2
Atom not-flattire()
NegatedAtom not-flattire()
end_variable
begin_variable
var1
-1
2
Atom spare-in(l21)
NegatedAtom spare-in(l21)
end_variable
begin_variable
var2
-1
2
Atom spare-in(l22)
NegatedAtom spare-in(l22)
end_variable
begin_variable
var3
-1
2
Atom spare-in(l31)
NegatedAtom spare-in(l31)
end_variable
begin_variable
var4
-1
6
Atom vehicleat(l11)
Atom vehicleat(l12)
Atom vehicleat(l13)
Atom vehicleat(l21)
Atom vehicleat(l22)
Atom vehicleat(l31)
end_variable
1
begin_mutex_group
6
4 0
4 1
4 2
4 3
4 4
4 5
end_mutex_group
begin_state
0
0
0
0
0
end_state
begin_goal
1
4 4
end_goal
11
begin_operator
changetire l21
2
1 0
4 3
1
2
0 0 -1 0
0 1 0 1
0
end_operator
begin_operator
changetire l22
2
2 0
4 4
1
2
0 0 -1 0
0 2 0 1
0
end_operator
begin_operator
changetire l31
2
3 0
4 5
1
2
0 0 -1 0
0 3 0 1
0
end_operator
begin_operator
move-car l11 l12
2
0 0
4 0
2
2
0 0 0 1
0 4 0 1
1
0 4 0 1
0
end_operator
begin_operator
move-car l11 l21
2
0 0
4 0
2
2
0 0 0 1
0 4 0 3
1
0 4 0 3
0
end_operator
begin_operator
move-car l12 l13
2
0 0
4 1
2
2
0 0 0 1
0 4 1 2
1
0 4 1 2
0
end_operator
begin_operator
move-car l13 l22
2
0 0
4 2
2
2
0 0 0 1
0 4 2 4
1
0 4 2 4
0
end_operator
begin_operator
move-car l21 l12
2
0 0
4 3
2
2
0 0 0 1
0 4 3 1
1
0 4 3 1
0
end_operator
begin_operator
move-car l21 l22
2
0 0
4 3
2
2
0 0 0 1
0 4 3 4
1
0 4 3 4
0
end_operator
begin_operator
move-car l21 l31
2
0 0
4 3
2
2
0 0 0 1
0 4 3 5
1
0 4 3 5
0
end_operator
begin_operator
move-car l31 l22
2
0 0
4 5
2
2
0 0 0 1
0 4 5 4
1
0 4 5 4
0
end_operator
0
