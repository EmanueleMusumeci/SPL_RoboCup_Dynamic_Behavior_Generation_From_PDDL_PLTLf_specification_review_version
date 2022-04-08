begin_version
3.FOND
end_version
begin_metric
0
end_metric
6
begin_variable
var0
-1
2
Atom ball-passed()
NegatedAtom ball-passed()
end_variable
begin_variable
var1
-1
2
Atom goal-scored()
NegatedAtom goal-scored()
end_variable
begin_variable
var2
-1
2
Atom is-opponent-blocking-goal()
NegatedAtom is-opponent-blocking-goal()
end_variable
begin_variable
var3
0
2
Atom new-axiom@0()
NegatedAtom new-axiom@0()
end_variable
begin_variable
var4
-1
2
Atom striker-can-kick()
NegatedAtom striker-can-kick()
end_variable
begin_variable
var5
-1
2
Atom striker-has-ball()
NegatedAtom striker-has-ball()
end_variable
0
begin_state
1
1
1
1
1
1
end_state
begin_goal
1
3 0
end_goal
4
begin_operator
kick-to-goal 
2
2 1
4 0
1
1
0 1 -1 0
0
end_operator
begin_operator
move-to-ball 
1
5 1
2
2
0 2 -1 0
0 5 1 0
2
0 2 -1 1
0 5 1 0
0
end_operator
begin_operator
move-to-kicking-position 
3
2 1
4 1
5 0
2
0
2
0 2 1 0
0 4 1 0
0
end_operator
begin_operator
pass-ball-to-supporter 
2
2 0
4 1
1
1
0 0 -1 0
0
end_operator
2
begin_rule
1
0 0
3 1 0
end_rule
begin_rule
1
1 0
3 1 0
end_rule
