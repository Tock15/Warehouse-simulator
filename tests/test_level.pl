% Run: swipl -g run_tests -t halt tests/test_level.pl
:- use_module('../prolog/level').
:- use_module(library(plunit)).

% sample(-Level): one of everything; robot at 1-1, belt 2-5 -> 5-5, vertical belt 7-2 -> 7-4.
sample(level([4-4], [goal(g1, 8-8), goal(g2, 8-1)],
             [box(a, 3-3, 2, any), box(b, 6-6, 3, only(g1))],
             1-1, [fragile(2-2, 1)], [belt(2-5, 5-5), belt(7-2, 7-4)], [1-8], [5-1])).

:- begin_tests(level, [setup((sample(L), load_level(L)))]).

test(border_walls) :-
    aggregate_all(count, (level:wall(X-Y), once((X =:= 0 ; X =:= 9 ; Y =:= 0 ; Y =:= 9))), 36).
test(inner_wall) :- level:wall(4-4).
test(no_extra_walls) :- aggregate_all(count, level:wall(_), 37).
test(goals, Gs == [g1-(8-8), g2-(8-1)]) :- findall(G-P, level:goal(G, P), Gs).
test(boxes, Bs == [a-(3-3)-2-any, b-(6-6)-3-only(g1)]) :-
    findall(I-P-W-T, (level:box_start(I, P), level:weight(I, W), level:target(I, T)), Bs).
test(robot, P == 1-1) :- level:robot_start(P).
test(fragile) :- level:fragile(2-2, 1).
test(plate_door) :- level:plate(1-8), level:door(5-1).
test(belt_horizontal, L == 4) :- level:belt(2-5, 5-5, L).
test(belt_vertical, L == 3) :- level:belt(7-2, 7-4, L).
test(belt_cells, Cs == [2-5, 3-5, 4-5, 7-2, 7-3]) :- findall(C, level:belt_cell(C), Cs).
test(reload_clears) :-
    load_level(level([], [goal(g1, 2-2)], [box(z, 3-3, 1, any)], 4-4, [], [], [], [])),
    \+ level:box_start(a, _), \+ level:belt(_, _, _), \+ level:belt_cell(_),
    \+ level:plate(_), \+ level:door(_), \+ level:fragile(_, _), \+ level:wall(4-4),
    aggregate_all(count, level:goal(_, _), 1), aggregate_all(count, level:robot_start(_), 1).

:- end_tests(level).
