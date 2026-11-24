% Run: swipl -g run_tests -t halt tests/test_solve.pl
:- use_module('../prolog/solve').
:- use_module(library(plunit)).

% lv(+Name, -Level): small levels; base is valid, the others break one rule each.
lv(base, level([], [goal(g1, 6-1)], [box(a, 3-1, 1, any)], 1-1, [], [], [], [])).
lv(no_boxes, level([], [goal(g1, 6-1)], [], 1-1, [], [], [], [])).
lv(five_boxes, level([], Gs, Bs, 1-1, [], [], [], [])) :-
    findall(goal(G, X-8), (member(G-X, [g1-1, g2-2, g3-3, g4-4, g5-5])), Gs),
    findall(box(B, X-5, 1, any), (member(B-X, [a-1, b-2, c-3, d-4, e-5])), Bs).
lv(dup_box, level([], [goal(g1, 6-1), goal(g2, 6-2)], [box(a, 3-1, 1, any), box(a, 4-4, 1, any)], 1-1, [], [], [], [])).
lv(dup_goal, level([], [goal(g1, 6-1), goal(g1, 6-2)], [box(a, 3-1, 1, any)], 1-1, [], [], [], [])).
lv(missing_goal, level([], [goal(g1, 6-1)], [box(a, 3-1, 1, only(g9))], 1-1, [], [], [], [])).
lv(shared_goal, level([], [goal(g1, 6-1), goal(g2, 6-2)],
                      [box(a, 3-1, 1, only(g1)), box(b, 4-4, 1, only(g1))], 1-1, [], [], [], [])).
lv(few_goals, level([], [goal(g1, 6-1)], [box(a, 3-1, 1, any), box(b, 4-4, 1, any)], 1-1, [], [], [], [])).
lv(goal_on_wall, level([], [goal(g1, 0-3)], [box(a, 3-1, 1, any)], 1-1, [], [], [], [])).
lv(goals_stacked, level([], [goal(g1, 6-1), goal(g2, 6-1)], [box(a, 3-1, 1, any)], 1-1, [], [], [], [])).
lv(belts_cross, level([], [goal(g1, 6-1)], [box(a, 3-1, 1, any)], 1-1, [],
                      [belt(2-5, 5-5), belt(3-3, 3-7)], [], [])).
lv(belt_into_wall, level([], [goal(g1, 6-1)], [box(a, 3-1, 1, any)], 1-1, [], [belt(5-2, 5-0)], [], [])).
lv(fragile_belt, level([], [goal(g1, 6-1)], [box(a, 3-1, 1, any)], 1-1, [fragile(3-5, 1)],
                       [belt(2-5, 5-5)], [], [])).
lv(boxes_stacked, level([], [goal(g1, 6-1), goal(g2, 6-2)], [box(a, 3-1, 1, any), box(b, 3-1, 1, any)], 1-1, [], [], [], [])).
lv(box_on_door, level([], [goal(g1, 6-1)], [box(a, 3-1, 1, any)], 1-1, [], [], [7-7], [3-1])).
lv(box_too_heavy, level([], [goal(g1, 6-1)], [box(a, 3-1, 3, any)], 1-1, [fragile(3-1, 2)], [], [], [])).
lv(robot_on_wall, level([4-4], [goal(g1, 6-1)], [box(a, 3-1, 1, any)], 4-4, [], [], [], [])).
lv(robot_on_box, level([], [goal(g1, 6-1)], [box(a, 3-1, 1, any)], 3-1, [], [], [], [])).
lv(door_no_plate, level([], [goal(g1, 6-1)], [box(a, 3-1, 1, any)], 1-1, [], [], [], [5-5])).
lv(plate_no_door, level([], [goal(g1, 6-1)], [box(a, 3-1, 1, any)], 1-1, [], [], [5-5], [])).
lv(box_walled_in, level([2-1, 1-2], [goal(g1, 6-1)], [box(a, 1-1, 1, any)], 5-5, [], [], [], [])).
lv(goal_walled_in, level([2-1, 1-2], [goal(g1, 6-1), goal(g2, 1-1)],
                         [box(a, 3-3, 1, any), box(b, 4-4, 1, any)], 5-5, [], [], [], [])).
% Doorstop (plan section 2): a's goal is past the door at 4-2; the door only opens with a box
% on the plate at 2-1. With b to park there it works; alone, a can't open the door for itself.
lv(doorstop, level(Ws, [goal(g1, 7-4), goal(g2, 1-1)],
                   [box(a, 2-3, 1, only(g1)), box(b, 1-3, 1, only(g2))], 3-4, [], [], [2-1], [4-2])) :-
    room(Ws).
lv(doorstop_alone, level(Ws, [goal(g1, 7-4)], [box(a, 2-3, 1, only(g1))], 3-4, [], [], [2-1], [4-2])) :-
    room(Ws).

% room(-Walls): a 7x4 room split by a wall at x = 4, with a gap for the door at 4-2.
room(Ws) :-
    findall(X-Y, (between(1, 8, X), between(1, 8, Y), (X > 7 ; Y > 4)), Out),
    findall(4-Y, (between(1, 4, Y), Y =\= 2), Column),
    append(Out, Column, Ws).

% reason(+Name, -Reason): what validate/2 says about level Name.
reason(Name, R) :- lv(Name, L), validate(L, error(R)).

:- begin_tests(validate).

test(ok) :- lv(base, L), validate(L, ok).
test(no_boxes, R == 'A level needs 1 to 4 boxes') :- reason(no_boxes, R).
test(five_boxes, R == 'A level needs 1 to 4 boxes') :- reason(five_boxes, R).
test(dup_box, R == 'Two boxes are named a') :- reason(dup_box, R).
test(dup_goal, R == 'Two goals are named g1') :- reason(dup_goal, R).
test(missing_goal, R == 'Box a needs goal g9, which does not exist') :- reason(missing_goal, R).
test(shared_goal, R == 'Boxes a and b both need goal g1') :- reason(shared_goal, R).
test(few_goals, R == '2 boxes but only 1 goals') :- reason(few_goals, R).
test(goal_on_wall, R == 'Cell (0, 3) is both a goal and a wall') :- reason(goal_on_wall, R).
test(goals_stacked, R == 'Cell (6, 1) has two goals') :- reason(goals_stacked, R).
test(belts_cross, R == 'Two belts cross at (3, 5)') :- reason(belts_cross, R).
test(belt_into_wall, R == 'The belt ending at (5, 0) ends on a wall') :- reason(belt_into_wall, R).
test(fragile_belt, R == 'Cell (3, 5) is both fragile and a belt') :- reason(fragile_belt, R).
test(boxes_stacked, R == 'Two boxes start on (3, 1)') :- reason(boxes_stacked, R).
test(box_on_door, R == 'Box a starts on a door at (3, 1)') :- reason(box_on_door, R).
test(box_too_heavy, R == 'Box a (weight 3) is too heavy for the fragile tile at (3, 1)') :-
    reason(box_too_heavy, R).
test(robot_on_wall, R == 'The robot starts on a wall at (4, 4)') :- reason(robot_on_wall, R).
test(robot_on_box, R == 'The robot starts on box a at (3, 1)') :- reason(robot_on_box, R).
test(door_no_plate, R == 'There are doors but no plates to open them') :- reason(door_no_plate, R).
test(plate_no_door, R == 'There are plates but no doors for them to open') :- reason(plate_no_door, R).
test(box_walled_in, R == 'Box a cannot reach any goal it may finish on') :- reason(box_walled_in, R).
test(goal_walled_in, R == 'The boxes cannot all reach different goals') :- reason(goal_walled_in, R).
test(doorstop) :- lv(doorstop, L), validate(L, ok).
test(doorstop_alone, R == 'This level has no solution') :- reason(doorstop_alone, R).

:- end_tests(validate).

:- begin_tests(solve).

% Cost 11 (see test_search.pl one_box); the plan ends with the drop onto the goal.
test(steps, [nondet, true(C-Last == 11-drop(a, 6-1))]) :-
    lv(base, L), solve(L, astar, found(Steps, C, _)),
    last(Steps, Last), memberchk(pick(a), Steps).
test(no_solution, [nondet, true(R = no_solution(_))]) :- lv(doorstop_alone, L), solve(L, astar, R).
test(malformed, R == error('Two boxes start on (3, 1)')) :- lv(boxes_stacked, L), solve(L, ucs, R).
test(flat, [nondet, true(Out = [found, _, 11, _])]) :-
    lv(base, L), bridge_solve(L, astar, Out),
    Out = [found, Fs, _, _], last(Fs, [drop, a, 6, 1]).

% With a tiny guard the bridge answers "too big" instead of hanging; the guard is restored after.
test(guard, [ setup((retract(solve:guard_seconds(Old)), assertz(solve:guard_seconds(0.001)))),
              cleanup((retractall(solve:guard_seconds(_)), assertz(solve:guard_seconds(Old)))),
              true(Out == [error, 'This level is too big to check in time']) ]) :-
    lv(doorstop, L), bridge_validate(L, Out).

:- end_tests(solve).
