% Run: swipl -g run_tests -t halt tests/test_search.pl
:- use_module('../prolog/level').
:- use_module('../prolog/rules').
:- use_module('../prolog/search').
:- use_module(library(plunit)).

% solve_ucs(+Level, -Result): load Level and run UCS from its start.
solve_ucs(L, R) :- load_level(L), initial_state(S), search(ucs, S, R).

% room(+MaxX, +MaxY, -Walls): walls filling every inner cell outside 1..MaxX x 1..MaxY.
room(MX, MY, Walls) :-
    findall(X-Y, (between(1, 8, X), between(1, 8, Y), (X > MX ; Y > MY)), Ws), sort(Ws, Walls).

% replay(+State, +Actions, -End, -Cost): apply the actions with action/4, summing their costs.
replay(S, [], S, 0).
replay(S, [A|As], End, C) :- once(action(S, A, C1, S1)), replay(S1, As, End, C2), C is C1 + C2.

% check_plan(+Result): the plan replays from the start, ends in a goal state, costs what it says.
check_plan(found(As, Cost, _)) :-
    initial_state(S), replay(S, As, End, Cost), goal_state(End).

:- begin_tests(ucs).

test(already_solved, R == found([], 0, 0)) :-
    solve_ucs(level([], [goal(g1, 3-3)], [box(a, 3-3, 1, any)], 1-1, [], [], [], []), R).

% Walk 1 + PICK + carry 3 x (1 + 1) + DROP = 9.
test(one_box, Cost == 9) :-
    solve_ucs(level([], [goal(g1, 6-1)], [box(a, 3-1, 1, any)], 1-1, [], [], [], []), R),
    R = found(_, Cost, _), check_plan(R).

% Weight 3: cheaper to walk empty around to 4-1 (5) + PICK + carry 1 x 4 + DROP = 11.
test(heavy_box, Cost == 11) :-
    solve_ucs(level([], [goal(g1, 6-1)], [box(a, 3-1, 3, any)], 1-1, [], [], [], []), R),
    R = found(_, Cost, _), check_plan(R).

% Box in the corner, walled in: the robot can never stand next to it.
test(unreachable_box, [nondet, true(R = no_solution(_))]) :-
    solve_ucs(level([2-1, 1-2], [goal(g1, 6-6)], [box(a, 1-1, 1, any)], 5-5, [], [], [], []), R).

% Blocker: b sits in the only way into the dead-end corridor along row 1, where a's goal is.
% b must be moved out first. Room 5x5.
test(blocker) :-
    room(5, 5, Room),
    findall(X-2, between(2, 5, X), Corridor),
    append(Room, Corridor, Walls),
    solve_ucs(level(Walls, [goal(g1, 5-1), goal(g2, 4-4)],
                    [box(a, 3-4, 1, only(g1)), box(b, 1-1, 1, only(g2))], 1-3, [], [], [], []), R),
    R = found([fetch(b, _)|_], _, _), check_plan(R).

% Two boxes, pinned goals on the opposite side from where they start. Room 5x5.
test(two_boxes_pinned) :-
    room(5, 5, Walls),
    solve_ucs(level(Walls, [goal(g1, 1-5), goal(g2, 5-5)],
                    [box(a, 5-2, 1, only(g1)), box(b, 1-2, 2, only(g2))], 3-1, [], [], [], []), R),
    check_plan(R).

% Door at 4-2 in wall column x = 4: some box must sit on the plate at 2-1 while b is carried
% through to its goal on the right. Room 7x4.
test(door_needs_plate) :-
    room(7, 4, Room),
    findall(4-Y, (between(1, 4, Y), Y =\= 2), Column),
    append(Room, Column, Walls),
    solve_ucs(level(Walls, [goal(g1, 7-4), goal(g2, 1-1)],
                    [box(a, 1-3, 1, only(g2)), box(b, 2-3, 1, only(g1))], 3-4,
                    [], [], [2-1], [4-2]), R),
    check_plan(R).

:- end_tests(ucs).
