% Run: swipl -g run_tests -t halt tests/test_heuristic.pl
:- use_module('../prolog/level').
:- use_module('../prolog/rules').
:- use_module('../prolog/heuristic').
:- use_module('../prolog/search').
:- use_module(library(plunit)).

% load(+Level): load a level and build its distance tables.
load(L) :- load_level(L), build_dist.

% board(+Goals, +Boxes, +Robot, -Level): open 8x8 board, no special tiles.
board(Gs, Bs, R, level([], Gs, Bs, R, [], [], [], [])).

% room(+MaxX, +MaxY, -Walls): walls filling every inner cell outside 1..MaxX x 1..MaxY.
room(MX, MY, Walls) :-
    findall(X-Y, (between(1, 8, X), between(1, 8, Y), (X > MX ; Y > MY)), Ws), sort(Ws, Walls).

% level(?Name, -Level): small levels covering each tile type; UCS solves each in about a second.
level(one_box, L) :- board([goal(g1, 6-1)], [box(a, 3-1, 1, any)], 1-1, L).
level(heavy_box, L) :- board([goal(g1, 6-1)], [box(a, 3-1, 3, any)], 1-1, L).
level(heavy_room, level(Ws, [goal(g1, 6-1)], [box(a, 3-1, 3, any)], 1-1, [], [], [], [])) :-
    room(6, 3, Ws).
level(blocker, level(Ws, [goal(g1, 5-1), goal(g2, 4-4)],
                     [box(a, 3-4, 1, only(g1)), box(b, 1-1, 1, only(g2))], 1-3, [], [], [], [])) :-
    room(5, 5, Room), findall(X-2, between(2, 5, X), C), append(Room, C, Ws).
level(two_pinned, level(Ws, [goal(g1, 1-5), goal(g2, 5-5)],
                        [box(a, 5-2, 1, only(g1)), box(b, 1-2, 2, only(g2))], 3-1, [], [], [], [])) :-
    room(5, 5, Ws).
level(two_any, level(Ws, [goal(g1, 2-2), goal(g2, 6-3)],
                     [box(a, 4-1, 2, any), box(b, 5-3, 1, any)], 1-1, [], [], [], [])) :-
    room(6, 3, Ws).
level(door, level(Ws, [goal(g1, 7-4), goal(g2, 1-1)],
                  [box(a, 1-3, 1, only(g2)), box(b, 2-3, 1, only(g1))], 3-4, [], [], [2-1], [4-2])) :-
    room(7, 4, Room), findall(4-Y, (between(1, 4, Y), Y =\= 2), C), append(Room, C, Ws).
level(belt, level(Ws, [goal(g1, 6-2)], [box(a, 1-3, 2, any)], 1-1, [], [belt(2-2, 5-2)], [], [])) :-
    room(6, 3, Ws).
% Free key box k: parked on the plate at 2-1 it holds the door at 4-2 open for a, and may stay.
level(key, level(Ws, [goal(g1, 7-4)], [box(a, 2-3, 1, any), box(k, 1-3, 1, none)], 3-4, [], [], [2-1], [4-2])) :-
    room(7, 4, Room), findall(4-Y, (between(1, 4, Y), Y =\= 2), C), append(Room, C, Ws).
% Heavy key: carrying the weight-3 key to the plate is the cost h cannot see.
level(heavy_key, level(Ws, [goal(g1, 8-1)], [box(a, 2-3, 1, any), box(k, 3-5, 3, none)], 4-1,
                       [], [], [1-1], [5-3])) :-
    room(8, 5, Room), findall(5-Y, (between(1, 5, Y), Y =\= 3), C), append(Room, C, Ws).
level(fragile, level(Ws, [goal(g1, 5-2)], [box(a, 1-2, 2, any)], 1-1,
                     [fragile(3-1, 1), fragile(3-2, 1)], [], [], [])) :-
    room(5, 3, Ws).

% optimal(+Name, -States, -Cost): UCS plan for the level, as the states it passes through.
optimal(Name, States, Cost) :-
    level(Name, L), load(L),
    initial_state(S0),
    search(ucs, S0, found(As, Cost, _)),
    foldl(apply_action, As, [S0-0], Trail),
    reverse(Trail, States).

% apply_action(+Action, +Trail0, -Trail): extend the trail of State-CostSoFar.
apply_action(A, [S-G|T], [S1-G1, S-G|T]) :- once(action(S, A, C, S1)), G1 is G + C.

% admissible(+Name): at every state on an optimal plan, both heuristics are defined and
% at most the true remaining cost (a suffix of an optimal plan is optimal).
admissible(Name) :-
    optimal(Name, States, Cost),
    forall(member(S-G, States),
           ( Rem is Cost - G,
             h_match(S, H1), H1 =< Rem,
             h_tour(S, H2), H2 =< Rem )).

% admissible_off_path(+Name): every child of every state on an optimal plan (good moves and
% bad ones) gets h at most its true cost to go, found by UCS from that child. Dead children
% must really be unsolvable.
admissible_off_path(Name) :-
    optimal(Name, States, _),
    forall(( member(S-_, States), action(S, _, _, C) ),
           ( search(ucs, C, R),
             (   R = found(_, Star, _)
             ->  h_match(C, H1), H1 =< Star, h_tour(C, H2), H2 =< Star
             ;   true                      % unsolvable: any h, or none, is fine
             ) )).

% run(+Name, +Algo, -Cost, -Nodes): solve the level, check the plan replays to a win at Cost.
run(Name, Algo, Cost, Nodes) :-
    level(Name, L), load(L),
    initial_state(S0),
    search(Algo, S0, found(As, Cost, Nodes)),
    foldl(apply_action, As, [S0-0], [End-Cost|_]),
    goal_state(End).

:- begin_tests(dist).

test(open, D == 3) :- board([], [], 1-1, L), load(L), dist(0, 1-1, 4-1, D).
test(wall_detour, D == 4) :-
    load(level([2-1], [], [], 1-1, [], [], [], [])), dist(0, 1-1, 3-1, D).
test(fragile_detour, Ds == [2, 2, 4]) :-
    load(level([], [], [], 1-1, [fragile(2-1, 1)], [], [], [])),
    findall(D, (member(W, [0, 1, 2]), dist(W, 1-1, 3-1, D)), Ds).
test(belt_both_ways, D == 3) :-
    load(level([], [], [], 1-1, [], [belt(2-5, 5-5)], [], [])), dist(0, 5-5, 2-5, D).
test(doors_ignored, D == 2) :-
    load(level([], [], [], 1-1, [], [], [7-7], [2-1])), dist(0, 1-1, 3-1, D).
test(unreachable, fail) :-
    load(level([2-1, 1-2], [], [], 5-5, [], [], [], [])), dist(0, 5-5, 1-1, _).

:- end_tests(dist).

:- begin_tests(values).

test(goal_state_is_zero, [nondet, true(H == 0)]) :-
    board([goal(g1, 3-3)], [box(a, 3-3, 1, any)], 1-1, L), load(L),
    h(state(1-1, none, [a-(3-3)]), H).
% One box: both are walk 1 + 3 cells x (1 + 1) = 7 (true cost 11: the box can't go straight).
test(one_box, [nondet, true(H1-H2 == 7-7)]) :-
    level(one_box, L), load(L), initial_state(S), h_match(S, H1), h_tour(S, H2).
% Held weight-1 box at 3-1, goal 6-1: carry 2 cells + DROP = 3 box moves x 2 = 6, exact.
test(held_box, [nondet, true(H1-H2 == 6-6)]) :-
    level(one_box, L), load(L), h_match(state(3-1, a, []), H1), h_tour(state(3-1, a, []), H2).
test(dead_box_walled_in, fail) :-
    load(level([2-1, 1-2], [goal(g1, 6-6)], [box(a, 1-1, 1, any)], 5-5, [], [], [], [])),
    h(state(5-5, none, [a-(1-1)]), _).

% Neither dominates (plan section 3, proof 6).
% Boxes next to their goals, far from the robot and each other: the route matters.
% h_match = first walk 6 + 2 x 1 + 2 x 1 = 10.
% h_tour = robot 6 + (7 - 2) + 0, plus w x shift + PICK/DROP (1 + 2) per box = 17.
test(tour_wins, [nondet, true(H1-H2 == 10-17)]) :-
    board([goal(g1, 8-2), goal(g2, 7-8)], [box(a, 8-1, 1, any), box(b, 8-8, 1, any)], 1-1, L),
    load(L), initial_state(S), h_match(S, H1), h_tour(S, H2).
% Both boxes next to the robot, both far from goals, one near goal each: carrying matters.
% h_match = 2 x 12 + 2 x 13 = 50 (distinct goals). h_tour = robot 0 + 0 + 10, plus
% (1 x 12 + 2) per box = 38: it counts the robot's steps under only the last box.
test(match_wins, [nondet, true(H1-H2 == 50-38)]) :-
    board([goal(g1, 8-8), goal(g2, 8-7)], [box(a, 2-1, 1, any), box(b, 1-2, 1, any)], 1-1, L),
    load(L), initial_state(S), h_match(S, H1), h_tour(S, H2).

% A free box adds nothing: the goal state ignores it, and h stays defined (not "dead") while
% the robot holds it. Here a is on its goal, so h = 0 although k must still be dropped.
test(free_box_ignored_by_goal_state) :-
    level(key, L), load(L), goal_state(state(5-5, none, [a-(7-4), k-(3-3)])).
test(free_box_held, [nondet, true(H == 0)]) :-
    level(key, L), load(L), h(state(3-3, k, [a-(7-4)]), H).
test(free_box_held_work_left, [nondet, true(H > 0)]) :-
    level(key, L), load(L), h(state(2-2, k, [a-(2-3)]), H).

:- end_tests(values).

:- begin_tests(admissible).

test(one_box) :- admissible(one_box).
test(heavy_box) :- admissible(heavy_box).
test(heavy_room) :- admissible(heavy_room).
test(blocker) :- admissible(blocker).
test(two_pinned) :- admissible(two_pinned).
test(two_any) :- admissible(two_any).
test(door) :- admissible(door).
test(belt) :- admissible(belt).
test(fragile) :- admissible(fragile).

test(heavy_room_off_path) :- admissible_off_path(heavy_room).
test(belt_off_path) :- admissible_off_path(belt).
test(fragile_off_path) :- admissible_off_path(fragile).
test(key) :- admissible(key).
test(heavy_key) :- admissible(heavy_key).
test(key_off_path) :- admissible_off_path(key).

:- end_tests(admissible).

:- begin_tests(algorithms).

% Plan section 8: UCS and A* return the same cost, greedy one that is >= it.
test(astar_optimal_greedy_not_better) :-
    forall(level(Name, _),
           ( run(Name, ucs, U, _), run(Name, astar, A, _), run(Name, greedy, G, _),
             assertion(A =:= U), assertion(G >= U) )).
test(dead_start_pruned, R == no_solution(0)) :-
    load(level([2-1, 1-2], [goal(g1, 6-6)], [box(a, 1-1, 1, any)], 5-5, [], [], [], [])),
    initial_state(S), search(astar, S, R).

:- end_tests(algorithms).
