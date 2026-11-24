% Run: swipl -g run_tests -t halt tests/test_rules.pl
% One scenario per rule of plan section 2. Open 8x8 board (border walls only) unless stated.
:- use_module('../prolog/level').
:- use_module('../prolog/rules').
:- use_module(library(plunit)).
:- use_module(library(assoc)).

% load(+Goals, +Boxes, +Robot, +Extra): load a level; Extra = [Fragile, Belts, Plates, Doors].
load(Goals, Boxes, Robot, [F, B, P, D]) :- load_level(level([], Goals, Boxes, Robot, F, B, P, D)).

% children(+State, -Actions): every action-Cost pair from State.
children(S, As) :- findall(A-C, action(S, A, C, _), As).

:- begin_tests(fragile).

test(carry_onto, [nondet, setup(load([], [], 1-1, [[fragile(5-5, 2)], [], [], []]))]) :-
    step(4-5, ctx(0, []), r, 5-5),
    step(4-5, ctx(2, []), r, 5-5),
    \+ step(4-5, ctx(3, []), r, 5-5).
test(drop_onto, [setup(load([], [box(a, 2-2, 3, any), box(b, 2-4, 2, any)], 1-1,
                            [[fragile(5-5, 2)], [], [], []]))]) :-
    children(state(4-5, a, [b-(2-4)]), A3),
    \+ memberchk(deliver(a, _, 5-5)-_, A3),
    children(state(4-5, b, [a-(2-2)]), A2),
    memberchk(deliver(b, 4-5, 5-5)-3, A2).
test(pick_while_standing_on, [setup(load([], [box(a, 6-5, 3, any)], 5-5,
                                         [[fragile(5-5, 2)], [], [], []]))]) :-
    children(state(5-5, none, [a-(6-5)]), As),
    \+ memberchk(fetch(a, 5-5)-_, As),
    memberchk(fetch(a, 7-5)-_, As).

:- end_tests(fragile).

:- begin_tests(belt, [setup(load([], [], 1-5, [[], [belt(2-5, 5-5)], [], []]))]).

test(ride_from_behind, [nondet, true(C == 4)]) :- step(1-5, ctx(0, []), ride(5-5, 4), 5-5), move_cost(ride(5-5, 4), 0, C).
test(ride_from_side, nondet) :- step(2-4, ctx(0, []), ride(5-5, 4), 5-5).
test(ride_cost_carrying, C == 12) :- move_cost(ride(5-5, 4), 2, C).
test(never_stand_on_belt) :-
    forall((between(1, 8, X), between(1, 8, Y)),
           \+ (step(X-Y, ctx(0, []), _, To), level:belt_cell(To))).
test(never_back, nondet) :-
    dijkstra(5-5, ctx(0, []), T),
    route(T, 1-5, Moves),
    \+ memberchk(ride(_, _), Moves).
test(shortest_uses_belt, [nondet, true(Moves-C == [ride(5-5, 4), r]-5)]) :-
    dijkstra(1-5, ctx(0, []), T),
    route(T, 6-5, Moves),
    get_assoc(6-5, T, C-_).
test(exit_blocked) :- \+ step(1-5, ctx(0, [a-(5-5)]), ride(_, _), _).
test(no_drop_on_belt) :-
    children(state(2-4, a, []), As),
    forall(level:belt_cell(B), \+ memberchk(deliver(a, _, B)-_, As)).

:- end_tests(belt).

:- begin_tests(door, [setup(load([], [], 1-1, [[], [], [2-2], [5-5]]))]).

test(closed_without_plate_box) :- \+ door_open([a-(3-3)]), \+ step(4-5, ctx(0, [a-(3-3)]), r, 5-5).
test(open_with_plate_box, nondet) :- door_open([a-(2-2)]), step(4-5, ctx(0, [a-(2-2)]), r, 5-5).
test(no_drop_on_door) :-
    children(state(4-5, a, [b-(2-2)]), As),
    \+ memberchk(deliver(a, _, 5-5)-_, As).

:- end_tests(door).

:- begin_tests(pick_plate_box_from_door).

test(last_plate_box, [setup(load([], [box(a, 3-3, 1, any)], 5-3, [[], [], [3-3], [4-3]]))]) :-
    children(state(5-3, none, [a-(3-3)]), As),
    \+ memberchk(fetch(a, 4-3)-_, As),
    memberchk(fetch(a, 2-3)-_, As).
test(other_plate_still_covered,   % door stays open after the lift, so the pick is fine
     [setup(load([], [box(a, 3-3, 1, any), box(b, 7-7, 1, any)], 5-3, [[], [], [3-3, 7-7], [4-3]]))]) :-
    children(state(5-3, none, [a-(3-3), b-(7-7)]), As),
    memberchk(fetch(a, 4-3)-3, As).

:- end_tests(pick_plate_box_from_door).

:- begin_tests(goals, [setup(load([goal(g1, 2-2), goal(g2, 7-7)],
                                  [box(a, 4-4, 1, only(g1)), box(b, 5-5, 1, any)], 1-1,
                                  [[], [], [], []]))]).

test(pinned_on_own_goal) :- goal_state(state(1-1, none, [a-(2-2), b-(7-7)])).
test(pinned_on_other_goal) :- \+ goal_state(state(1-1, none, [a-(7-7), b-(2-2)])).
test(off_goal) :- \+ goal_state(state(1-1, none, [a-(2-2), b-(5-5)])).
test(holding_is_not_done) :- \+ goal_state(state(1-1, b, [a-(2-2)])).
test(initial, S == state(1-1, none, [a-(4-4), b-(5-5)])) :- initial_state(S).

:- end_tests(goals).

:- begin_tests(actions, [setup(load([], [box(a, 4-1, 2, any), box(b, 1-3, 1, any)], 1-1,
                                    [[], [], [], []]))]).

test(fetch_cost, [nondet, true(C == 5)]) :- action(state(1-1, none, [a-(4-1), b-(1-3)]), fetch(a, 3-1), C, _).
test(fetch_next, [nondet, true(N == state(3-1, a, [b-(1-3)]))]) :-
    action(state(1-1, none, [a-(4-1), b-(1-3)]), fetch(a, 3-1), _, N).
test(deliver_cost, [nondet, true(C == 12)]) :-       % 3 carried steps x (1 + 2) + DROP (1 + 2)
    action(state(3-1, a, [b-(1-3)]), deliver(a, 6-1, 7-1), C, _).
test(deliver_next, [nondet, true(N == state(6-1, none, [a-(7-1), b-(1-3)]))]) :-
    action(state(3-1, a, [b-(1-3)]), deliver(a, 6-1, 7-1), _, N).
test(boxes_block) :- \+ step(1-2, ctx(0, [b-(1-3)]), d, 1-3).
test(no_drop_on_box) :-
    children(state(1-2, a, [b-(1-3)]), As),
    \+ memberchk(deliver(a, _, 1-3)-_, As).

:- end_tests(actions).
