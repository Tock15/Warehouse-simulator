% Admissible heuristics (plan section 3): h = max(h_match, h_tour).
% Every way of moving a box one cell (PICK, a carried step, DROP) costs 1 + w, and an empty
% step costs 1. So a plan's cost splits exactly into
%   robot moves (1 per step) + w per cell each box moves + 1 per PICK/DROP.
% h_match bounds each box's own movement, with boxes on distinct goals: the boxes' view.
% h_tour bounds the robot's route from box to box: the robot's view.
% Both fail on a dead state: some box cannot reach any goal, even in the relaxation.
:- module(heuristic, [build_dist/0, dist/4, h_match/2, h_tour/2, h/2]).
:- use_module(level).
:- use_module(rules).
:- use_module(library(assoc)).
:- dynamic dist_row/3.

% h(+State, -H): the combined heuristic. A max of admissible heuristics is admissible.
h(S, H) :- h_match(S, H1), h_tour(S, H2), H is max(H1, H2).

%% Distance tables ------------------------------------------------------------------------------

% build_dist: dist_row(W, From, Row) for W = 0..3 and every non-wall From, Row = row(D0..D99)
% indexed by cell number (n = unreachable). Run once after load_level/1.
build_dist :-
    retractall(dist_row(_, _, _)),
    forall(( between(0, 3, W), between(0, 99, I), cell_at(I, C), \+ wall(C) ),
           ( bfs(W, C, Row), assertz(dist_row(W, I, Row)) )).

% dist(+W, +From, +To, -D): relaxed shortest path for a robot carrying weight W: walls block,
% cells entered must hold W, belt cells are plain floor both ways, doors and boxes are ignored.
% Fails if To is unreachable. Fewer restrictions than the real moves, so D is a lower bound.
dist(W, A, B, D) :-
    cell_at(IA, A), dist_row(W, IA, Row),
    cell_at(IB, B), J is IB + 1, arg(J, Row, D),
    integer(D).

% cell_at(?I, ?Cell): cell number I = 10 * Y + X.
cell_at(I, X-Y) :- integer(I), !, X is I mod 10, Y is I // 10.
cell_at(I, X-Y) :- I is 10 * Y + X.

% bfs(+W, +Start, -Row): unit-cost distances from Start in the relaxed grid for weight W.
bfs(W, S, Row) :-
    list_to_assoc([S-0], A0),
    layers([S]-0, W, A0, A),
    findall(D, ( between(0, 99, I), cell_at(I, C), ( get_assoc(C, A, D) -> true ; D = n ) ), Ds),
    Row =.. [row|Ds].

% layers(+Frontier-D, +W, +Seen0, -Seen): breadth-first, one distance layer per call.
layers([]-_, _, A, A).
layers([C|Cs]-D, W, A0, A) :-
    D1 is D + 1,
    findall(N, ( member(X-Y, [C|Cs]), delta(_, DX, DY), NX is X + DX, NY is Y + DY,
                 N = NX-NY, relaxed(N, W), \+ get_assoc(N, A0, _) ), Ns0),
    sort(Ns0, Ns),
    foldl(seen(D1), Ns, A0, A1),
    layers(Ns-D1, W, A1, A).

% seen(+D, +Cell, +A0, -A): record Cell at distance D.
seen(D, C, A0, A) :- put_assoc(C, A0, D, A).

% relaxed(+Cell, +W): Cell may be entered carrying W in the relaxed grid.
relaxed(C, W) :- \+ wall(C), ( belt_cell(C) -> true ; can_hold(C, W) ).

%% Per-box bounds -------------------------------------------------------------------------------

% A box is described as box(Id, Where, W): Where = at(P) on the floor, held(R) in the robot's hands.

% boxes(+State, -All): every box, the held one included.
boxes(state(R, Held, Boxes), All) :-
    findall(box(Id, at(P), W), ( member(Id-P, Boxes), weight(Id, W) ), Floor),
    held(R, Held, Hs),
    append(Hs, Floor, All).

% held(+R, +Held, -Hs): [] or the held box.
held(_, none, []).
held(R, Id, [box(Id, held(R), W)]) :- Id \== none, weight(Id, W).

% unplaced(+Box): the box still has work to do (held, or not on a goal it accepts).
unplaced(box(_, held(_), _)).
unplaced(box(Id, at(P), _)) :- \+ accepts(Id, P).

% shift(+Box, +Goal, -D): lower bound on the cells the box moves to end on Goal. The box only
% rests on, or is held over, cells that hold w, and each PICK, carried step or DROP moves it
% one cell, so it moves at least dist(w). A held box must at least be dropped.
shift(box(_, at(P), _), P, 0).
shift(box(_, at(P), W), G, D) :- P \== G, dist(W, P, G, D).
shift(box(_, held(R), W), G, D) :- dist(W, R, G, D0), D is max(1, D0).

% option(+Box, -Goal-Cost): a goal the box accepts, and (1 + w) x shift.
option(B, G-Cost) :-
    B = box(Id, _, W),
    goal(G, GC), accepts(Id, GC),
    shift(B, GC, D),
    Cost is (1 + W) * D.

%% h_match: the boxes' view ---------------------------------------------------------------------

% h_match(+State, -H): cheapest assignment of boxes to distinct accepted goals, summing each
% box's (1 + w) x shift, plus the empty walk to the first box picked. Box moves of different
% boxes never overlap (one box at a time), nor with that first walk.
h_match(S, H) :-
    boxes(S, Bs),
    maplist(options, Bs, Opts),
    aggregate_all(min(C), assign(Opts, [], C), M),
    first_walk(S, Bs, Wk),
    H is M + Wk.

% options(+Box, -Options): every Goal-Cost the box could finish on.
options(B, Os) :- findall(O, option(B, O), Os).

% assign(+Options, +Used, -Cost): pick one goal per box, never a goal already Used.
% shortcut: brute force over goals, fine for 4 boxes and a handful of goals; bound it if
% levels get many more goals than boxes.
assign([], _, 0).
assign([Os|Oss], Used, C) :-
    member(G-C1, Os), \+ memberchk(G, Used),
    assign(Oss, [G|Used], C2),
    C is C1 + C2.

% first_walk(+State, +Boxes, -Wk): an empty robot with work left must first walk next to some
% box (perhaps a placed one, moved out of the way): at least the nearest one, dist - 1.
first_walk(state(R, none, _), Bs, Wk) :-
    include(unplaced, Bs, [_|_]),
    aggregate_all(min(D1), ( member(box(_, at(P), _), Bs), dist(0, R, P, D), D1 is max(0, D - 1) ), Wk).
first_walk(state(_, none, _), Bs, 0) :- \+ include(unplaced, Bs, [_|_]).
first_walk(state(_, Id, _), _, 0) :- Id \== none.

%% h_tour: the robot's view ---------------------------------------------------------------------

% A job is j(Where, W, Shift, Ops, Last): an unplaced box, its least shift to an accepted goal,
% its PICK/DROP count (2 on the floor, 1 held) and Last, the robot moves it needs once picked.

% h_tour(+State, -H): robot moves + w x shift + PICK/DROPs, each bounded separately.
% Robot moves: order the unplaced floor boxes by when each is first picked. The robot must
% stand next to each in turn, then still move Last for the last one (it is picked after
% every other box was reached, and the robot ends next to its goal).
h_tour(state(R, Held, Boxes), H) :-
    boxes(state(R, Held, Boxes), Bs),
    include(unplaced, Bs, Us),
    maplist(job, Us, Jobs),
    foldl(weight_ops, Jobs, 0, WO),
    partition(floor_job, Jobs, Floor, HeldJobs),
    aggregate_all(min(M), robot_moves(R, Floor, HeldJobs, M), Robot),
    H is Robot + WO.

% job(+Box, -Job): the box's bounds over its accepted goals (fails if it can reach none).
job(B, j(Where, W, Shift, Ops, Last)) :-
    B = box(Id, Where, W),
    aggregate_all(min(D), ( goal(_, GC), accepts(Id, GC), shift(B, GC, D) ), Shift),
    aggregate_all(min(M), ( goal(_, GC), accepts(Id, GC), last_moves(Where, GC, M) ), Last),
    ops(Where, Ops).

% ops(+Where, -N): PICKs + DROPs still needed at least.
ops(at(_), 2).
ops(held(_), 1).

% last_moves(+Where, +Goal, -M): robot moves after the box's first pick. The robot starts next
% to P (or on R) and ends next to Goal: dist - 2 (or dist - 1). Empty-robot grid: box moves by
% PICK/DROP need no robot move, so the carry grid would overestimate.
last_moves(at(P), G, M) :- dist(0, P, G, D), M is max(0, D - 2).
last_moves(held(R), G, M) :- dist(0, R, G, D), M is max(0, D - 1).

% weight_ops(+Job, +A0, -A): add the job's w x shift and PICK/DROPs.
weight_ops(j(_, W, Shift, Ops, _), A0, A) :- A is A0 + W * Shift + Ops.

% floor_job(+Job): the job's box is on the floor.
floor_job(j(at(_), _, _, _, _)).

% robot_moves(+R, +Floor, +Held, -M): robot moves for one order of the floor jobs. The held
% box, if any, counts as picked first (it is already up), so only alone is it the last one.
robot_moves(_, [], [], 0).
robot_moves(_, [], [j(_, _, _, _, Last)], Last).
robot_moves(R, [J|Js], _, M) :- permutation([J|Js], Order), tour(R, Order, M).

% tour(+R, +Order, -M): walk next to the first box (dist - 1), box to box (dist - 2: both
% ends are next to a box), then the last box's Last.
tour(R, [J|Js], M) :-
    J = j(at(P), _, _, _, _),
    dist(0, R, P, D), L is max(0, D - 1),
    legs([J|Js], Rest),
    M is L + Rest.

% legs(+Order, -M): box-to-box legs, then the last box's Last.
legs([j(_, _, _, _, Last)], Last).
legs([j(at(P1), _, _, _, _), J2|Js], M) :-
    J2 = j(at(P2), _, _, _, _),
    dist(0, P1, P2, D), L is max(0, D - 2),
    legs([J2|Js], M2),
    M is L + M2.
