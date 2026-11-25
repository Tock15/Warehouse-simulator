% Move rules (plan section 2): single steps, belt rides, shortest paths, fetch/deliver actions.
% State = state(Robot, Held, Boxes). Held = none | Id. Boxes = Id-Cell pairs sorted by Id,
% held box excluded. Ctx = ctx(W, Boxes): what the robot carries (W = 0 empty) and where boxes are.
:- module(rules, [delta/3, can_hold/2, door_open/1, step/4, move_cost/3, dijkstra/3, route/3,
                  action/4, initial_state/1, goal_state/1, accepts/2]).
:- use_module(level).
:- use_module(library(heaps)).
:- use_module(library(assoc)).

% delta(?Move, ?Dx, ?Dy): one grid step.
delta(u, 0, -1).
delta(d, 0, 1).
delta(l, -1, 0).
delta(r, 1, 0).

% neighbor(+Cell, -Next, -Move): Next is one Move away from Cell.
neighbor(X-Y, NX-NY, M) :- delta(M, DX, DY), NX is X + DX, NY is Y + DY.

% door_open(+Boxes): some box sits on a plate.
door_open(Boxes) :- member(_-P, Boxes), plate(P), !.

% can_hold(+Cell, +W): Cell can hold weight W (robot carrying W, or a box of weight W).
can_hold(C, _) :- \+ fragile(C, _).
can_hold(C, W) :- fragile(C, Max), W =< Max.

% passable(+Cell, +Boxes): no wall, box or closed door on Cell.
passable(C, Boxes) :- \+ wall(C), \+ memberchk(_-C, Boxes), \+ closed_door(C, Boxes).

% closed_door(+Cell, +Boxes): Cell is a door and no box is on a plate.
closed_door(C, Boxes) :- door(C), \+ door_open(Boxes).

% stand(+Cell, +Ctx): the robot may stop on Cell.
stand(C, ctx(W, Boxes)) :- passable(C, Boxes), \+ belt_cell(C), can_hold(C, W).

% step(+From, +Ctx, -Move, -To): one step (u/d/l/r), or stepping onto a belt entry from any
% side and riding to its exit (ride(Exit, Length)).
step(From, Ctx, M, To) :- neighbor(From, To, M), stand(To, Ctx).
step(From, Ctx, ride(Exit, L), Exit) :- neighbor(From, E, _), belt(E, Exit, L), stand(Exit, Ctx).

% move_cost(+Move, +W, -Cost): (1 + W) per cell walked; a ride counts its whole length.
% PICK and DROP also cost 1 + w: every way of moving a box one cell costs the same.
move_cost(M, W, C) :- delta(M, _, _), C is 1 + W.
move_cost(ride(_, L), W, C) :- C is (1 + W) * L.

% dijkstra(+Start, +Ctx, -Table): Table maps every cell the robot can reach to Cost-Via,
% Via = start | Prev/Move. Nothing moves during a walk, so door status is fixed.
dijkstra(Start, Ctx, Table) :-
    singleton_heap(H, 0, Start-start),
    empty_assoc(T0),
    settle(H, Ctx, T0, Table).

% settle(+Heap, +Ctx, +Done, -Table): pop the cheapest cell; its first pop has its shortest cost.
settle(H, _, T, T) :- empty_heap(H).
settle(H, Ctx, T0, T) :-
    get_from_heap(H, D, C-Via, H1),
    (   get_assoc(C, T0, _)
    ->  settle(H1, Ctx, T0, T)
    ;   put_assoc(C, T0, D-Via, T1),
        Ctx = ctx(W, _),
        findall(D1-(N-(C/M)), (step(C, Ctx, M, N), move_cost(M, W, K), D1 is D + K), Ns),
        foldl(push, Ns, H1, H2),
        settle(H2, Ctx, T1, T)
    ).

% push(+Priority-Key, +Heap0, -Heap): add_to_heap with foldl's argument order.
push(P-K, H0, H) :- add_to_heap(H0, P, K, H).

% route(+Table, +Cell, -Moves): the moves from the Dijkstra start to Cell.
route(T, C, Moves) :- route(T, C, [], Moves).

% route(+Table, +Cell, +Acc, -Moves): walk the Via links back to the start.
route(T, C, Acc, Moves) :- get_assoc(C, T, _-Via), route_via(Via, T, Acc, Moves).

% route_via(+Via, +Table, +Acc, -Moves): follow one Via link.
route_via(start, _, Acc, Acc).
route_via(P/M, T, Acc, Moves) :- route(T, P, [M|Acc], Moves).

% action(+State, -Action, -Cost, -Next): one macro action. One Dijkstra per call; backtracking
% (e.g. findall) enumerates every child from that one table.
% fetch(Id, Stand): walk empty to Stand next to box Id, PICK (cost 1 + w). Lifting must not
% close a door the robot stands on.
action(state(R, none, Boxes), fetch(Id, S), Cost, state(S, Id, Rest)) :-
    dijkstra(R, ctx(0, Boxes), T),
    select(Id-P, Boxes, Rest),
    neighbor(P, S, _),
    get_assoc(S, T, D-_),
    weight(Id, W),
    can_hold(S, W),
    ( door(S) -> door_open(Rest) ; true ),
    Cost is D + 1 + W.
% deliver(Id, Stand, Target): carry Id to Stand, DROP onto neighboring Target (cost 1 + w).
action(state(R, Id, Boxes), deliver(Id, S, Tg), Cost, state(S, none, Boxes1)) :-
    Id \== none,
    weight(Id, W),
    dijkstra(R, ctx(W, Boxes), T),
    gen_assoc(S, T, D-_),
    neighbor(S, Tg, _),
    droppable(Tg, W, Boxes),
    Cost is D + 1 + W,
    sort([Id-Tg|Boxes], Boxes1).

% droppable(+Cell, +W, +Boxes): a box of weight W may be dropped on Cell.
droppable(C, W, Boxes) :-
    \+ wall(C), \+ belt_cell(C), \+ door(C), \+ memberchk(_-C, Boxes), can_hold(C, W).

% initial_state(-State): the loaded level's start.
initial_state(state(R, none, Boxes)) :-
    robot_start(R),
    findall(Id-P, box_start(Id, P), Bs),
    sort(Bs, Boxes).

% goal_state(+State): robot empty, every box that needs a goal on a goal it accepts.
goal_state(state(_, none, Boxes)) :- forall(( member(Id-P, Boxes), \+ free_box(Id) ), accepts(Id, P)).

% accepts(+Id, +Cell): Cell is a goal box Id may finish on (none for a free box).
accepts(Id, P) :- goal(G, P), target(Id, T), ( T == any ; T == only(G) ).
