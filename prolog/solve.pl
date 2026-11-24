% Entry points (plan sections 4 and 5): validate/2, solve/3, macro plan -> step plan.
:- module(solve, [load/1, problem/1, validate/2, solve/3, bridge_validate/2, bridge_solve/3]).
:- use_module(level).
:- use_module(rules).
:- use_module(heuristic).
:- use_module(search).

% load(+Level): assert the level's facts and build its distance tables.
load(L) :- load_level(L), build_dist.

%% Validator --------------------------------------------------------------------------------

% validate(+Level, -Result): ok, or error(Reason) for the first problem found. Cheap checks
% first, then A*: a level is valid exactly when it has a solution.
validate(L, R) :-
    load(L),
    (   once(problem(P))
    ->  R = error(P)
    ;   initial_state(S), search(astar, S, found(_, _, _))
    ->  R = ok
    ;   R = error('This level has no solution')
    ).

% problem(-Reason): a reason the loaded level is malformed. Clause order is check order.
problem('A level needs 1 to 4 boxes') :-
    aggregate_all(count, box_start(_, _), N), \+ between(1, 4, N).
problem(R) :- duplicate(Id, box_start(Id, _)), fmt('Two boxes are named ~w', [Id], R).
problem(R) :- duplicate(Id, goal(Id, _)), fmt('Two goals are named ~w', [Id], R).
problem(R) :- target(B, only(G)), \+ goal(G, _), fmt('Box ~w needs goal ~w, which does not exist', [B, G], R).
problem(R) :-
    target(A, only(G)), target(B, only(G)), A @< B,
    fmt('Boxes ~w and ~w both need goal ~w', [A, B, G], R).
problem(R) :-
    aggregate_all(count, box_start(_, _), NB), aggregate_all(count, goal(_, _), NG), NG < NB,
    fmt('~w boxes but only ~w goals', [NB, NG], R).
problem(R) :-
    tile(C, K1), tile(C, K2), K1 @< K2,
    fmt('Cell ~w is both a ~w and a ~w', [C, K1, K2], R).
problem(R) :- duplicate(C, goal(_, C)), fmt('Cell ~w has two goals', [C], R).
problem(R) :- duplicate(C, belt_cell(C)), fmt('Two belts cross at ~w', [C], R).
problem(R) :-
    belt(_, X, _), blocked(X, K),
    fmt('The belt ending at ~w ends on a ~w', [X, K], R).
problem(R) :-
    fragile(C, _), blocked(C, K),
    fmt('Cell ~w is both fragile and a ~w', [C, K], R).
problem(R) :- duplicate(C, box_start(_, C)), fmt('Two boxes start on ~w', [C], R).
problem(R) :-
    box_start(B, C), blocked(C, K),
    fmt('Box ~w starts on a ~w at ~w', [B, K, C], R).
problem(R) :-
    box_start(B, C), weight(B, W), \+ can_hold(C, W),
    fmt('Box ~w (weight ~w) is too heavy for the fragile tile at ~w', [B, W, C], R).
problem(R) :- robot_start(C), blocked(C, K), fmt('The robot starts on a ~w at ~w', [K, C], R).
problem(R) :- robot_start(C), box_start(B, C), fmt('The robot starts on box ~w at ~w', [B, C], R).
problem('There are doors but no plates to open them') :- door(_), \+ plate(_).
problem('There are plates but no doors for them to open') :- plate(_), \+ door(_).
problem(R) :-
    box_start(B, P), weight(B, W),
    \+ ( goal(_, G), accepts(B, G), dist(W, P, G, _) ),
    fmt('Box ~w cannot reach any goal it may finish on', [B], R).
problem('The boxes cannot all reach different goals') :-
    initial_state(S), \+ h(S, _).

% tile(?Cell, ?Kind): Cell holds a special tile; at most one per cell.
tile(C, wall) :- wall(C).
tile(C, belt) :- belt_cell(C).
tile(C, door) :- door(C).
tile(C, plate) :- plate(C).
tile(C, goal) :- goal(_, C).

% blocked(?Cell, -Kind): no box may start, rest or stand still on Cell.
blocked(C, K) :- tile(C, K), memberchk(K, [wall, belt, door]).

% duplicate(-Key, +Goal): Key occurs in more than one solution of Goal.
duplicate(K, G) :- findall(K, G, Ks), msort(Ks, S), append(_, [K, K|_], S).

% fmt(+Format, +Args, -Atom): format into an atom (atoms reach Python as str), cells as (x, y).
fmt(F, As, R) :- maplist(show, As, Ss), format(atom(R), F, Ss).

% show(+Arg, -Shown): a cell X-Y as '(x, y)', anything else as itself.
show(A, S) :- ( A = X-Y -> format(atom(S), '(~w, ~w)', [X, Y]) ; S = A ).

%% Solver -----------------------------------------------------------------------------------

% solve(+Level, +Algo, -Result): found(Steps, Cost, Nodes), no_solution(Nodes), or
% error(Reason) when the level is malformed (the search assumes a well-formed level).
solve(L, Algo, R) :-
    load(L),
    (   once(problem(P))
    ->  R = error(P)
    ;   initial_state(S),
        search(Algo, S, Res),
        result(S, Res, R)
    ).

% result(+Start, +SearchResult, -Result): macro plan -> step plan.
result(S, found(As, C, N), found(Steps, C, N)) :- steps(S, As, Steps).
result(_, no_solution(N), no_solution(N)).

% steps(+State, +Actions, -Steps): each macro action as its walk, then pick/drop.
steps(_, [], []).
steps(S, [A|As], Steps) :-
    expand(S, A, Ms),
    once(action(S, A, _, S1)),
    steps(S1, As, Rest),
    append(Ms, Rest, Steps).

% expand(+State, +Action, -Steps): the shortest walk the action's cost was computed from.
expand(state(R, none, Bs), fetch(Id, St), Steps) :-
    dijkstra(R, ctx(0, Bs), T), route(T, St, Ms),
    append(Ms, [pick(Id)], Steps).
expand(state(R, Id, Bs), deliver(Id, St, Tg), Steps) :-
    weight(Id, W), dijkstra(R, ctx(W, Bs), T), route(T, St, Ms),
    append(Ms, [drop(Id, Tg)], Steps).

%% Bridge-facing shapes: flat lists of atoms and integers, which PySwip turns into Python lists.

% bridge_validate(+Level, -Out): [ok] or [error, Reason].
bridge_validate(L, Out) :- validate(L, R), flat(R, Out).

% bridge_solve(+Level, +Algo, -Out): [found, Steps, Cost, Nodes], [no_solution], [error, Reason].
bridge_solve(L, Algo, Out) :- solve(L, Algo, R), flat(R, Out).

% flat(+Result, -Out): a result as a flat list.
flat(ok, [ok]).
flat(error(P), [error, P]).
flat(no_solution(_), [no_solution]).
flat(found(Steps, C, N), [found, Fs, C, N]) :- maplist(flat_step, Steps, Fs).

% flat_step(+Step, -Out): u/d/l/r, [ride, X, Y], [pick, Id], [drop, Id, X, Y].
flat_step(M, M) :- atom(M).
flat_step(ride(X-Y, _), [ride, X, Y]).
flat_step(pick(Id), [pick, Id]).
flat_step(drop(Id, X-Y), [drop, Id, X, Y]).
