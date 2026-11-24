% Benchmark for the report: cost, expanded nodes and time of UCS, A* and greedy on a fixed
% level set. Run: swipl -g main -t halt bench/bench.pl
% Times are wall-clock and vary between runs; costs and node counts are deterministic.
:- use_module('../prolog/solve').
:- use_module('../prolog/rules').
:- use_module('../prolog/search').
:- use_module(library(time)).

% limit(-Seconds): give up on one run after this long (UCS cannot do 3+ boxes).
limit(120).

% room(+MaxX, +MaxY, -Walls): walls filling every inner cell outside 1..MaxX x 1..MaxY.
room(MX, MY, Ws) :-
    findall(X-Y, (between(1, 8, X), between(1, 8, Y), (X > MX ; Y > MY)), Ws).

% shelves(-Walls): two rows of shelving with aisles between them.
shelves(Ws) :- findall(X-Y, (member(Y, [3, 6]), (between(2, 4, X) ; between(6, 7, X))), Ws).

% level(?Name, -Level): the benchmark set, small to large.
level(fragile, level(Ws, [goal(g1, 5-2)], [box(a, 1-2, 2, any)], 1-1,
                     [fragile(3-1, 1), fragile(3-2, 1)], [], [], [])) :- room(5, 3, Ws).
level(belt, level(Ws, [goal(g1, 6-2)], [box(a, 1-3, 2, any)], 1-1, [], [belt(2-2, 5-2)], [], [])) :-
    room(6, 3, Ws).
level(blocker, level(Ws, [goal(g1, 5-1), goal(g2, 4-4)],
                     [box(a, 3-4, 1, only(g1)), box(b, 1-1, 1, only(g2))], 1-3, [], [], [], [])) :-
    room(5, 5, Room), findall(X-2, between(2, 5, X), C), append(Room, C, Ws).
level(doorstop, level(Ws, [goal(g1, 7-4), goal(g2, 1-1)],
                      [box(a, 2-3, 1, only(g1)), box(b, 1-3, 1, only(g2))], 3-4, [], [], [2-1], [4-2])) :-
    room(7, 4, Room), findall(4-Y, (between(1, 4, Y), Y =\= 2), C), append(Room, C, Ws).
level(two_pinned, level(Ws, [goal(g1, 1-5), goal(g2, 5-5)],
                        [box(a, 5-2, 1, only(g1)), box(b, 1-2, 2, only(g2))], 3-1, [], [], [], [])) :-
    room(5, 5, Ws).
level(open2, level([], [goal(g1, 2-6), goal(g2, 6-6)],
                   [box(a, 6-3, 1, only(g1)), box(b, 2-3, 2, only(g2))], 4-1, [], [], [], [])).
level(shelves3, level(Ws, [goal(g1, 8-1), goal(g2, 8-4), goal(g3, 8-7)],
                      [box(a, 3-2, 1, any), box(b, 5-5, 2, any), box(c, 2-7, 3, any)], 1-1, [], [], [], [])) :-
    shelves(Ws).
level(open3, level([], [goal(g1, 8-8), goal(g2, 1-8), goal(g3, 8-1)],
                   [box(a, 2-2, 1, any), box(b, 7-2, 2, any), box(c, 2-7, 3, any)], 1-1, [], [], [], [])).
level(shelves4, level(Ws, [goal(g1, 8-1), goal(g2, 8-4), goal(g3, 8-7), goal(g4, 8-8)],
                      [box(a, 3-2, 1, any), box(b, 5-5, 2, only(g1)), box(c, 2-7, 3, any),
                       box(d, 6-8, 1, only(g4))], 1-1, [], [], [], [])) :-
    shelves(Ws).
level(open4, level([], [goal(g1, 8-8), goal(g2, 1-8), goal(g3, 8-1), goal(g4, 5-5)],
                   [box(a, 2-2, 1, any), box(b, 7-2, 2, any), box(c, 2-7, 3, any), box(d, 4-4, 1, any)],
                   1-1, [], [], [], [])).

% main: one table row per level and algorithm.
main :-
    format("~w~t~12|~w~t~19|~w~t~27|~w~t~42|~w~t~51|~w~n", [level, boxes, algo, cost, nodes, seconds]),
    forall(level(Name, L), forall(member(A, [ucs, astar, greedy]), run(Name, L, A))).

% run(+Name, +Level, +Algo): solve once and print a row; a timeout or full stack is reported.
run(Name, L, Algo) :-
    L = level(_, _, Bs, _, _, _, _, _), length(Bs, NB),
    load(L), initial_state(S), limit(Lim),
    get_time(T0),
    catch(call_with_time_limit(Lim, search(Algo, S, R)), E, true),
    get_time(T1), T is T1 - T0,
    (   var(E), R = found(_, C, N) -> true
    ;   var(E), R = no_solution(N) -> C = none
    ;   E == time_limit_exceeded -> C = timeout, N = '-'
    ;   E = error(resource_error(_), _) -> C = out_of_memory, N = '-'
    ;   throw(E)
    ),
    format("~w~t~12|~w~t~19|~w~t~27|~w~t~42|~w~t~51|~2f~n", [Name, NB, Algo, C, N, T]).
