% Best-first loop shared by ucs/greedy/astar (plan section 3); only priority/4 differs.
% Best-g table instead of a closed set: a state is expanded again whenever it is reached
% with a strictly smaller g, so A* stays optimal with an admissible, possibly inconsistent h.
:- module(search, [search/3]).
:- use_module(rules).
:- use_module(library(heaps)).
:- use_module(library(assoc)).

% search(+Algo, +Start, -Result): Result = found(Actions, Cost, Nodes) | no_solution(Nodes).
% Nodes = number of expanded states.
search(Algo, S0, Result) :-
    estimate(Algo, S0, H),
    priority(Algo, 0, H, P),
    singleton_heap(F, P, node(S0, 0, [])),
    list_to_assoc([S0-0], Best),
    loop(Algo, q(F, Best, 0), Result).

% priority(+Algo, +G, +H, -P): frontier order. Smallest P is expanded first.
priority(ucs, G, _H, G).

% estimate(+Algo, +State, -H): heuristic value; UCS never computes one.
estimate(ucs, _, 0).

% loop(+Algo, +q(Frontier, Best, Nodes), -Result): pop, skip if stale, stop at a goal, else expand.
loop(_, q(F, _, N), no_solution(N)) :- empty_heap(F).
loop(Algo, q(F0, Best, N), Result) :-
    get_from_heap(F0, _, node(S, G, Path), F1),
    (   get_assoc(S, Best, G0), G0 < G           % a cheaper copy was queued later
    ->  loop(Algo, q(F1, Best, N), Result)
    ;   goal_state(S)
    ->  reverse(Path, Actions), Result = found(Actions, G, N)
    ;   N1 is N + 1,
        findall(A-C-S1, action(S, A, C, S1), Children),
        foldl(relax(from(Algo, G, Path)), Children, F1-Best, F2-Best2),
        loop(Algo, q(F2, Best2, N1), Result)
    ).

% relax(+from(Algo, G, Path), +Action-Cost-Child, +F0-B0, -F-B): queue Child unless it was
% already reached with a g that is <= the new one.
relax(from(Algo, G, Path), A-C-S, F0-B0, F-B) :-
    G1 is G + C,
    (   get_assoc(S, B0, Old), Old =< G1
    ->  F = F0, B = B0
    ;   put_assoc(S, B0, G1, B),
        estimate(Algo, S, H),
        priority(Algo, G1, H, P),
        add_to_heap(F0, P, node(S, G1, [A|Path]), F)
    ).
