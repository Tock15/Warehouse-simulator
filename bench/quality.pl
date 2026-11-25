% Heuristic quality for the report, on the bench.pl level set. Run: swipl bench/quality.pl
% Per level, from the A* (optimal) plan:
%   h0/cost   share of the true cost that h sees at the start (1.00 = perfect)
%   mean      the same ratio averaged over every state on the plan (goal state left out)
%   free      cost of the plan's actions that move free boxes (the part h cannot see)
:- consult(bench).
:- use_module('../prolog/heuristic').
:- use_module('../prolog/level').
:- initialization(quality, main).

quality :-
    format("~w~t~12|~w~t~19|~w~t~26|~w~t~32|~w~t~41|~w~t~48|~w~t~54|~w~t~64|~w~n",
           [level, boxes, cost, h0, 'h0/cost', mean, free, 'A* nodes', seconds]),
    forall(level(Name, L), row(Name, L)).

row(Name, L) :-
    L = level(_, _, Bs, _, _, _, _, _), length(Bs, NB),
    load(L), initial_state(S0), limit(Lim),
    get_time(T0),
    catch(call_with_time_limit(Lim, search(astar, S0, R)), E, true),
    get_time(T1), T is T1 - T0,
    (   var(E), R = found(As, Cost, Nodes)
    ->  h(S0, H0), R0 is H0 / Cost,
        trail(As, S0, 0, States, Steps),
        findall(X, ( member(S-G, States), Rem is Cost - G, Rem > 0, h(S, H), X is H / Rem ), Xs),
        sum_list(Xs, Sum), length(Xs, N), Mean is Sum / N,
        findall(C, ( member(A-C, Steps), arg(1, A, Id), free_box(Id) ), Cs), sum_list(Cs, Free),
        format("~w~t~12|~w~t~19|~w~t~26|~w~t~32|~2f~t~41|~2f~t~48|~w~t~54|~w~t~64|~2f~n",
               [Name, NB, Cost, H0, R0, Mean, Free, Nodes, T])
    ;   format("~w~t~12|~w~t~19|A* gave up: ~w~n", [Name, NB, E])
    ).

% trail(+Actions, +State, +G, -States, -Steps): the plan's states as State-G, its actions as Action-Cost.
trail([], S, G, [S-G], []).
trail([A|As], S, G, [S-G|Ss], [A-C|Cs]) :- once(action(S, A, C, S1)), G1 is G + C, trail(As, S1, G1, Ss, Cs).
