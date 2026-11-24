% Level facts asserted by load_level/1 from one level/8 term (plan section 6).
% Cells are X-Y. The 10x10 border is always wall, whatever the term says.
:- module(level, [load_level/1, wall/1, goal/2, box_start/2, weight/2, target/2,
                  fragile/2, belt/3, belt_cell/1, plate/1, door/1, robot_start/1]).
:- dynamic wall/1, goal/2, box_start/2, weight/2, target/2,
           fragile/2, belt/3, belt_cell/1, plate/1, door/1, robot_start/1.

% load_level(+Level): replace the current level facts with those of Level.
load_level(level(Walls, Goals, Boxes, Robot, Fragile, Belts, Plates, Doors)) :-
    clear_level,
    findall(C, border(C), Border),
    append(Border, Walls, All),
    sort(All, Ws),
    forall(member(W, Ws), assertz(wall(W))),
    forall(member(goal(G, P), Goals), assertz(goal(G, P))),
    forall(member(B, Boxes), add_box(B)),
    assertz(robot_start(Robot)),
    forall(member(fragile(P, M), Fragile), assertz(fragile(P, M))),
    forall(member(B, Belts), add_belt(B)),
    forall(member(P, Plates), assertz(plate(P))),
    forall(member(P, Doors), assertz(door(P))).

% clear_level: retract every level fact.
clear_level :-
    forall(member(N/A, [wall/1, goal/2, box_start/2, weight/2, target/2, fragile/2,
                        belt/3, belt_cell/1, plate/1, door/1, robot_start/1]),
           ( functor(H, N, A), retractall(H) )).

% border(-Cell): Cell is on the edge of the 10x10 board.
border(X-Y) :- between(0, 9, X), between(0, 9, Y), ( X =:= 0 ; X =:= 9 ; Y =:= 0 ; Y =:= 9 ).

% add_box(+Box): assert start, weight and target (any | only(G)) of one box.
add_box(box(Id, P, W, T)) :-
    assertz(box_start(Id, P)), assertz(weight(Id, W)), assertz(target(Id, T)).

% add_belt(+Belt): assert belt(Entry, Exit, Length) and its belt cells.
% Length = steps a walk would take from the cell before Entry to Exit.
add_belt(belt(E, X)) :-
    manhattan(E, X, D),
    L is D + 1,
    assertz(belt(E, X, L)),
    forall(on_belt(E, X, C), assertz(belt_cell(C))).

% on_belt(+Entry, +Exit, -Cell): Cell runs from Entry up to, not including, Exit.
on_belt(Ex-Ey, Xx-Xy, X-Y) :-
    Dx is sign(Xx - Ex), Dy is sign(Xy - Ey),
    manhattan(Ex-Ey, Xx-Xy, D),
    N is D - 1,
    between(0, N, K),
    X is Ex + K * Dx, Y is Ey + K * Dy.

% manhattan(+A, +B, -D): grid distance between cells A and B.
manhattan(Ax-Ay, Bx-By, D) :- D is abs(Ax - Bx) + abs(Ay - By).
