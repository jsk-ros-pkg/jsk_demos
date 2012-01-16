%%
%% Copyright (c) 2011, Lars Kunze <kunzel@cs.tum.edu>
%% All rights reserved.
%%
%% Redistribution and use in source and binary forms, with or without
%% modification, are permitted provided that the following conditions are met:
%%
%%     * Redistributions of source code must retain the above copyright
%%       notice, this list of conditions and the following disclaimer.
%%     * Redistributions in binary form must reproduce the above copyright
%%       notice, this list of conditions and the following disclaimer in the
%%       documentation and/or other materials provided with the distribution.
%%     * Neither the name of the Intelligent Autonomous Systems Group/
%%       Technische Universitaet Muenchen nor the names of its contributors 
%%       may be used to endorse or promote products derived from this software 
%%       without specific prior written permission.
%%
%% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
%% AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
%% IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
%% ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
%% LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
%% CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
%% SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
%% INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
%% CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
%% ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
%% POSSIBILITY OF SUCH DAMAGE.
%%

:- register_ros_package(comp_spatial).
:- register_ros_package(comp_temporal).
:- register_ros_package(comp_semantic_map).
:- register_ros_package(mod_vis).
:- register_ros_package(knowrob_omics).

:- use_module(library('comp_spatial')).
:- use_module(library('comp_temporal')).
:- use_module(library('comp_semantic_map')).

:- consult('jsk_map').

:- rdf_meta
  hl_path_costs_from(r,r).


obj_room_history(Obj, Room, ObjT, RoomT):-
  findall(_R, (owl_individual_of(_L, knowrob:'OmicsLocations'),
               owl_has(_L, knowrob:object, _R)), 
          _RoomTLst),
  list_to_set(_RoomTLst, _RoomTs), 
  member(RoomT,_RoomTs), 
  owl_individual_of(Room,RoomT), 
  holds(in_ContGeneric(Obj, Room),Time), 
  owl_has(Obj,rdf:type,ObjT), 
  owl_subclass_of(ObjT, knowrob:'Artifact').
  

vis_types:-
  visualisation_canvas(C),
  remove_object('http://www.jsk.t.u-tokyo.ac.jp/jsk_map.owl#eng2-8f-83a3',C),
  remove_object('http://www.jsk.t.u-tokyo.ac.jp/jsk_map.owl#eng2-8f-83a4',C),
  remove_object('http://www.jsk.t.u-tokyo.ac.jp/jsk_map.owl#eng2-8f-83b2',C),
  remove_object('http://www.jsk.t.u-tokyo.ac.jp/jsk_map.owl#eng2-8f-83a1',C),
  remove_object('http://www.jsk.t.u-tokyo.ac.jp/jsk_map.owl#eng2-8f-83a2',C),
  remove_object('http://www.jsk.t.u-tokyo.ac.jp/jsk_map.owl#room83b1-83b1-ground',C),
  remove_object('http://www.jsk.t.u-tokyo.ac.jp/jsk_map.owl#room73a3-73a3-ground',C),
  remove_object('http://www.jsk.t.u-tokyo.ac.jp/jsk_map.owl#room73b2-73b2-ground',C),
  setof(I, (owl_individual_of(I, knowrob:'OfficeRoom'), highlight_object(I,@(true),0,200,200,'1',C)), _),
  setof(I, (owl_individual_of(I, knowrob:'LevelOfAConstruction'), highlight_object(I,@(true),200,200,200,'0.1',C)), _),
  setof(I, (owl_individual_of(I, knowrob:'ProfessorsOffice'),highlight_object(I,@(true),0,200,0,'1',C)), _),
  setof(I, (owl_individual_of(I, knowrob:'Elevator'), highlight_object(I,@(true),250,250,0,'1',C)), _),
  setof(I, (owl_individual_of(I, knowrob:'LectureHall'), highlight_object(I,@(true),250,150,0,'1',C)), _),
  setof(I, (owl_individual_of(I, knowrob:'FastFoodRestaurantSpace'), highlight_object(I,@(true),0,100,0,'1',C)), _),
  setof(I, (owl_individual_of(I, knowrob:'LaboratoryRoom'), highlight_object(I,@(true),0,0,100,'1',C)), _),
  setof(I, (owl_individual_of(I, knowrob:'ControlRoom'), highlight_object(I,@(true),100,0,100,'1',C)), _),
  setof(I, (owl_individual_of(I, knowrob:'LockerRoom'), highlight_object(I,@(true),100,200,100,'1',C)), _),
  setof(I, (owl_has(I,rdf:type,knowrob:'Place'), highlight_object(I,@(true),200,0,100,'1',C)), _).

vis_clear:-
  visualisation_canvas(C), clear_canvas(C), draw_background(C).

vis_cups:-
  visualisation_canvas(C),
  
  setof(I, (owl_individual_of(I, knowrob:'LevelOfAConstruction'), highlight_object(I,@(true),200,200,200,'0.1',C)), _),
  setof(I, (owl_individual_of(I, knowrob:'Chair-PieceOfFurniture'), remove_object(I,C)), _),
  remove_object('http://www.jsk.t.u-tokyo.ac.jp/jsk_map.owl#eng2-2f-22b',C), % because orientation is not taken into account
  remove_object('http://www.jsk.t.u-tokyo.ac.jp/jsk_map.owl#eng2-7f-room73B2-table-front',C),
  remove_object('http://www.jsk.t.u-tokyo.ac.jp/jsk_map.owl#eng2-7f-room73A3-front',C),
  remove_object('http://www.jsk.t.u-tokyo.ac.jp/jsk_map.owl#eng2-8f-83a3',C),
  remove_object('http://www.jsk.t.u-tokyo.ac.jp/jsk_map.owl#eng2-8f-83a4',C),
  remove_object('http://www.jsk.t.u-tokyo.ac.jp/jsk_map.owl#eng2-8f-83b2',C),
  remove_object('http://www.jsk.t.u-tokyo.ac.jp/jsk_map.owl#eng2-8f-83a1',C),
  remove_object('http://www.jsk.t.u-tokyo.ac.jp/jsk_map.owl#eng2-8f-83a2',C),
  remove_object('http://www.jsk.t.u-tokyo.ac.jp/jsk_map.owl#room83b1-83b1-ground',C),
  remove_object('http://www.jsk.t.u-tokyo.ac.jp/jsk_map.owl#room73a3-73a3-ground',C),
  remove_object('http://www.jsk.t.u-tokyo.ac.jp/jsk_map.owl#room73b2-73b2-ground',C),

  remove_object('http://www.jsk.t.u-tokyo.ac.jp/jsk_map.owl#room73b2-kettle',C),
  remove_object('http://www.jsk.t.u-tokyo.ac.jp/jsk_map.owl#room73b2-sponge',C),
  remove_object('http://www.jsk.t.u-tokyo.ac.jp/jsk_map.owl#room73b2-cup',C),
  remove_object('http://www.jsk.t.u-tokyo.ac.jp/jsk_map.owl#room73b2-dish',C),
  remove_object('http://www.jsk.t.u-tokyo.ac.jp/jsk_map.owl#room73b2-knife',C),
  remove_object('http://www.jsk.t.u-tokyo.ac.jp/jsk_map.owl#room73b2-mug-cup',C),
  remove_object('http://www.jsk.t.u-tokyo.ac.jp/jsk_map.owl#room73b2-tray',C).
      

% create_object_instance(knowrob:'Place', cur).
% update_pose(cur, [1.0,0.0,0.0,8.98, 0.0,1.0,0.0,4.8, 0.0,0.0,1.0,27.0, 0.0,0.0,0.0,1.0]).
% visualisation_canvas(C), add_object(cur,C).
% visualisation_canvas(C), highlight_object(cur,@(true), 255, 255, 255, '1.0', C).


%findall(X, (owl_individual_of(I, knowrob:'Cup'),owl_has(X, knowrob:'inFrontOf-Generally',I)), Is), get_sorted_path_costs(cur, Is, Sorted), heatmap(Sorted).


% setof(I, owl_individual_of(I, knowrob:'Cup'), Cs), get_sorted_path_costs('http://ias.cs.tum.edu/kb/knowrob.owl#foo', Cs, SortedGoals), member([CC, CUP], SortedGoals), rdf_has(PLACE, knowrob:'inFrontOf-Generally' ,CUP), highlight_object(CUP,@(true), 250, 0, 0, '1', $C), highlight_object(PLACE,@(true), 250, 0, 0, '1', $C).


r_func(X,Y):-
  A is -0.125,
  B is 0.125,
  C is 0.375,
  D is 0.625,
  trapezoidal_shaped_func(A,B,C,D,X,YVal),
  Y is floor(YVal * 255).

g_func(X,Y):-
  A is 0.125,
  B is 0.375,
  C is 0.625,
  D is 0.875,
  trapezoidal_shaped_func(A,B,C,D,X,YVal),
  Y is floor(YVal * 255).
  

b_func(X,Y):-
  A is 0.375,
  B is 0.625,
  C is 0.875,
  D is 1.125,
  trapezoidal_shaped_func(A,B,C,D,X,YVal),
  Y is floor(YVal * 255).

  % min_list([ ((X - A) / (B - A)) ,1 ,((D - X) / (D - C)) ], Min),
  % Y is floor(max(Min, 0) * 255).

trapezoidal_shaped_func(A,B,C,D,X,Y):-
  min_list([ ((X - A) / (B - A)) ,1 ,((D - X) / (D - C)) ], Min),
  Y is max(Min, 0).

heatmap(Objs):-
  visualisation_canvas(Canvas),
  findall(Cost, member([Cost, _], Objs), Costs),
  reverse(Costs,Rev),
  nth0(0, Rev, Max),
  findall(_,(member([C,Obj], Objs),
             X is C / Max,
             r_func(X,R),
             g_func(X,G),
             b_func(X,B),
             highlight_object(Obj, @(true), R, G, B, '1.0', Canvas)),_).



% findall([Cost,Obj], (lookForKMostSimilarObjTBecause('http://ias.cs.tum.edu/kb/knowrob.owl#RoomInAConstruction', 10,  Obj, [S, T]), Cost is 1.0 - S), Objs), heatmap(Objs).

vis_costs :- hl_path_costs_from(jsk_map:'eng2-7f-73a4',knowrob:'RoomInAConstruction').

hl_path_costs_from(Current, Type):-
  setof(I, owl_individual_of(I,Type), Is),
  get_sorted_path_costs(Current, Is, SortedGoals),
  heatmap(SortedGoals).

 
  

draw_all(C) :-
  draw_rooms(0,250,0,'0.5',C),
  draw_levels(250,0,250,'0.3',C).


draw_rooms(R,G,B,P,C) :-
  findall(Ro, (owl_has(Ro,rdf:type,knowrob:'RoomInAConstruction'),
               add_object(Ro,C),
               highlight_object(Ro,@(true),R,G,B,P,C)),
          _Rs).

draw_levels(R,G,B,P,C) :-
  findall(L, (owl_has(L,rdf:type,knowrob:'LevelOfAConstruction'),
              add_object(L,C),
              highlight_object(L,@(true),R,G,B,P,C)),
          _Ls).

draw_all_n(N,C) :-
  draw_rooms_in_level_n(N,0,250,0,'0.5',C),
  draw_level_n(N,250,0,250,'0.3',C).

draw_level_n(N,R,G,B,P,C) :-
  owl_has(L,rdf:type,knowrob:'LevelOfAConstruction'),
  owl_has(L,knowrob:floorNumber,literal(type(_,N))),
  add_object(L,C),
  highlight_object(L,@(true),R,G,B,P,C).

draw_rooms_in_level_n(N,R,G,B,P,C) :-
  owl_has(L,rdf:type,knowrob:'LevelOfAConstruction'),
  owl_has(L,knowrob:floorNumber,literal(type(_,N))),
  findall(Room, (owl_has(L,knowrob:hasRooms,Room),
                 add_object(Room,C),
                 highlight_object(Room,@(true),R,G,B,P,C)), _Rs).


