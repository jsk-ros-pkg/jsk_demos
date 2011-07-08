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

:- use_module(library('comp_spatial')).
:- use_module(library('comp_temporal')).
:- use_module(library('comp_semantic_map')).

:- consult('jsk_map').

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


