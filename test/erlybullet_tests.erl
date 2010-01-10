%% @author jason
%% @copyright jason Dec 27, 2009  PROPRIETARY-- NOT FOR DISTRIBUTION
%% @doc TODO: Add description to erlybullet_tests
%% @end
%% --------------------------------------------------------------------
-module(erlybullet_tests).
-include_lib("eunit/include/eunit.hrl").
-compile(export_all).

-define(TIMEOUT,500).

% External exports
-export([]).
% test cases for the module with setup and teardown
%% erlybullet_tests_test_() ->
%%   { foreach,
%%      fun() -> setup() end,  % setup
%%      fun(SetupRetVal) -> teardown(SetupRetVal) end, % teardown
%%      [
%%         ?_test(test_create()),
%%         ?_test(test_entity_create()),
%%         ?_test(test_entity_custom_id()),
%%         ?_test(test_entity_removal()),
%%         ?_test(test_entity_collision())
%%      ]
%%    }.
%% 
%% setup() -> ok.
%% 
%% teardown(_SetupRetval) -> ok.

create_test() ->
  {ok,Pid}=erlybullet:start_link(),
  erlybullet:stop(Pid),
  ?assertNot(is_process_alive(Pid)).

entity_create_test() ->
  {ok,World}=erlybullet:start_link(),
  {ok,EntityId} = erlybullet:create_entity(World,
                                           self(),
                                           [{shape,{sphere,50.0}},
                                            {location,{0.0,0.0,0.0}},
                                            {mass, 25.0},
                                            {velocity,{100.0,0.0,100.0}}]),
  erlybullet:step_simulation(World),
  RV=receive
    {EntityId,Props} ->  ?debugFmt("Props are ~p~n",[Props]), proplists:get_value(location, Props);
     Props -> ?debugFmt("Props are ~p~n",[Props])
    after ?TIMEOUT -> failed_to_receive
  end,
  erlybullet:stop(World),
  ?assertMatch({_X,_Y,_Z},RV).

entity_custom_id_test() ->
  EntityId=test_id,
  {ok,World}=erlybullet:start_link(),
  {ok,EntityId} = erlybullet:create_entity(World,
                                           self(),
                                           [{id,EntityId},
                                            {shape,{sphere,50.0}},
                                            {location,{0.0,0.0,0.0}},
                                            {mass, 25.0},
                                            {velocity,{100.0,0.0,100.0}}]),
  erlybullet:step_simulation(World),
  RV=receive
    {EntityId,Props} -> proplists:get_value(location, Props)
    after ?TIMEOUT -> failed_to_receive
  end,
  erlybullet:stop(World),
  ?assertMatch({_X,_Y,_Z},RV).
  
entity_removal_test() ->
  EntityId=test_id,
  {ok,World}=erlybullet:start_link(),
  {ok,EntityId} = erlybullet:create_entity(World,
                                           self(),
                                           [{id,EntityId},
                                            {shape,{sphere,50.0}},
                                            {location,{0.0,0.0,0.0}},
                                            {mass, 25.0},
                                            {velocity,{100.0,0.0,100.0}}]),
  erlybullet:destroy_entity(World,EntityId),
  erlybullet:step_simulation(World),
  RetVal=receive
    {EntityId,_} -> should_not_have_received  
    after ?TIMEOUT -> ok
  end,
  ?assertMatch(ok, RetVal),
  erlybullet:stop(World),
  ?assertMatch(ok, RetVal).

entity_collision_test() ->
  {ok,World}=erlybullet:start_link(),
  {ok,Entity1} = erlybullet:create_entity(World,
                                           self(),
                                           [{shape,{sphere,20.0}},
                                            {location,{20.0,0.0,0.0}},
                                            {mass, 25.0},
                                            {restitution,0.5},
                                            {velocity,{-100.0,0.0,0.0}}]),
  {ok,Entity2} = erlybullet:create_entity(World,
                                           self(),
                                           [{shape,{sphere,20.0}},
                                            {location,{-20.0,0.0,0.0}},
                                            {mass, 25.0},
                                            {restitution,0.5},
                                            {velocity,{110.0,0.0,0.0}}]),

  erlybullet:step_simulation(World),
  receive
    {Entity1,Entity1Status} -> ok
    after ?TIMEOUT -> Entity1Status=no_status
  end,
  receive
    {Entity2,Entity2Status} -> ok
    after ?TIMEOUT -> Entity2Status=no_status
  end,
  ?assertMatch({_X,_Y,_Z},proplists:get_value(location,Entity1Status)),
  ?assertMatch({_X,_Y,_Z},proplists:get_value(location,Entity2Status)),
  
  Self=self(),
  ?assertMatch([{Entity2,Self,{_X,_Y,_Z}}],proplists:get_value(collisions,Entity1Status)),
  ?assertMatch([{Entity1,Self,{_X,_Y,_Z}}],proplists:get_value(collisions,Entity2Status)),
  
  erlybullet:stop(World).