%% @author Jason Wagner
%% @copyright Jason Wagner Dec 27, 2009 under the ZLib license
%% @doc Test cases for the interface.  Note that there are no asserts while the simulation is running.  I want
%% to make sure that the simulator gets completely cleaned up between tests.
%% @end
%% --------------------------------------------------------------------
-module(erlybullet_tests).
-include_lib("eunit/include/eunit.hrl").
-compile(export_all).

-define(TIMEOUT,500).

% External exports
-export([]).

% simplist step-- start a simulation then kill it, making sure that it's dead
create_test() ->
  {ok,Pid}=erlybullet:start_link(),
  erlybullet:stop(Pid),
  ?assertNot(is_process_alive(Pid)).

% make sure that we can create an entity in the simulation and receive a location update from it
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
    {EntityId,Props} ->   proplists:get_value(location, Props)
    after ?TIMEOUT -> failed_to_receive
  end,
  erlybullet:stop(World),
  ?assertMatch({_X,_Y,_Z},RV).

% create an entity with an id that we provide to make sure that all the dictionaries are working
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

% make sure that we can create and remove an entity to get no messages
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
  erlybullet:step_simulation(World,1.0),
  RetVal=receive
    {EntityId,_} -> should_not_have_received  
    after ?TIMEOUT -> ok
  end,
  erlybullet:stop(World),
  ?assertMatch(ok, RetVal).

% two entities on a direct course.  numbers are chosen so that we get a collision in the
% first 1/30th of a second that the simulator runs on the first call.
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
                                            {velocity,{100.0,0.0,0.0}}]),

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

%% check that we can apply impulse
entity_impulse_test() ->
  {ok,World}=erlybullet:start_link(),
  {ok,EntityId} = erlybullet:create_entity(World,
                                           self(),
                                           [{shape,{sphere,50.0}},
                                            {location,{0.0,0.0,0.0}},
                                            {mass, 25.0},
                                            {velocity,{100.0,0.0,0.0}}]),
  erlybullet:step_simulation(World),
  RV1=receive
    {EntityId,Props1} ->  proplists:get_value(velocity, Props1)
    after ?TIMEOUT -> failed_to_receive
  end,

  erlybullet:apply_impulse(World,EntityId, {0.0,2600.0,0.0}),
  erlybullet:step_simulation(World,1.0),
  RV2=receive
    {EntityId,Props2} ->  proplists:get_value(velocity, Props2)
    after ?TIMEOUT -> failed_to_receive
  end,
  erlybullet:stop(World),
  {X1,Y1,Z1}=RV1,
  {X2,Y2,Z2}=RV2,
  ?assert(X1 > 99.0),
  ?assert(Y1 < 1),
  ?assert(X2 > 99.0),
  ?assert(Y2 > 99.0),

  ok.

