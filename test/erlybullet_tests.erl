%% @author jason
%% @copyright jason Dec 27, 2009  PROPRIETARY-- NOT FOR DISTRIBUTION
%% @doc TODO: Add description to erlybullet_tests
%% @end
%% --------------------------------------------------------------------
-module(erlybullet_tests).
-include_lib("eunit/include/eunit.hrl").
-compile(export_all).

% External exports
-export([]).
% test cases for the module with setup and teardown
erlybullet_tests_test_() ->
  { foreach,
     fun() -> setup() end,  % setup
     fun(SetupRetVal) -> teardown(SetupRetVal) end, % teardown
     [
        ?_test(test_create()),
        ?_test(test_entity_create())
     ]
   }.

setup() -> ok.

teardown(_SetupRetval) -> ok.

test_create() ->
  {ok,Pid}=erlybullet:start_link(),
  erlybullet:stop(Pid),
  ?assertNot(is_process_alive(Pid)).

test_entity_create() ->
  {ok,World}=erlybullet:start_link(),
  {ok,EntityId} = erlybullet:create_entity(World,
                                           self(),
                                           [{shape,{sphere,50.0}},
                                            {location,{0.0,0.0,0.0}},
                                            {mass, 25.0},
                                            {velocity,{100.0,0.0,100.0}}]),
  erlybullet:step_simulation(World),
  RetVal=receive
    {erlybullet,EntityId,{location,{_X,_Y,_Z}}} -> ok 
    after 2000 -> ?assert(failed_to_receive)
  end,
  ?assertMatch(ok, RetVal),
  erlybullet:stop(World).