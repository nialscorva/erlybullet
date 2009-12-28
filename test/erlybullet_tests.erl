%% @author jason
%% @copyright jason Dec 27, 2009  PROPRIETARY-- NOT FOR DISTRIBUTION
%% @doc TODO: Add description to erlybullet_tests
%% @end
%% --------------------------------------------------------------------
-module(erlybullet_tests).
-include_lib("eunit/include/eunit.hrl").

% External exports
-export([]).
% test cases for the module with setup and teardown
erlybullet_tests_test_() ->
  { foreach,
     fun() -> setup() end,  % setup
     fun(SetupRetVal) -> teardown(SetupRetVal) end, % teardown
     [
        ?_test(test_create())
     ]
   }.

setup() -> ok.

teardown(_SetupRetval) -> ok.

test_create() ->
  {ok,Pid}=erlybullet:start_link(),
  erlybullet:stop(Pid),
  ?assertNot(is_process_alive(Pid)).

