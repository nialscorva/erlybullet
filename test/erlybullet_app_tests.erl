%% @author jason
%% @copyright jason Dec 27, 2009  PROPRIETARY-- NOT FOR DISTRIBUTION
%% @doc TODO: Add description to erlybullet_app_tests
%% @end
%% --------------------------------------------------------------------
-module(erlybullet_app_tests).
-include_lib("eunit/include/eunit.hrl").

% External exports
-export([]).
% test cases for the module with setup and teardown
erlybullet_app_tests_test_() ->
  { foreach,
     fun() -> setup() end,  % setup
     fun(SetupRetVal) -> teardown(SetupRetVal) end, % teardown
     [
        ?_assert(lists:any(fun({erlybullet,_,_}) -> true ; (_) -> false end, application:loaded_applications()))
     ]
   }.

setup() ->
  application:start(erlybullet),
  ok.

teardown(_SetupRetval) ->
  application:stop(erlybullet),
  ok.


