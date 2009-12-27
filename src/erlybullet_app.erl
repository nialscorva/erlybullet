%% @author jason
%% @copyright jason Dec 27, 2009  PROPRIETARY-- NOT FOR DISTRIBUTION
%% @doc TODO: Add description to erlybullet_app
%% @end
%% --------------------------------------------------------------------
-module(erlybullet_app).

-behaviour(application).

% Behavioural exports
-export([
	 start/2,
	 stop/1
        ]).

% supervisor exports
-export([init/1,start_link/0]).


%------------------------------------------------------------
%% @spec start(Type, StartArgs)-> {ok, Pid::pid()} | {ok, Pid::pid(), State} | {error, Reason}
%% @private
%% @doc TODO put documentation here
%% @end
%------------------------------------------------------------
start(_Type, _StartArgs) ->
  case erl_ddll:load_driver(code:lib_dir(erlybullet,priv), "erlybullet_drv") of
        ok -> start_link();
        {error, already_loaded} -> start_link();
        E -> {error, E}
  end.

%------------------------------------------------------------
%% @spec stop(State) -> any()
%% @private
%% @doc TODO put documentation here
%% @end
%------------------------------------------------------------
stop(_State) ->
  ok.

%% --------------------------------------------------------------------
%% Supervisor functions
%% --------------------------------------------------------------------
init([]) ->
  {ok,{{one_for_one,3,10}, 
       [
       ]
      }}.

start_link() ->
  supervisor:start_link({local,?MODULE}, ?MODULE, []).
