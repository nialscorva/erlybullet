%% @author jason
%% @copyright jason Dec 27, 2009  PROPRIETARY-- NOT FOR DISTRIBUTION
%% @doc TODO: Add description to erlybullet
%% @end
%% --------------------------------------------------------------------
-module(erlybullet).

-behaviour(gen_server).

% External exports
-export([start_link/0,stop/1]).

% gen_server callbacks
-export([init/1, handle_call/3, handle_cast/2, handle_info/2, terminate/2, code_change/3]).

-record(state, {port}).

% --------------------------------------------------------------------
%% @spec start_link() -> {ok, Pid}
%% @doc Starts the bullet node.
%% @end
% --------------------------------------------------------------------
start_link() ->
  gen_server:start_link(?MODULE,[],[]).

% --------------------------------------------------------------------
%% @spec stop(Pid) -> {ok, Pid}
%% @doc Stops the bullet engine and frees the resources.
%% @end
% --------------------------------------------------------------------
stop(Pid) when is_pid(Pid) ->
  gen_server:call(Pid,{stop}).

% --------------------------------------------------------------------
%% @spec init([]) ->
%%          {ok, State}          |
%%          {ok, State, Timeout} |
%%          ignore               |
%%          {stop, Reason}
%% @private
%% @doc Initiates the server
%% @end 
% --------------------------------------------------------------------
init([]) ->
  case erl_ddll:load_driver(code:lib_dir(erlybullet,priv), "erlybullet_drv") of
        ok -> ok;
        {error, already_loaded} -> ok;
        E -> exit({stop, E})
  end,
  Port=open_port({spawn, "erlybullet_drv"}, [binary]),
  {ok, #state{port=Port}}.

% --------------------------------------------------------------------
%% @spec handle_call(Request::term(), From::pid(), State::state()) ->
%%          {reply, Reply, State}          |
%%          {reply, Reply, State, Timeout} |
%%          {noreply, State}               |
%%          {noreply, State, Timeout}      |
%%          {stop, Reason, Reply, State}   |
%%          {stop, Reason, State}
%% @private
%% @doc Handling call messages
%% @end
% --------------------------------------------------------------------
handle_call({stop}, _From, State) ->
  {stop,normal,ok,State};
handle_call(_Request, _From, State) ->
    Reply = ok,
    {reply, Reply, State}.

% --------------------------------------------------------------------
%% @spec handle_cast(Msg::term(), State::state()) ->
%%          {noreply, State}          |
%%          {noreply, State, Timeout} |
%%          {stop, Reason, State}
%% @private
%% @doc Handling cast messages
%% @end
% --------------------------------------------------------------------
handle_cast(_Msg, State) ->
    {noreply, State}.

% --------------------------------------------------------------------
%% @spec handle_info(Info::term(), State::state()) ->
%%          {noreply, State}          |
%%          {noreply, State, Timeout} |
%%          {stop, Reason, State}
%% @private
%% @doc Handling all non call/cast messages
%% @end
% --------------------------------------------------------------------
handle_info(_Info, State) ->
    {noreply, State}.

% --------------------------------------------------------------------
%% @spec terminate/2(Reason::term(),State::state()) -> any()
%% @private
%% @doc Shutdown the server
%% @end
% --------------------------------------------------------------------
terminate(_Reason, #state{port=Port}) ->
  port_close(Port),
  ok.

% --------------------------------------------------------------------
%% @spec code_change(OldVsn,State,Extra) -> {ok, NewState}
%% @private
%% @doc Convert process state when code is changed
%% @end
% --------------------------------------------------------------------
code_change(_OldVsn, State, _Extra) ->
    {ok, State}.

