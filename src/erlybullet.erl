%% @author jason
%% @copyright jason Dec 27, 2009  PROPRIETARY-- NOT FOR DISTRIBUTION
%% @doc TODO: Add description to erlybullet
%% @end
%% --------------------------------------------------------------------
-module(erlybullet).

-behaviour(gen_server).

-include_lib("erlybullet/include/erlybulletcommands.hrl").

% External exports
-export([start_link/0,stop/1,create_entity/3,step_simulation/1]).

% gen_server callbacks
-export([init/1, handle_call/3, handle_cast/2, handle_info/2, terminate/2, code_change/3]).

-record(state, {port,next_id=1,id_table}).

% --------------------------------------------------------------------
%% @spec start_link() -> {ok, World::pid()}
%% @doc Starts the bullet node.
%% @end
% --------------------------------------------------------------------
start_link() ->
  gen_server:start_link(?MODULE,[],[]).

% --------------------------------------------------------------------
%% @spec stop(World::pid()) -> ok
%% @doc Stops the bullet engine and frees the resources.
%% @end
% --------------------------------------------------------------------
stop(World) when is_pid(World) ->
  gen_server:call(World,{stop}).

% --------------------------------------------------------------------
%% @spec step_simulation(World::pid()) -> ok
%% @doc Asynchronously advances the simulation by a time unit.
%% @end
% --------------------------------------------------------------------
step_simulation(World) when is_pid(World) ->
  gen_server:cast(World,{step_simulation}).

% --------------------------------------------------------------------
%% @spec create_entity(World,BoundPid::pid,Options) -> {ok, EntityId}
%% @doc Creates a new entity in the world using Options.
%% @end
% --------------------------------------------------------------------
create_entity(World,BoundPid,Options) when is_pid(World),is_pid(BoundPid),is_list(Options) ->
  gen_server:call(World,{create_entity,BoundPid,Options}).

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
  Table=ets:new(id_table,[set,private]),
  {ok, #state{port=Port,id_table=Table}}.

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
handle_call({create_entity,BoundPid,Options},_From,State) ->
  ShapeBin    = shape_to_binary(proplists:get_value(shape,Options)),
  Id          = State#state.next_id, 
  Mass        = proplists:get_value(mass,Options,1.0),
  LocBin      = vector_to_binary(proplists:get_value(location,Options,{0.0,0.0,0.0})),
  VelocityBin = vector_to_binary(proplists:get_value(velocity,Options,{0.0,0.0,0.0})),
  cast_port(State#state.port,<<?EB_ADD_SHAPE:8,ShapeBin/binary,Id:64/native-integer,Mass/native-float,LocBin/binary,VelocityBin/binary>>),
  case proplists:get_value(id,Options,undefined) of
    undefined -> ets:insert(State#state.id_table, {Id,BoundPid}), 
                 UserId=Id;
    UserId    -> ets:insert(State#state.id_table, {Id,{UserId,BoundPid}})
  end,
  {reply,{ok,UserId},State#state{next_id=Id+1}}.


% --------------------------------------------------------------------
%% @spec handle_cast(Msg::term(), State::state()) ->
%%          {noreply, State}          |
%%          {noreply, State, Timeout} |
%%          {stop, Reason, State}
%% @private
%% @doc Handling cast messages
%% @end
% --------------------------------------------------------------------
handle_cast({step_simulation}, #state{port=Port}=State) ->
  call_port(Port,<<?EB_STEP_SIMULATION:8/integer>>),
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
handle_info({erlybullet,Id,Rest}=Message,#state{id_table=Table}=State) ->
  case ets:lookup(Table,Id) of
    [{Id,{UserId,Pid}}] -> Pid ! {erlybullet,UserId,Rest}; 
    [{Id,Pid}] -> Pid ! Message
  end,
  {noreply,State};
handle_info(Info, State) ->
  io:format("Received ~p~n",[Info]),
  {noreply, State}.

% --------------------------------------------------------------------
%% @spec terminate/2(Reason::term(),State::state()) -> any()
%% @private
%% @doc Shutdown the server
%% @end
% --------------------------------------------------------------------
terminate(_Reason, #state{port=Port,id_table=Table}) ->
  port_close(Port),
  ets:delete(Table),
  ok.

% --------------------------------------------------------------------
%% @spec code_change(OldVsn,State,Extra) -> {ok, NewState}
%% @private
%% @doc Convert process state when code is changed
%% @end
% --------------------------------------------------------------------
code_change(_OldVsn, State, _Extra) ->
    {ok, State}.


call_port(Port, Command) when is_port(Port) ->
  port_command(Port, Command),
  receive
    {port,Data} -> Data
    after 100 -> {error, timeout}
  end.

cast_port(Port, Command) when is_port(Port) ->
  port_command(Port,Command).

shape_to_binary({sphere,Radius}) -> <<?EB_SPHERE_SHAPE,Radius/native-float>>.

vector_to_binary({X,Y,Z}) -> <<X/native-float,Y/native-float,Z/native-float>>.
