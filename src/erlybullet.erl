%% @author Jason Wagner
%% @copyright Jason Wagner Dec 27, 2009 under the ZLib license
%% @doc Interfaces to a bullet simulation.  The intent is that a process generates
%% one or more entities in the simulation and receives messages on their location, velocity,
%% collisions, etc.  See the client_message() type for what gets sent.  Generally the process
%% will get one message per simulation step.
%% @end
%% --------------------------------------------------------------------
-module(erlybullet).

-behaviour(gen_server).

% constants used to communicate to the driver
-include_lib("erlybullet/include/erlybulletcommands.hrl").

% External exports
-export([start_link/0,stop/1,create_entity/3,step_simulation/1,step_simulation/2,destroy_entity/2,apply_impulse/3]).

% gen_server callbacks
-export([init/1, handle_call/3, handle_cast/2, handle_info/2, terminate/2, code_change/3]).

-record(state, {port,next_id=1,id_table}).

%% @type vector3() = {X::float(), Y::float(), Z::float()}.

%% @private @type id_tuple() = {Id::integer(), UserId::term(), Pid::pid()}.

%% @type body_shape() = {sphere, Radius::float()}.

%% @type client_message() = {UserId, [ client_params() ]}.

%% @type client_params() = {location, vector3()} 
%%                       | {velocity, vector3()} 
%%                       | {collision, [{With_Who_Id::term(), With_Who_Pid::pid(), Where::vector3()}]}.

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
%% @doc Asynchronously advances the simulation by how long it's been since last run.
%% @end
% --------------------------------------------------------------------
step_simulation(World) when is_pid(World) ->
  gen_server:cast(World,{step_simulation}).

% --------------------------------------------------------------------
%% @spec step_simulation(World::pid(),TimeStep::float()) -> ok
%% @doc Asynchronously advances the simulation by TimeStep seconds.
%% @end
% --------------------------------------------------------------------
step_simulation(World,TimeStep) when is_pid(World), is_float(TimeStep) ->
  gen_server:cast(World,{step_simulation,TimeStep}).

% --------------------------------------------------------------------
%% @spec create_entity(World,BoundPid::pid, [Options]) -> {ok, EntityId}
%%    Options = {id , UserId}
%%            | {location, vector3()}
%%            | {velocity, vector3()}
%%            | {mass, float()}
%%            | {restitution, float()}
%%            | {shape, body_shape()}
%% @doc Creates a new entity in the world using Options.  Either UserId (if supplied) 
%% or a generated unique identifier will be returned from this function.  
%% <table border="1">
%% <tr><td>Option</td><td>Default</td><td>Notes</td></tr>
%% <tr><td>shape</td><td></td><td>Required field</td></tr>
%% <tr><td>id</td><td></td><td>Generated and returned in the form {erlybullet, Id::integer}.</td></tr>
%% <tr><td>location</td><td>{0.0,0.0,0.0}</td></tr>
%% <tr><td>velocity</td><td>{0.0,0.0,0.0}</td></tr>
%% <tr><td>mass</td><td>1.0</td></tr>
%% <tr><td>restitution</td><td>0.5</td></tr>
%% </table>
%% @end
% --------------------------------------------------------------------
create_entity(World,BoundPid,Options) when is_pid(World),is_pid(BoundPid),is_list(Options) ->
  gen_server:call(World,{create_entity,BoundPid,Options}).

% --------------------------------------------------------------------
%% @spec destroy_entity(World,EntityId) -> {ok, EntityId}
%% @doc Removes an entity from the simulation.
%% @end
% --------------------------------------------------------------------
destroy_entity(World,EntityId) when is_pid(World) ->
  gen_server:call(World,{destroy_entity,EntityId}).

% --------------------------------------------------------------------
%% @spec apply_impulse(World,EntityId,Impulse::vector3()) -> ok
%% @doc Applies the impulse (delta-momentum) at the center of mass
%% on the next simulation step.
%% @end
% --------------------------------------------------------------------
apply_impulse(World,EntityId,Impulse) when is_pid(World) ->
  gen_server:cast(World,{apply_impulse,EntityId,Impulse}).

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
        {error, Any} -> exit({stop, Any, erl_ddll:format_error(Any)});
        E -> exit({stop, E})
  end,
  Port=open_port({spawn, "erlybullet_drv"}, [binary]),
  Table=create_table(),
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
handle_call({destroy_entity,UserId},_From,State) ->
  {Id,UserId,_Pid}=find_by_user_id(State#state.id_table,UserId),
  cast_port(State#state.port,<<?EB_REMOVE_ENTITY:8,Id:64/native-integer>>),
  {reply,ok,State};
handle_call({create_entity,BoundPid,Options},_From,State) ->
  ShapeBin    = shape_to_binary(proplists:get_value(shape,Options)),
  Id          = State#state.next_id, 
  Mass        = proplists:get_value(mass,Options,1.0),
  LocBin      = vector_to_binary(proplists:get_value(location,Options,{0.0,0.0,0.0})),
  VelocityBin = vector_to_binary(proplists:get_value(velocity,Options,{0.0,0.0,0.0})),
  Restitution = proplists:get_value(restitution,Options,0.5),
  UserId      = proplists:get_value(id,Options,{erlybullet,Id}),
  insert_entity(State#state.id_table, {Id,UserId,BoundPid}),
  cast_port(State#state.port,<<?EB_ADD_ENTITY:8,ShapeBin/binary,Id:64/native-integer,Mass/native-float,
                               LocBin/binary,VelocityBin/binary,Restitution/native-float>>),
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
handle_cast({apply_impulse, UserId, Impulse}, #state{port=Port}=State) ->
  {Id,UserId,_Pid}=find_by_user_id(State#state.id_table,UserId),
  ImpulseVec = vector_to_binary(Impulse),
  cast_port(Port,<<?EB_APPLY_IMPULSE:8/integer,Id:64/native-integer,ImpulseVec/binary>>),
  {noreply,State};
handle_cast({step_simulation, TimeStep}, #state{port=Port}=State) ->
  call_port(Port,<<?EB_STEP_SIMULATION:8/integer,TimeStep/native-float>>),
  {noreply, State};
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
handle_info({erlybullet,Id,Rest},#state{id_table=Table}=State) ->
  {Id,UserId,Pid}=find_by_id(Table,Id),
  Params=lists:map(fun(A) -> translate(A,Table) end, Rest),
  Pid ! {UserId,Params},
  {noreply,State};
handle_info(Info, State) ->
  io:format("Received ~p~n",[Info]),
  {noreply, State}.

% --------------------------------------------------------------------
%% @spec translate(E::term(),Table::table) -> E_prime::term()
%% @private
%% @doc The driver returns a proplist.  This function converts the peices that don't make sense
%% on their own, such as converting Ids to UserIds and/or Pids.
%% @end
% --------------------------------------------------------------------
translate({collisions,Collisions},Table) ->
  {collisions,lists:map(fun({Id,Where}) -> {Id, UserId, Pid} = find_by_id(Table,Id), 
                                           {UserId, Pid, Where} 
                        end, Collisions)
  };
translate(E, _Table) ->
  E.


% --------------------------------------------------------------------
%% @spec terminate(Reason::term(),State::state()) -> any()
%% @private
%% @doc Shutdown the server.  Close the port and clean up the tables
%% @end
% --------------------------------------------------------------------
terminate(_Reason, #state{port=Port,id_table=Table}) ->
  port_close(Port),
  free_table(Table),
  ok.

% --------------------------------------------------------------------
%% @spec code_change(OldVsn,State,Extra) -> {ok, NewState}
%% @private
%% @doc Convert process state when code is changed
%% @end
% --------------------------------------------------------------------
code_change(_OldVsn, State, _Extra) ->
    {ok, State}.

% --------------------------------------------------------------------
%% @spec call_port(Port::port(), Command::binary()) -> Data::term()
%% @private
%% @doc Sends a binary to the port and waits for a reply.
%% @end
% --------------------------------------------------------------------
call_port(Port, Command) when is_port(Port) ->
  port_command(Port, Command),
  receive
    {port,Data} -> Data
    after 100 -> {error, timeout}
  end.

% --------------------------------------------------------------------
%% @spec cast_port(Port, Command) -> ok
%% @private
%% @doc Sends a binary to the port without waiting for reply.
%% @end
% --------------------------------------------------------------------
cast_port(Port, Command) when is_port(Port) ->
  port_command(Port,Command).

% --------------------------------------------------------------------
%% @spec shape_to_binary(ShapeInfo) -> ShapeBin::binary()
%%     ShapeInfo = {sphere,Radius::float()}
%% @private
%% @doc Encodes a shape to binary.
%% @end
% --------------------------------------------------------------------
shape_to_binary({sphere,Radius}) -> <<?EB_SPHERE_SHAPE,Radius/native-float>>;

shape_to_binary(Shape) -> throw({unknown_shape,Shape}).

% --------------------------------------------------------------------
%% @spec vector_to_binary(vector3()) -> binary()
%% @private
%% @doc Turns a 3-tuple into a binary.
%% @end
% --------------------------------------------------------------------
vector_to_binary({X,Y,Z}) -> <<X/native-float,Y/native-float,Z/native-float>>.

% --------------------------------------------------------------------
%% @spec create_table() -> term()
%% @private
%% @doc Sets up ad-hoc process-specific storage.  Uses ets right now.
%% @end
% --------------------------------------------------------------------
create_table() ->
  IdTable=ets:new(entity_table,[set,private]),
  UserIdTable=ets:new(id_table,[set,private]),
  {IdTable,UserIdTable}.

% --------------------------------------------------------------------
%% @spec free_table(Table) -> ok
%% @private
%% @doc Frees the table information.
%% @end
% --------------------------------------------------------------------
free_table({IdTable,UserIdTable}) ->
  ets:delete(IdTable),
  ets:delete(UserIdTable).

% --------------------------------------------------------------------
%% @spec insert_entity(Table, Record::id_tuple()) -> ok
%% @private
%% @doc Inserts a new entity into the mapping.  Old entities are simply overwritten with no notice.
%% @end
% --------------------------------------------------------------------
insert_entity({IdTable,UserIdTable},{_,U,_}=Record) ->
  ets:insert(IdTable, Record),
  ets:insert(UserIdTable, {U,Record}).

% --------------------------------------------------------------------
%% @spec find_by_user_id(Table,UserId::term()) -> id_tuple()
%% @private
%% @doc Finds a record by the Id used by the Erlang side.
%% @end
% --------------------------------------------------------------------
find_by_user_id({_,Table},UserId) ->
  [{UserId,Rec}]=ets:lookup(Table,UserId),
  Rec.

% --------------------------------------------------------------------
%% @spec find_by_id(Table,Id::term()) -> id_tuple()
%% @private
%% @doc Finds a record by the Id used by the driver side.
%% @end
% --------------------------------------------------------------------
find_by_id({Table,_},Id) ->
  [Rec]=ets:lookup(Table,Id),
  Rec.
