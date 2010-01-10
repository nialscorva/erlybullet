#ifndef ERLYBULLETMOTIONSTATE_H_INCLUDED__
#define ERLYBULLETMOTIONSTATE_H_INCLUDED__

#include<LinearMath/btMotionState.h>
#include<LinearMath/btTransform.h>


namespace erlybullet {

/// Custom motion_state for the interface.  Tracks the identity, position, and collisions that
/// occur over a step of the simulation.
class motion_state : public btMotionState {
public:
	motion_state(const btVector3 &initial_pos, uint64_t entity_id) :dirty(false), id(entity_id) {
  	mPos1.setIdentity();
  	mPos1.setOrigin(initial_pos);
  }
  virtual ~motion_state() {}

  /// btMotionState override
  virtual void getWorldTransform(btTransform &worldTrans) const {worldTrans = mPos1;}

  /// btMotionState override
  virtual void setWorldTransform(const btTransform &worldTrans) {
  	dirty=true;
  	mPos1=worldTrans;
  }

  /// Used to store the position of collisions.  We don't use a btVector3 because bullet
  /// uses floats instead of doubles.  When we report back to erlang side we need doubles
  /// to take the address of.  Using this gives the doubles a place to live for long enough
  /// to send.  (see note in the driver_data::step_simulation() function.
  struct vec {
  	double x,y,z;
  	vec() : x(0.0),y(0.0),z(0.0) {}
  	vec(const btVector3& v) : x(v.x()), y(v.y()), z(v.z()) {}
  };

  typedef std::map<uint64_t,vec> collision_map;

  /// Adds a collision event.  Overwrites any previous collision with that object.  Thus
  /// if A and B collide 5 times in one step, only the 5th is reported back to the Erlang side.
  void addCollision(uint64_t id, const btVector3& where)
  {
  	collisions[id]= vec(where);
  	dirty=true;
  }

  /// Resets the dirty flag and collisions.  This is done after information is reported back
  /// to the Erlang side and before the step_simulation() is considered done.
  void reset()
  {
  	collisions.clear();
		dirty=false;
  }

  bool isDirty() const { return dirty; }

	/// For iterating over collisions
  collision_map::iterator begin_collision() { return collisions.begin();}
	/// For iterating over collisions
  collision_map::iterator end_collision() { return collisions.end();}

  uint64_t get_id() const { return id;}

protected:
  btTransform mPos1;
  bool dirty;
  uint64_t id;
  collision_map collisions;
};

} // namespace erlybullet

#endif //ERLYBULLETMOTIONSTATE_H_INCLUDED__
