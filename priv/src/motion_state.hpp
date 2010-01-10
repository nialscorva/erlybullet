#ifndef ERLYBULLETMOTIONSTATE_H_INCLUDED__
#define ERLYBULLETMOTIONSTATE_H_INCLUDED__

#include<LinearMath/btMotionState.h>
#include<LinearMath/btTransform.h>


namespace erlybullet {
class motion_state : public btMotionState {
public:
  motion_state(const btVector3 &initial_pos, uint64_t entity_id) :dirty(false), id(entity_id) {
  	mPos1.setIdentity();
  	mPos1.setOrigin(initial_pos);
  }

  virtual ~motion_state() {
  }

  virtual void getWorldTransform(btTransform &worldTrans) const {
      worldTrans = mPos1;
  }
  virtual void setWorldTransform(const btTransform &worldTrans) {

  	dirty=true;
  	mPos1=worldTrans;
  }

  struct vec {
  	double x,y,z;
  	vec() : x(0.0),y(0.0),z(0.0) {}
  	vec(const btVector3& v) : x(v.x()), y(v.y()), z(v.z()) {}
  };

  typedef std::map<uint64_t,vec> collision_map;

  void addCollision(uint64_t id, const btVector3& where)
  {

  	collisions[id]= vec(where);
  	dirty=true;
  }

  void reset()
  {
  	collisions.clear();
		dirty=false;
  }
  bool isDirty() const { return dirty; }

  collision_map::iterator begin_collision() { return collisions.begin();}
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
