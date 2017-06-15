#ifndef RADIX_ENTITIES_RIGID_BODY_HPP
#define RADIX_ENTITIES_RIGID_BODY_HPP

#include <memory>

#include <bullet/BulletCollision/CollisionShapes/btCollisionShape.h>
#include <bullet/BulletDynamics/Dynamics/btRigidBody.h>
#include <bullet/LinearMath/btDefaultMotionState.h>

#include <radix/Entity.hpp>

namespace radix {
namespace entities {

class RigidBody : public virtual Entity {
public:
  std::shared_ptr<btCollisionShape> shape;
  btDefaultMotionState motionState;
  btRigidBody *body;

  RigidBody();
  ~RigidBody();

  const char* getName() const {
    return "RigidBody";
  }

  virtual void setPosition(const Vector3f&) override;
  virtual void setScale(const Vector3f&) override;
  virtual void setOrientation(const Quaternion&) override;
};

} /* namespace entities */
} /* namespace radix */

#endif /* RADIX_ENTITIES_RIGID_BODY_HPP */
