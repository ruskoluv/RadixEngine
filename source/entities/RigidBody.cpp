#include <radix/entities/RigidBody.hpp>

namespace radix {

RigidBody::RigidBody(Entity &ent) :
  Component(ent),
  body(nullptr) {
}

RigidBody::RigidBody(Entity &ent, float mass,
  const std::shared_ptr<btCollisionShape> &collisionshape) :
  Component(ent), shape(collisionshape) {
  if (not entity.hasComponent<Transform>()) {
    entity.addComponent<Transform>();
  }
  Transform &tform = entity.getComponent<Transform>();
  motionState.setWorldTransform(btTransform(tform.getOrientation(), tform.getPosition()));
  btVector3 localInertia;
  collisionshape->calculateLocalInertia(mass, localInertia);
  btRigidBody::btRigidBodyConstructionInfo ci(mass, &motionState, shape.get(), localInertia);
  body = new btRigidBody(ci);
  body->setUserPointer(&entity);
}

RigidBody::~RigidBody() {
  delete body;
}

void RigidBody::serialize(serine::Archiver &ar) {
  /// @todo RigidBody serialization
}

void RigidBody::setPosition(const Vector3f &val) {
  position = val;
  if (entity.hasComponent<RigidBody>()) {
    btRigidBody &rb = *entity.getComponent<RigidBody>().body;
    btTransform t = rb.getWorldTransform();
    t.setOrigin(val);
    rb.setWorldTransform(t);
  } else if (entity.hasComponent<Player>()) {
    entity.getComponent<Player>().controller->warp(val);
  }
}

void RigidBody::setScale(const Vector3f &val) {
  scale = val;
}

void RigidBody::setOrientation(const Quaternion &val) {
  orientation = val;
  if (entity.hasComponent<RigidBody>()) {
    btRigidBody &rb = *entity.getComponent<RigidBody>().body;
    btTransform t = rb.getWorldTransform();
    t.setRotation(val);
    rb.setWorldTransform(t);
  }
}

} /* namespace radix */
