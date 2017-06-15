#include <radix/Entity.hpp>

#include <radix/World.hpp>

namespace radix {

Entity::Entity(const CreationParams &cp) :
  world(cp.world), id(cp.id) {
}
Entity::~Entity() {
}

void Entity::setPosition(const Vector3f &val) {
  position = val;
}

void Entity::setScale(const Vector3f &val) {
  scale = val;
}

void Entity::setOrientation(const Quaternion &val) {
  orientation = val;
}

};
