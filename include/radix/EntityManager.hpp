#ifndef RADIX_ENTITY_MANAGER_HPP
#define RADIX_ENTITY_MANAGER_HPP

#include <list>
#include <unordered_map>
#include <utility>
#include <type_traits>

#include <radix/Entity.hpp>

namespace radix {

class World;

/** \class EntityManager
 * @brief Manager and container of @ref Entity "entities"
 */
class EntityManager final : protected std::list<std::unique_ptr<Entity>> {
private:
  friend class Entity;
  std::unordered_map<std::string, Entity&> nameMap;
  void changeEntityName(Entity&, const std::string&, const std::string&);

  EntityId m_lastAllocatedId;
  EntityId allocateId() {
    return m_lastAllocatedId++;
  }

  using Base = std::list<std::unique_ptr<Entity>>;

public:
  using Base::front;
  using Base::back;
  using Base::begin;
  using Base::cbegin;
  using Base::end;
  using Base::cend;
  using Base::rbegin;
  using Base::crbegin;
  using Base::rend;
  using Base::crend;
  using Base::empty;
  using Base::size;
  using Base::max_size;

  World &world;
  EntityManager(World&);

  template<typename T, typename... Args>
  T& create(Args... args) {
    static_assert(std::is_base_of<Entity, T>::value, "T must derive from Entity");
    emplace_back(Entity::CreationParams(world, allocateId()), std::forward<Args>(args)...);
    return back();
  }

  /**
   * Gets the reference to the entity with specified ID.
   * @throws std::out_of_range if no entity with this ID is found.
   */
  Entity& getById(EntityId id);

  /**
   * Gets the reference to the entity with specified name.
   * @throws std::out_of_range if no entity with this name is found.
   */
  Entity& getByName(const std::string &name);
};

} /* namespace radix */

#endif /* RADIX_ENTITY_MANAGER_HPP */
