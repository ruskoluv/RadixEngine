#ifndef RADIX_SIMULATION_MANAGER_HPP
#define RADIX_SIMULATION_MANAGER_HPP

#include <condition_variable>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <queue>
#include <set>
#include <stack>
#include <thread>
#include <vector>

#include <radix/core/event/Event.hpp>
#include <radix/simulation/Simulation.hpp>

#undef max

namespace radix {

class World;

class SimulationManager {
protected:
  friend class World;
  World &w;
  SimulationManager(World&);

  void dispatchEvent(const Event&);
  void run(TDelta dtime);

public:
  using Simulation = simulation::Simulation;

  World& world() const {
    return w;
  }

  using SimulationPtrVector = std::vector<std::unique_ptr<Simulation>>;
  struct SimulationGraphNode;
  using SimulationGraphNodeVector = std::vector<std::unique_ptr<SimulationGraphNode>>;
  struct SimulationGraphNode {
    const Simulation &system;
    SimulationPtrVector &systems;
    SimulationGraphNodeVector &graph;

    std::set<SimulationTypeId> predecessors, successors;

    /* Tarjan's strongly connected components algothm data */
    SimulationTypeId index, lowlink;
    bool onStack;
    static constexpr decltype(index) indexUndef = std::numeric_limits<decltype(index)>::max();
    static constexpr decltype(lowlink) lowlinkUndef = std::numeric_limits<decltype(lowlink)>::max();

    /* Simulation execution variables */
    unsigned int counter;
    std::mutex counterMut;

    SimulationGraphNode(const Simulation &system, SimulationPtrVector &systems,
        SimulationGraphNodeVector &graph);
  };

protected:
  struct SimulationRunner {
    std::vector<std::thread> threads;
    std::queue<SimulationTypeId> queue;
    std::mutex queueMutex;
    std::condition_variable queueCondVar;
    bool exit;
    TDelta dtime;
    SimulationTypeId runCount;
    std::mutex runCountMutex;
    std::condition_variable runCountCondVar;
    SimulationRunner(SimulationManager&);
    ~SimulationRunner();
    void threadProc(SimulationManager&);
  } systemRun;

  SimulationPtrVector systems;
  SimulationGraphNodeVector systemGraph;
  std::map<std::string, Simulation&> systemsByName;

  void computeSimulationOrder();

public:
  using SimulationLoopPath = std::stack<SimulationTypeId>;
  using SimulationLoopVector = std::vector<std::set<SimulationTypeId>>;

  struct RunsBeforeCreatesLoopsException : public std::logic_error, SimulationLoopVector {
    RunsBeforeCreatesLoopsException(SimulationLoopVector &&slv) :
      std::logic_error("Execution graph contain loops"),
      SimulationLoopVector(slv) {}
  };
  struct RunsAfterCreatesLoopException : public std::logic_error, SimulationLoopPath {
    RunsAfterCreatesLoopException(SimulationLoopPath &&slp) :
      std::logic_error("Inserting a Simulation into the execution graph according to"
        "Simulation::runsAfter() would create a loop"),
      SimulationLoopPath(slp) {}
  };

  struct SimulationAddedEvent : public Event {
    static constexpr StaticEventTypeName TypeName = "radix/SimulationManager:SimulationAdded";
    const EventTypeName getTypeName() const {
      return TypeName;
    }
    static constexpr StaticEventType Type = TypeNameHash(TypeName);
    const EventType getType() const {
      return Type;
    }

    SimulationManager &sysMan;
    Simulation &system;
    SimulationAddedEvent(SimulationManager &sm, Simulation &s) :
      sysMan(sm), system(s) {}
  };
  struct SimulationRemovedEvent : public Event {
    static constexpr StaticEventTypeName TypeName = "radix/SimulationManager:SimulationRemoved";
    const EventTypeName getTypeName() const {
      return TypeName;
    }
    static constexpr StaticEventType Type = TypeNameHash(TypeName);
    const EventType getType() const {
      return Type;
    }

    SimulationManager &sysMan;
    Simulation &system;
    SimulationRemovedEvent(SimulationManager &sm, Simulation &s) :
      sysMan(sm), system(s) {}
  };

  class Transaction final {
  public:
    SimulationManager &sm;

  private:
    friend class SimulationManager;
    Transaction(SimulationManager &sm) : sm(sm) {}

  public:
    Transaction(const Transaction&) = delete;
    Transaction& operator=(const Transaction&) = delete;

    Transaction(Transaction&&) = default;
    Transaction& operator=(Transaction&&) = default;

    ~Transaction() {
      sm.computeSimulationOrder();
    }

    template<class T, typename... ArgsT> void addSimulation(ArgsT... args) {
      static_assert(std::is_base_of<Simulation, T>::value, "T must be a Simulation");
      const SimulationTypeId stid = Simulation::getTypeId<T>();
      if (sm.systems.size() <= stid) {
        sm.systems.resize(stid + 1);
      }
      sm.systems.at(Simulation::getTypeId<T>()).reset(new T(sm.w, std::forward<ArgsT>(args)...));
      Simulation &sys = *sm.systems.at(Simulation::getTypeId<T>());
      sm.systemsByName.emplace(std::piecewise_construct,
        std::forward_as_tuple(sys.getName()),
        std::forward_as_tuple(sys));
      sm.dispatchEvent(SimulationAddedEvent(sm, *sm.systems.at(Simulation::getTypeId<T>())));
    }

    template<class T> void removeSimulation() {
      Simulation &sys = *sm.systems.at(Simulation::getTypeId<T>());
      sm.dispatchEvent(SimulationRemovedEvent(sm, sys));
      sm.systemsByName.erase(sys.getName());
      sm.systemGraph.at(Simulation::getTypeId<T>()).reset(nullptr);
      sm.systems.at(Simulation::getTypeId<T>()).reset(nullptr);
    }
  };

  Transaction transact() {
    return Transaction(*this);
  }

  template<class T> T& get() {
    // We're casting down the class hierarchy, maybe use dynamic_cast instead ?
    // But unless the systems collection break, it should always be ok.
    return static_cast<T&>(*systems.at(Simulation::getTypeId<T>()));
  }

  template<class T, typename... ArgsT> void add(ArgsT... args) {
    Transaction st(*this);
    st.addSimulation<T>(std::forward(args)...);
  }

  template<class T> void remove() {
    Transaction st(*this);
    st.removeSimulation<T>();
  }
};

} /* namespace radix */

#endif /* RADIX_SIMULATION_MANAGER_HPP */
