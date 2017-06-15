#include <radix/SimulationManager.hpp>

#include <radix/env/Util.hpp>
#include <radix/World.hpp>

namespace radix {

SimulationManager::SimulationRunner::SimulationRunner(SimulationManager &sysMan) :
  threads(std::thread::hardware_concurrency()),
  exit(false) {
  for (unsigned i = 0; i < std::thread::hardware_concurrency(); ++i) {
    threads.emplace_back(std::bind(&SimulationRunner::threadProc, this, std::ref(sysMan)));
    Util::SetThreadName(threads.back(), ("SimulationRunner thread " + std::to_string(i)).c_str());
  }
}

void SimulationManager::SimulationRunner::threadProc(SimulationManager &sysMan) {
  while (!exit) {
    SimulationTypeId processStid;
    { std::unique_lock<std::mutex> lk(queueMutex);
      queueCondVar.wait(lk, [this](){ return exit || queue.size() > 0; });
      if (exit) {
        return;
      }
      processStid = queue.front();
      queue.pop();
    }
    sysMan.systems[processStid]->update(dtime);
    { std::unique_lock<std::mutex> lk(queueMutex);
      SimulationGraphNode &sgn = *sysMan.systemGraph[processStid];
      for (SimulationTypeId nextStid : sgn.successors) {
        SimulationGraphNode &nextSgn = *sysMan.systemGraph[nextStid];
        std::unique_lock<std::mutex> clk(nextSgn.counterMut);
        ++nextSgn.counter;
        if (nextSgn.counter == nextSgn.predecessors.size()) {
          queue.push(nextStid);
          queueCondVar.notify_one();
        }
      }
    }
    { std::unique_lock<std::mutex> lk(runCountMutex);
      ++runCount;
      runCountCondVar.notify_all();
    }
  }
}

SimulationManager::SimulationRunner::~SimulationRunner() {
  exit = true;
  queueCondVar.notify_all();
  for (std::thread &thr : threads) {
    if (thr.joinable()) {
      thr.join();
    }
  }
}

SimulationManager::SimulationGraphNode::SimulationGraphNode(SimulationTypeId system,
  SimulationPtrVector &systems, SimulationGraphNodeVector &graph) :
  system(system),
  systems(systems),
  graph(graph),
  index(indexUndef),
  lowlink(lowlinkUndef),
  onStack(false) {
}

using SimulationTypeId = SimulationManager::SimulationTypeId;
using SimulationGraphNode = SimulationManager::SimulationGraphNode;
using SimulationGraphNodeVector = SimulationManager::SimulationGraphNodeVector;

static void strongconnect(SimulationTypeId &index, std::stack<SimulationTypeId> &S,
  SimulationTypeId stid, SimulationGraphNode &si, SimulationGraphNodeVector &sinfo,
  SimulationManager::SimulationLoopVector &stronglyConnected) {
  // Set the depth index for v to the smallest unused index
  si.index = index;
  si.lowlink = index;
  ++index;
  S.push(stid);
  si.onStack = true;

  // Consider successors of s
  for (SimulationTypeId wtid : si.successors) {
    SimulationGraphNode &wi = *sinfo.at(wtid);
    if (wi.index == SimulationGraphNode::indexUndef) {
      // Successor w has not yet been visited; recurse on it
      strongconnect(index, S, wtid, wi, sinfo, stronglyConnected);
      si.lowlink = std::min(si.lowlink, wi.lowlink);
    } else if (wi.onStack) {
      // Successor w is in stack S and hence in the current SCC
      si.lowlink = std::min(si.lowlink, wi.index);
    }
  }

  // If v is a root node, pop the stack and generate an SCC
  if (si.lowlink == si.index) {
    stronglyConnected.emplace_back();
    std::set<SimulationTypeId> &strong = stronglyConnected.back();
    SimulationTypeId wtid;
    do {
      wtid = S.top();
      S.pop();
      SimulationGraphNode &wi = *sinfo.at(wtid);
      wi.onStack = false;
      strong.insert(wtid);
    } while (wtid != stid);
  }
}

static bool isReachableBySuccessors(const SimulationGraphNode &start, SimulationTypeId targetstid,
  const SimulationGraphNodeVector &sinfo, std::stack<SimulationTypeId> &path) {
  auto search = start.successors.find(targetstid);
  if (search != start.successors.end()) {
    path.push(targetstid);
    return true;
  } else {
    for (SimulationTypeId stid : start.successors) {
      if (isReachableBySuccessors(*sinfo[stid], targetstid, sinfo, path)) {
        path.push(stid);
        return true;
      }
    }
  }
  return false;
}

static void df(SimulationGraphNode &vertex0, SimulationGraphNode &child0,
        std::set<SimulationTypeId> &done, SimulationGraphNodeVector &sinfo) {
  if (done.find(child0.system) != done.end()) {
    return;
  }
  for (SimulationTypeId child : child0.successors) {
    vertex0.successors.erase(child);
    SimulationGraphNode &childNi = *sinfo[child];
    childNi.predecessors.erase(vertex0.system);
    df(vertex0, childNi, done, sinfo);
  }
  done.insert(child0.system);
}

#if 0
static void dumpGraph(const std::string &path, const std::vector<std::unique_ptr<Simulation>> &systems,
  const SimulationGraphNodeVector &sinfo) {
  std::ofstream dot;
  dot.open(path, std::ios_base::out | std::ios_base::trunc);
  dot << "digraph SimulationRunGraph {" << std::endl;
  for (const std::unique_ptr<SimulationGraphNode> &sgnptr : sinfo) {
    if (sgnptr) {
      if (sgnptr->successors.size() > 0) {
        for (SimulationTypeId succStid : sgnptr->successors) {
          dot << systems[sgnptr->system]->getName() << " -> " << systems[succStid]->getName() << ';' << std::endl;
        }
      } else {
        dot << systems[sgnptr->system]->getName() << ';' << std::endl;
      }
    }
  }
  dot << "}";
  dot.close();
  std::system(("dot -O -Tpng " + path).c_str());
}
#endif

void SimulationManager::computeSimulationOrder() {
  systemGraph.clear();
  systemGraph.reserve(systems.size());
  // Each Simulation can request to be run before other Simulations, and after some others.
  // This allows for the injection of Simulations between two other in the execution graph, which
  // otherwise wouldn't be possible with only 1 of the "run before" and "run after" condition.
  // 1-way dependencies exhibit a single issue in our case: creating loops in the graph. This is
  // fairly easy to detect thanks to Tarjan's strongly connected components algorithm.
  // Having Simulations both requesting to run before and after some others, even more loops might get
  // introduced, i.e. A wants to run both before and after B, even indirectly; e.g. A runs before B
  // & after C, but C runs after B -- keeping in mind the graph we eventually want to get is a
  // plain Directed Acyclic Graph.
  //
  // A solution to successfully create an execution graph is the following:
  // Step 1: create basic "run before" directed graph.
  // Step 2: check for loops with Tarjan. If none, proceed.
  // Step 3: add "run after" deps by reversing them (B runs after A, so A runs before B).
  //         If there already exists a reverse path from B to A then it's incoherent
  //         (because transitively B runs after A, therefore cannot also run before A).
  // Step 4: delete any edges that can be transitively obtained (i.e. only keeping the longest
  //         paths), a.k.a. transitive reduction (as opposed to transitive closure).

  /* Step 0, O(V) */ {
    for (const std::unique_ptr<Simulation> &sptr : systems) {
      if (sptr) {
        SimulationTypeId stid = sptr->getTypeId();
        systemGraph.emplace_back(new SimulationGraphNode(stid, systems, systemGraph));
      } else {
        systemGraph.emplace_back();
      }
    }
  }

  /* Step 1, O(V²) best & worst case (O(V(V-1)) really) */ {
    for (const std::unique_ptr<Simulation> &sptr : systems) {
      if (sptr) {
        SimulationTypeId stid = sptr->getTypeId();
        SimulationGraphNode &si = *systemGraph[stid];
        for (const std::unique_ptr<Simulation> &rbsptr : systems) {
          if (rbsptr and rbsptr != sptr and sptr->runsBefore(*rbsptr)) {
            SimulationTypeId rbstid = rbsptr->getTypeId();
            si.successors.insert(rbstid);
            systemGraph[rbstid]->predecessors.insert(stid);
          }
        }
      }
    }
  }

  /* Step 2, O(|V| + |E|) worst case */ {
    SimulationTypeId index = 0;
    std::stack<SimulationTypeId> S;
    SimulationLoopVector stronglyConnected;
    for (const std::unique_ptr<Simulation> &sptr : systems) {
      if (sptr) {
        SimulationTypeId stid = sptr->getTypeId();
        SimulationGraphNode &si = *systemGraph[stid];
        if (si.index == SimulationGraphNode::indexUndef) {
          strongconnect(index, S, stid, si, systemGraph, stronglyConnected);
        }
      }
    }
    for (const std::set<SimulationTypeId> &elems : stronglyConnected) {
      if (elems.size() > 1) {
        throw RunsBeforeCreatesLoopsException(std::move(stronglyConnected));
      }
    }
  }

  /* Step 3, O(V² × <?>) */ {
    SimulationLoopPath succReachPath;
    for (const std::unique_ptr<Simulation> &sptr : systems) {
      if (sptr) {
        SimulationTypeId stid = sptr->getTypeId();
        SimulationGraphNode &si = *systemGraph[stid];
        for (const std::unique_ptr<Simulation> &rasptr : systems) {
          if (rasptr and rasptr != sptr and sptr->runsAfter(*rasptr)) {
            SimulationTypeId rastid = rasptr->getTypeId();
            // Check if the Simulation to run after is already reachable, if yes, it would
            // create a loop; complain.
            if (isReachableBySuccessors(si, rastid, systemGraph, succReachPath)) {
              succReachPath.push(stid);
              throw RunsAfterCreatesLoopException(std::move(succReachPath));
            }
            systemGraph[rastid]->successors.insert(stid);
            si.predecessors.insert(rastid);
          }
        }
      }
    }
  }

  /* Step 4, O(<?>) */ {
    // http://stackoverflow.com/a/11237184/1616310
    for (std::unique_ptr<SimulationGraphNode> &sgnptr : systemGraph) {
      if (sgnptr) {
        std::set<SimulationTypeId> done;
        for (SimulationTypeId child : sgnptr->successors) {
          df(*sgnptr, *systemGraph[child], done, systemGraph);
        }
      }
    }
  }
}

SimulationManager::SimulationManager(World &w) :
  w(w),
  systemRun(*this) {
}

void SimulationManager::dispatchEvent(const Event &evt) {
  w.event.dispatch(evt);
}

void SimulationManager::run(TDelta dtime) {
  std::unique_lock<std::mutex> lk(systemRun.runCountMutex);
  // Reset previous run count and set the dtime parameter to pass to Simulations
  systemRun.runCount = 0;
  systemRun.dtime = dtime;

  // Count the Simulations so we know when all of them were run. Also count the ones without
  // predecessors, i.e. the ones that runs first, to know how many threads to notify.
  SimulationTypeId targetRunCount = 0;
  unsigned int startCount = 0;
  { std::unique_lock<std::mutex> qlk(systemRun.queueMutex);
    for (std::unique_ptr<SimulationGraphNode> &sgn : systemGraph) {
      if (sgn) {
        sgn->counter = 0;
        if (sgn->predecessors.empty()) {
          systemRun.queue.push(sgn->system);
          ++startCount;
        }
        ++targetRunCount;
      }
    }
  }

  // Wake (notify) threads to start running Simulations
  while (startCount--) {
    systemRun.queueCondVar.notify_one();
  }

  // Wait for all Simulations to have run, i.e. runCount reached its target.
  systemRun.runCountCondVar.wait(lk, [this, targetRunCount]() {
    return systemRun.runCount == targetRunCount;
  });
}

} /* namespace radix */
