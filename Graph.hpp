#pragma once

#include <unordered_map>
#include <unordered_set>
#include <deque>
#include <vector>
#include <initializer_list>
#include <iterator>
#include <utility>
#include <functional>

#include <boost/mpi.hpp>
#include <boost/serialization/deque.hpp>
#include <boost/serialization/unordered_set.hpp>
#include <boost/serialization/unordered_map.hpp>

// Class graph:
// Graph objects represent unweight and undirected graphs.
// Exposes functions to calculate simple measures, some of them in parallel.
// 
// Template parameters:
// * T: identifier type for each node;
// * hash: hash function to apply on each node's identifier (used by internal structures of this class)
//   (it must be a function-like type that takes a parameter of type T and returns a size_t)
//   (defaults to STL's hash function (std::hash<T>);
// * pred: function to compare whether two nodes are equal (by comparing their identifiers)
//   (it must be a function-like type that takes two parameters of type T and returns a bool);
//   (defaults to operator == between T and T (std::equal_to<T>).
template<class T, class hash = std::hash<T>, class pred = std::equal_to<T>>
class graph {
public:

  // Following the philsophy of STL container development,
  // this class has quite a number of inner classes.

  // "node_type" is an alias for "T", the first template argument;
  // it is the type of the identifier for the node.
  using node_type = T;

  // "hasher" is an alias for "hash", the second template argument;
  // it is the hash function.
  using hasher = hash;

  // "node_equal" is an alias for "pred", the third template
  // argument;
  // it is the comparer function.
  using node_equal = pred;

  // "size" type is an alias for "size_t";
  // it is a type that can hold a size, which can be a very
  // large non-negative number (usually, "size_t" is an alias for
  // "unsigned long int", thus satisfying the conditions to
  // hold a very large non-negative number).
  using size_type = size_t;

private:

  // Structure traversal_info:
  // traversal_info objects are simple objects that hold information releated to traversals;
  // this type is used in a mapping key'd by the node, so it holds infromation related to that node on the traversal.
  struct traversal_info {

    // This "friend" statement and the "serialize" member
    // function are there to be used by boost's seralization
    // library, which, in turn, is used by boost's MPI
    // scatter and gather operations.
    friend class boost::serialization::access;

    // The antecessor of the node (the node before this one on the traversal).
    node_type antecessor;

    // The disancte between the traversal source and this node.
    size_type distance;

    template<class Archive>
    void serialize(Archive& ar, const unsigned int version) {
      ar & this->antecessor;
      ar & this->distance;
    }

  };

public:

  // The aliases below are for convenience and readability:

  // "edge_list_type" is an alias for the adjacency list of each node;
  using edge_list_type = std::unordered_set<node_type, hasher, node_equal>;
  // "node_list_type" is an alias for the adjacency list itself,
  // which stores the nodes along with their own adjacency lists;
  using node_list_type = std::unordered_map<node_type, edge_list_type, hasher, node_equal>;

  // "node_set" is a set of the identifier type of nodes
  // (covenient, because of wtihout it, it would become pretty
  // unreadable, thanks to all those template arguments);
  using node_set = std::unordered_set<node_type, hasher, node_equal>;
  // "node_map" is a mapping of the identifier type of nodes
  // into another type (definied due to the same reasons of
  // "node_set");
  template<class Value>
  using node_map = std::unordered_map<node_type, Value, hasher, node_equal>;

  // "node_count_type" is a convience alias for mapping of the
  // node identifier type into size type (counts some property
  // related to the node);
  using node_count_type = node_map<size_type>;

  // "traversal_type" is a convenience alias for the type
  // returned by the traversal functions, i.e., a mapping of
  // each node, into its traversal-related info.
  using traversal_type = node_map<traversal_info>;

private:

  // Structure traversal_step:
  // This inner structure is the heart of the parallel BFS algorithm.
  // It holds information this algorithm uses in each of its steps.
  struct traversal_step {

    // This "friend" statement and the "serialize" member
    // function are there to be used by boost's seralization
    // library, which, in turn, is used by boost's MPI
    // scatter and gather operations.
    friend class boost::serialization::access;

    // The nodes that will be visited this step.
    // (deque is used over queue due to serialization performance reasons).
    std::deque<node_type> immediate_visits;

    // The nodes that are to be visited next step
    // (they don't need to be unique, since trying to ensure uniqueness is slower than allowing multiple visits to each node);
    // (deque is used over queue due to serialization performance reasons).
    std::deque<node_type> future_visits;

    // All nodes visited so far.
    node_set visited;

    // The traversal result.
    traversal_type info;

    template<class Archive>
    void serialize(Archive& ar, const unsigned int version) {
      ar & this->immediate_visits;
      ar & this->future_visits;
      ar & this->visited;
      ar & this->info;
    }

    // split member function:
    // Splits the information contained in this object into multiple smaller objects, so that they can be scatter'd.
    // The algorithm ensures that all the parts have the same amount of info (number of nodes to visit);
    // if this size is not pertectly divisible into parts, the largest divisible number is used and a remainder is created.
    // The algorithm also empties this object's immediate visits when splitting it into the smaller ones.
    // Parameters:
    // * parts: the number of parts to split in.
    // Returns:
    //   A pair (2-tuple), in which the first member is a vector with the smaller objects / split result,
    //   and the second member, the reaminder of the division.
    std::tuple<std::vector<traversal_step>, traversal_step> split(size_type parts) {

      // The split result.
      std::vector<traversal_step> retval;

      // The amount of work each smaller object will have
      // (for this algortihm, work == number of nodes to visit in the current step).
      size_type payload = this->immediate_visits.size() / parts;

      // For each part:
      for (size_type i = size_type(0); i < parts; i++) {

        // Create the smaller object.
        traversal_step current_step;

        // Already visited nodes and traversal result are
        // fully copied into each smaller object; this way,
        // all objects know everything about the past,
        // eliminating the need of one of them having
        // to query the other about such info.
        current_step.visited = this->visited;
        current_step.info = this->info;

        // Builds the nodes to be visited, by moving an amout
        // of nodes to visited from the main object into the
        // current smaller one equals to the calculated
        // payload.
        for (size_type j = size_type(0); j < payload; j++) {
          current_step.immediate_visits.push_back(this->immediate_visits.front());
          this->immediate_visits.pop_front();
        }

        // Inserts this smaller object into the vector that
        // stores the split result.
        retval.push_back(current_step);

      }

      // The same algorithm is repeated to build the reaminder.
      // (it will move all remaining nodes to visit from the
      // main object to the remainder).
      traversal_step remaining;
      remaining.visited = this->visited;
      remaining.info = this->info;
      while (!this->immediate_visits.empty()) {
        remaining.immediate_visits.push_back(this->immediate_visits.front());
        this->immediate_visits.pop_front();
      }

      // Creates the tuple with the splitted parts and the
      // reaminder and returns it.
      return std::make_tuple(retval, remaining);

    }

    // merge static member function:
    // Merges the information contained in multiple objects into a single one, so that the gather'd information can be unified.
    // This function calls the non-static merge function, and depends heavily on it.
    // Parameters:
    // * steps: vector containing the small objects to merged into a single one.
    // Returns:
    //   The result object.
    static traversal_step merge(std::vector<traversal_step>& steps) {

      // Creates an empty object.
      traversal_step retval;

      // Fills it with info from every small object in the
      // vector (by calling non-static merge).
      for (traversal_step& step : steps) {
        retval.merge(step);
      }

      // Returns the object, now filled with all info
      // from the smaller ones.
      return retval;

    }

    // merge member function (non-static):
    // Merges the information from another object into the current one.
    // The algoritm empties the other object's future visits, and fills this one's immediate visits.
    // Parameters:
    // * step: object from which the information will be taken and merged into this one.
    void merge(traversal_step& step) {

      // Moves each node marked as future visit to this
      // object's immediate visits
      // (this is because a new step in the traversal will
      // start, making all future visits become immediate).
      while (!step.future_visits.empty()) {
        this->immediate_visits.push_back(step.future_visits.front());
        step.future_visits.pop_front();
      }

      // Inserts every visited from the object mbeing merged
      // into this one
      // (since it's a set, no need to handle duplicates).
      for (const node_type& v : step.visited) {
        this->visited.insert(v);
      }

      // Also copies the info acquired from the object being
      // merged and completes the merge operation.
      for (const std::pair<node_type, traversal_info>& i : step.info) {

        // The first info acquired from one node is the one
        // that is going to be kept.
        if (!this->info.count(i.first)) {
          this->info.insert(i);
        }

      }

    }

  };

public:

  // Below are iterator classes.
  // They iterate over a pair of nodes (the edges).
  // They are pretty self-explainatory, so they won't
  // be deeply documentated, for now.

  class const_iterator : public std::iterator<std::forward_iterator_tag, std::pair<node_type, node_type>> {

  protected:

    friend class graph;

    graph* _iterating;
    typename node_list_type::iterator _node_iterator;
    typename edge_list_type::iterator _edge_iterator;

    const_iterator(graph* iterating, const typename node_list_type::iterator& node, const typename edge_list_type::iterator& edge)
      : _iterating(iterating), _node_iterator(node), _edge_iterator(edge) {}

  public:

    explicit const_iterator() : _iterating(nullptr) {}
    const_iterator(const const_iterator& x) = default;
    const_iterator& operator =(const const_iterator& rhs) = default;

    std::pair<const node_type&, const node_type&> operator *() {
      return std::pair<const node_type&, const node_type&>(_node_iterator->first, *_edge_iterator);
    }

    std::pair<const node_type*, const node_type*> operator ->() {
      return std::pair<const node_type*, const node_type*>(&(_node_iterator->first), &(*_edge_iterator));
    }

    const_iterator& operator ++() {

      ++this->_edge_iterator;

      if (this->_edge_iterator == this->_node_iterator->second.end()) {

        ++this->_node_iterator;
        if (this->_node_iterator != this->_iterating->_adjacency.end()) {
          this->_edge_iterator = this->_node_iterator->second.begin();
        }

      }

      return *this;

    }

    const_iterator operator ++(int) {
      const_iterator _this(*this);
      ++(*this);
      return _this;
    }

    bool operator ==(const const_iterator& rhs) const {

      return (this->_node_iterator == rhs._node_iterator) && (this->_node_iterator == this->_iterating->_adjacency.end() || this->_edge_iterator == rhs._edge_iterator);

    }

    inline bool operator !=(const const_iterator& rhs) const {
      return !(*this == rhs);
    }

  };

  class iterator : public const_iterator {

    friend class graph;

  private:

    iterator(graph* iterating, const typename node_list_type::iterator& node, const typename edge_list_type::iterator& edge)
      : const_iterator(iterating, node, edge) {}

  public:

    explicit iterator() : const_iterator() {}
    iterator(const iterator& x) = default;
    iterator& operator =(const iterator& rhs) = default;

  };

private:

  // The adjacency info.
  node_list_type _adjacency;

public:

  // All constructors have to build a single member only.
  // The default implementations for most constructors are fine on
  // what they should do, and the non-default ones can be simply
  // solved by delegating the construction its member.
  // Therefore, there is no reason not to use it.

  // Default constructor:
  // Creates an empty graph (no vertices nor edges).
  explicit graph() = default;

  // Copy constructor:
  // Creates a graph from another, by cloning its adjacency info.
  graph(const graph& x) = default;

  // Move constructor:
  // Creates a graph from another, by moving its adjacency info.
  graph(graph&& x) = default;

  // Constructor:
  // Creates a graph with the supplied adjacency info, containing vertices and edges.
  graph(const node_list_type& x) : _adjacency(x) {}

  // Constructor:
  // Creates a graph with the supplied adjacency info, containing vertices and edges.
  graph(node_list_type&& x) : _adjacency(x) {}

  // Constructor:
  // Creates a graph with its adjacency info supplied as an intializer info, thus allwoing uniform initiaization.
  graph(std::initializer_list<std::pair<node_type, node_type>> il) {

    // For each pair present in the initializer list:
    for (auto e : il) {

      // Adds the one node, if still not in the graph;
      if (!this->_adjacency.count(e.first)) {
        this->add_node(e.first);
      }

      // Adds the other one, if still not in the graph;
      if (!this->_adjacency.count(e.second)) {
        this->add_node(e.second);
      }

      // With both guranteed to be in the graph, creates the
      // edge between them.
      this->add_edge(e.first, e.second);

    }

  }

  // begin member function:
  // Gets an iterator to start iterating from it.
  // Returns:
  //   Iterator to the beginning of this container (in valid state).
  iterator begin() {
    return iterator(this, this->_adjacency.begin(), this->_adjacency.begin()->second.begin());
  }

  // end member function:
  // Gets an iterator that indicates end of iteration.
  // Returns:
  //   Iterator past the end of this container (in invalid state).
  iterator end() {
    return iterator(this, this->_adjacency.end(), edge_list_type::iterator());
  }

  // cbegin member function:
  // Gets a constant iterator to start iterating from it.
  // Returns:
  //   Constant iterator to the beginning of this container (in valid state).
  const_iterator cbegin() {
    return const_iterator(this, this->_adjacency.begin(), this->_adjacency.begin()->second.begin());
  }

  // cend member function:
  // Gets an iterator that indicates end of iteration.
  // Returns:
  //   Constant iterator past the end of this container (in invalid state).
  const_iterator cend() {
    return const_iterator(this, this->_adjacency.end(), edge_list_type::const_iterator());
  }

  // order member function:
  // Gets the order of the graph, i.e., number of vertices / nodes.
  // Returns:
  //    Graph order.
  size_type order() const {
    // Since the adjacency info structure stores one set
    // of linked nodes per node, the number size of it
    // is the total number of nodes in the graph.
    return this->_adjacency.size();
  }

  // size member function:
  // Gets the size of the graph, i.e., the number of edges / links;
  // Returns:
  //   Graph size.
  size_type size() const {

    // The size of the graph is the sum of the sizes
    // of each node's own adjacency list.
    size_type retval = 0;
    for (const edge_list_type& edges : this->_adjacency) {
      retval += edges.size();
    }

    return retval;

  }

  // neighbors member function:
  // Gets the neighbors of a node, i.e., the nodes that are linked to a specific node.
  // Parameters:
  // * node: the node whose neighbors will be gotten;
  // Returns:
  //   The neighbors of argument node.
  const edge_list_type& neighbors(const node_type& node) const {
    // "at" is preferred over [], because the former throws an
    // exception (that will be automatically rethrown), and
    // because it can be used in a method labeled with "const".
    return this->_adjacency.at(node);
  }

  // operator []:
  // Gets the neighbors of a node, i.e., the nodes that are linked to a specific node.
  // Parameters:
  // * node: the node whose neighbors will be gotten;
  // Returns:
  //   The neighbors of argument node.
  inline const edge_list_type& operator [](const node_type& node) const {
    return this->neighbors(node);
  }

  // neighbors member function:
  // Gets the neighbors of all nodes, which, is none more than the adjacency info itself.
  // Returns:
  //   Adjacency / neighboring info.
  const node_list_type& neighbors() const {
    return this->_adjacency;
  }

  // degree member function:
  // Gets the degree of a node, i.e., the number of links connected to that node.
  // Parameters:
  // * node: the node whose degree will be calculated;
  // Returns:
  //   The degree of argument node.
  size_type degree(const node_type& node) const {
    return this->_adjacency.at(node).size();
  }

  // degree member function:
  // Gets the degrees of all nodes in a mapping.
  // Returns:
  //   Mapping, in which the nodes are keys, and the values, the degrees.
  node_count_type degree() const {

    std::unordered_map<T, size_t, hash, pred> retval;

    for (std::pair<T, edge_list_type> it : this->_adjacency) {
      retval.emplace(it.first, this->degree(it.first));
    }

    return retval;

  }

  // minimum_path member function
  // Calculates the minimum path from a source to all other nodes, i.e.,
  // how to traverse starting from one node targeting every other, while traversing the least links possible.
  // Parameters:
  // * source: the source of the traversal;
  // * parallel: whether the traversal should be done in parallel or not (defaults to false).
  // Returns:
  //   Traversal info filled with the minimum paths.
  virtual traversal_type minimum_path(const node_type& source, bool parallel = false) const {
    return this->breadth_first_search(source, parallel);
  }

  // minimum_path member function
  // Performs the Breadth First Search (BFS) traversal algorithm.
  // Parameters:
  // * source: the source of the traversal;
  // * parallel: whether the traversal should be done in parallel or not (defaults to "no").
  // Returns:
  //   Traversal info filled with the minimum paths.
  traversal_type breadth_first_search(const node_type& source, bool parallel = false) const {
    return parallel ? this->_breadth_first_search_parallel(source) : this->_breadth_first_search_sequential(source);
  }

private:

  traversal_type _breadth_first_search_sequential(const node_type& source) const {

    traversal_type retval;

    std::unordered_set<node_type> visited;
    std::deque<node_type> to_visit;

    for (const node_type& neighbor : this->neighbors(source)) {
      retval.emplace(neighbor, traversal_info { source, 1 });
      to_visit.push_back(neighbor);
    }
    visited.insert(source);

    while (!to_visit.empty()) {

      node_type& current = to_visit.front();
      to_visit.pop_front();

      if (visited.count(current)) {
        continue;
      }

      for (const node_type& neighbor : this->neighbors(current)) {

        if (!retval.count(neighbor)) {

          if (!visited.count(neighbor)) {
            retval.emplace(neighbor, traversal_info{ this->_adjacency.find(current)->first, retval.at(current).distance + size_type(1) });
            to_visit.push_back(neighbor);
          }

        }

      }

      visited.insert(current);

    }

    return retval;

  }

  traversal_type _breadth_first_search_parallel(const node_type& source) const {

    traversal_step retval;

    for (const node_type& neighbor : this->neighbors(source)) {
      retval.info.emplace(neighbor, traversal_info { source, 1 });
      retval.immediate_visits.push_back(neighbor);
    }
    retval.visited.insert(source);

    std::vector<traversal_step> global;
    traversal_step remaining;
    std::vector<traversal_step> result;

    boost::mpi::environment env;
    boost::mpi::communicator world;
    bool master = world.rank() == 0;

    while (!retval.immediate_visits.empty()) {

      std::tie(global, remaining) = retval.split(world.size());

      traversal_step local;

      boost::mpi::scatter(world, global, local, 0);
      this->_bfs_visit_neighbors(local);

      if (master) {
        this->_bfs_visit_neighbors(remaining);
      }

      boost::mpi::gather(world, local, result, 0);
      retval = traversal_step::merge(result);

      if (master) {
        retval.merge(remaining);
      }

    }

    return retval.info;

  }

  void _bfs_visit_neighbors(traversal_step& step) const {

    while (!step.immediate_visits.empty()) {

      node_type& current = step.immediate_visits.front();
      step.immediate_visits.pop_front();

      if (step.visited.count(current)) {
        continue;
      }

      for (const node_type& neighbor : this->neighbors(current)) {

        if (!step.info.count(neighbor) && !step.visited.count(neighbor)) {

          step.info.emplace(neighbor, traversal_info{ this->_adjacency.find(current)->first, step.info.at(current).distance + size_type(1) });
          step.future_visits.push_back(neighbor);

        }

      }

      step.visited.insert(current);

    }

  }

public:

  // closness_centrality member function:
  // Calculates the closeness centrality of a node.
  // Parameters:
  // * source: the node whose closeness centrality will be calculated;
  // * parallel: whether the calculation should done in parallel or not (defaults to "no").
  // Returns:
  //   Closeness centrality value.
  double closeness_centrality(const node_type& source, bool parallel = false) const {

    double closeness = 0.0;
    traversal_type min_paths = this->minimum_path(source, parallel);

    for (const std::pair<T, traversal_info>& info : min_paths) {
      closeness += 1.0 / info.second.distance;
    }

    return closeness;

  }

  // add_node member function:
  // Adds a node to the graph.
  // Parameters:
  // * node: the node to be added.
  void add_node(const node_type& node) {
    this->_adjacency.emplace(node, edge_list_type());
  }

  // add_edge member function:
  // Adds an edge between to already existant nodes to the graph.
  // Since the graph is undirected, it does not matter who is the source,and who is the destination.
  // Parameters:
  // * source: the one end of the edge;
  // * destination: the other end of the edge.
  void add_edge(const node_type& source, const node_type& destination) {
    this->_adjacency.at(source).insert(destination);
    this->_adjacency.at(destination).insert(source);
  }

};
