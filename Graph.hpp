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

template<class T, class hash = std::hash<T>, class pred = std::equal_to<T>>
class graph {
public:

    using node_type = T;
    using hasher = hash;
    using node_equal = pred;
    using size_type = size_t;

private:

	struct traversal_info {

		friend class boost::serialization::access;

		node_type antecessor;
		size_type distance;

		template<class Archive>
		void serialize(Archive& ar, const unsigned int version) {
			ar & this->antecessor;
			ar & this->distance;
		}

	};

public:

    using edge_list_type = std::unordered_set<node_type, hasher, node_equal>;
    using node_list_type = std::unordered_map<node_type, edge_list_type, hasher, node_equal>;

    using node_set = std::unordered_set<node_type, hasher, node_equal>;
    template<class Value>
    using node_map = std::unordered_map<node_type, Value, hasher, node_equal>;

	using node_count_type = node_map<size_type>;
    
	using traversal_type = node_map<traversal_info>;

private:

    struct traversal_step {

		friend class boost::serialization::access;

        std::deque<node_type> immediate_visits;
        std::deque<node_type> future_visits;
        node_set visited;
        traversal_type info;

		template<class Archive>
		void serialize(Archive& ar, const unsigned int version) {

			ar & this->immediate_visits;
			ar & this->future_visits;
			ar & this->visited;
			ar & this->info;

		}

        std::tuple<std::vector<traversal_step>, traversal_step> split(size_type parts) {

            std::vector<traversal_step> retval;
            size_type payload = this->immediate_visits.size() / parts;

            for (size_type i = size_type(0); i < parts; i++) {

                traversal_step current_step;
                current_step.visited = this->visited;
                current_step.info = this->info;

                for (size_type j = size_type(0); j < payload; j++) {
                    current_step.immediate_visits.push_back(this->immediate_visits.front());
                    this->immediate_visits.pop_front();
                }

                retval.push_back(current_step);

            }

            traversal_step remaining;
            remaining.visited = this->visited;
            remaining.info = this->info;
            while (!this->immediate_visits.empty()) {
                remaining.immediate_visits.push_back(this->immediate_visits.front());
                this->immediate_visits.pop_front();
            }

            return std::make_tuple(retval, remaining);

        }

        static traversal_step merge(std::vector<traversal_step>& steps) {

            traversal_step retval;

            for (traversal_step& step : steps) {
                retval.merge(step);
            }

			return retval;

        }

        void merge(traversal_step& step) {

            while (!step.future_visits.empty()) {
                this->immediate_visits.push_back(step.future_visits.front());
                step.future_visits.pop_front();
            }

            for (const node_type& v : step.visited) {
                this->visited.insert(v);
            }

            for (const std::pair<node_type, traversal_info>& i : step.info) {

                if (!this->info.count(i.first)) {
                    this->info.insert(i);
                }

            }

        }

    };

public:

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

    node_list_type _adjacency;

public:

    explicit graph() = default;

    graph(const node_list_type& x) : _adjacency(x) {}
	graph(const graph& x) = default;
    graph(node_list_type&& x) : _adjacency(x) {}
	graph(graph&& x) = default;

    graph(std::initializer_list<std::pair<node_type, node_type>> il) {

        for (auto e : il) {

            if (!this->_adjacency.count(e.first)) {
                this->add_node(e.first);
            }

            if (!this->_adjacency.count(e.second)) {
                this->add_node(e.second);
            }

            this->add_edge(e.first, e.second);

        }

    }

    iterator begin() {
        return iterator(this, this->_adjacency.begin(), this->_adjacency.begin()->second.begin());
    }

    iterator end() {
		return iterator(this, this->_adjacency.end(), edge_list_type::iterator());
    }

	const_iterator cbegin() {
		return const_iterator(this, this->_adjacency.begin(), this->_adjacency.begin()->second.begin());
	}

	const_iterator cend() {
		return const_iterator(this, this->_adjacency.end(), edge_list_type::const_iterator());
	}

	size_type order() const {
		return this->_adjacency.size();
	}

	size_type size() const {

		size_type retval = 0;
		for (const edge_list_type& edges : this->_adjacency) {
			retval += edges.size();
		}

		return retval;

	}

    const edge_list_type& neighbors(const node_type& node) const {
        return this->_adjacency.at(node);
    }

    inline const edge_list_type& operator [](const node_type& node) const {
        return this->neighbors(node);
    }

    const node_list_type& neighbors() const {
        return this->_adjacency;
    }

    size_type degree(const node_type& node) const {
        return this->_adjacency.at(node).size();
    }

    node_count_type degree() const {

        std::unordered_map<T, size_t, hash, pred> retval;

        for (std::pair<T, edge_list_type> it : this->_adjacency) {
            retval.emplace(it.first, this->degree(it.first));
        }

        return retval;

    }

	virtual traversal_type minimum_path(const node_type& source, bool parallel = false) const {
		return this->breadth_first_search(source, parallel);
	}

	traversal_type breadth_first_search(const node_type& source, bool parallel = false) const {
		return parallel ? this->_breadth_first_search_parallel(source) : this->_breadth_first_search_sequential(source);
	}

private:

	traversal_type _breadth_first_search_sequential(const node_type& source) const {

		traversal_type retval;

		std::unordered_set<node_type> visited;
		std::deque<node_type> to_visit;

		for (const node_type& neighbor : this->neighbors(source)) {
			retval.emplace(neighbor, traversal_info {source, 1});
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
						retval.emplace(neighbor, traversal_info {this->_adjacency.find(current)->first, retval.at(current).distance + size_type(1)});
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
            retval.info.emplace(neighbor, traversal_info {source, 1});
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

                    step.info.emplace(neighbor, traversal_info {this->_adjacency.find(current)->first, step.info.at(current).distance + size_type(1)});
                    step.future_visits.push_back(neighbor);

				}

			}

			step.visited.insert(current);

		}

    }

public:

	double closeness_centrality(const node_type& source, bool parallel = false) const {

		double closeness = 0.0;
		traversal_type min_paths = this->minimum_path(source, parallel);

		for (const std::pair<T, traversal_info>& info : min_paths) {
			closeness += 1.0 / info.second.distance;
		}

		return closeness;

	}

    void add_node(const node_type& node) {
        this->_adjacency.emplace(node, edge_list_type());
    }

    void add_edge(const node_type& source, const node_type& destination) {
        this->_adjacency.at(source).insert(destination);
        this->_adjacency.at(destination).insert(source);
    }

};
