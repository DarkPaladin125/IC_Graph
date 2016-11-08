#pragma once

#include <unordered_map>
#include <unordered_set>

#include <queue>

#include <initializer_list>
#include <iterator>

#include <utility>

#include <functional>

template<class T, class hash = std::hash<T>, class pred = std::equal_to<T>>
class graph {

public:

    using node_type = T;
    using hasher = hash;
    using key_equal = pred;
    using size_type = size_t;

	struct traversal {

		const T& antecessor;
		size_type distance;

	};

    using edge_list_type = std::unordered_set<node_type, hasher, key_equal>;
    using node_list_type = std::unordered_map<node_type, edge_list_type, hasher, key_equal>;

    using node_count_type = std::unordered_map<node_type, size_type, hasher, key_equal>;
	using traversal_type = std::unordered_map<node_type, traversal, hasher, key_equal>;

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
    graph(node_list_type&& x) : _adjacency(x) {}

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

	size_t order() const {
		return this->_adjacency.size();
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

		std::queue<node_type> to_visit;
		std::unordered_set<node_type> visited;

		for (const node_type& neighbor : this->neighbors(source)) {
			retval.emplace(neighbor, traversal { source, 1 });
			to_visit.push(neighbor);
		}
		visited.insert(source);

		while (!to_visit.empty()) {

			node_type& current = to_visit.front();
			to_visit.pop();

			if (visited.count(current)) {
				continue;
			}

			for (const node_type& neighbor : this->neighbors(current)) {

				if (!retval.count(neighbor)) {

					if (!visited.count(neighbor)) {
						retval.emplace(neighbor, traversal { this->_adjacency.find(current)->first, retval.at(current).distance + size_type(1) });
						to_visit.push(neighbor);
					}

				}

			}

			visited.insert(current);

		}

		return retval;

	}

	traversal_type _breadth_first_search_parallel(const node_type& source) const {
		throw std::exception("Not implemented yet!"); // TODO
	}

public:

	double closeness_centrality(const node_type& source, bool parallel = false) const {

		double closeness = 0.0;
		traversal_type min_paths = this->minimum_path(source, parallel);

		for (const std::pair<T, traversal>& info : min_paths) {
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
