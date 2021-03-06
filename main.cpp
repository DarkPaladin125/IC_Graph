#include <iostream>
#include <string>

#include "Graph.hpp"

using namespace std;

int main(int argc, char* argv[]) {

    graph<int> g = {
		{2, 6},
		{6, 7},
		{10, 3},
		{1, 3},
		{1, 4},
		{1, 2},
		{2, 5},
		{6, 8},
		{6, 9},
		{1, 5},
		{8, 9}
	};

	auto r = g.breadth_first_search(1, true);
	for (const auto& p : r) {
		cout << p.first << " -> (" << p.second.antecessor << ", " << p.second.distance << ")" << endl;
	}

    return 0;

}