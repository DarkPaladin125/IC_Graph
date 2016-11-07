#include <iostream>
#include <string>

#include "Graph.hpp"

using namespace std;

int main() {

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

	cout << g.closeness_centrality(1) << endl;

    return 0;

}