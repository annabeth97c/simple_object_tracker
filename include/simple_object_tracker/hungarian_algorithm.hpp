#include <iostream>
#include <vector>
#include <algorithm>

using namespace std;

const int INF = 1e9; // A large value to use as an initial value for some variables

// Implementation of the Hungarian algorithm for finding a maximum-weighted perfect matching in a bipartite graph.
static vector<int> hungarian(vector<vector<int>>& cost) {
    int n = cost.size(), m = cost[0].size(); // n = number of vertices in the left part of the bipartite graph, m = number of vertices in the right part
    vector<int> u(n + 1), v(m + 1), p(m + 1), way(m + 1); // Some auxiliary vectors
    vector<int> match(n + 1, -1); // match[i] is the index of the vertex in the right part of the graph that is matched with vertex i in the left part
    for (int i = 1; i <= n; i++) { // Loop over all vertices in the left part of the graph
        p[0] = i; // The first vertex in the path is i
        int j0 = 0; // The index of the previous vertex in the path is 0
        vector<int> minv(m + 1, INF), used(m + 1, false); // minv[j] is the minimum cost of an edge entering j, used[j] is true if vertex j is already in the path
        do {
            used[j0] = true; // Mark vertex j0 as visited
            int i0 = p[j0], delta = INF, j1; // i0 is the vertex that precedes j0 in the path, delta is the minimum value of minv[j] for j not in the path, j1 is the vertex that minimizes minv[j]
            for (int j = 1; j <= m; j++) {
                if (!used[j]) { // If vertex j is not in the path
                    int cur = cost[i0-1][j-1] - u[i0] - v[j]; // The reduced cost of edge (i0,j) is cost[i0-1][j-1] - u[i0] - v[j]
                    if (cur < minv[j]) { // If the reduced cost is less than minv[j]
                        minv[j] = cur; // Update minv[j]
                        way[j] = j0; // Update the index of the previous vertex in the path for vertex j
                    }
                    if (minv[j] < delta) { // Update delta if necessary
                        delta = minv[j];
                        j1 = j;
                    }
                }
            }
            for (int j = 0; j <= m; j++) { // Update u and v
                if (used[j]) {
                    u[p[j]] += delta;
                    v[j] -= delta;
                }
                else {
                    minv[j] -= delta;
                }
            }
            j0 = j1; // Move to the next vertex in the path
        } while (p[j0] != 0); // Repeat until the last vertex in the path is 0
        do { // Update the augmenting path
            int j1 = way[j0];
            p[j0] = p[j1];
            j0 = j1;
        } while (j0);
    }
    for (int j = 1; j <= m; j++) { // Get the matching
        if (p[j] > 0) {
            match[p[j]] = j;
        }
    }
    return match;
}