#include <vector>
#include <iostream>
#include "hull.h"
#include <chrono>

using namespace std;
using namespace std::chrono;

// Function to generate a random set of points
vector<pair<int, int>> generateRandomPoints(int numPoints, int xRange, int yRange) {
    vector<pair<int, int>> points;
    srand(time(0)); // Seed for random number generation

    for (int i = 0; i < numPoints; ++i) {
        int x = rand() % xRange;   // Random x-coordinate within range
        int y = rand() % yRange;   // Random y-coordinate within range
        points.push_back({x, y});
    }

    return points;
}

int main() {
    // int n;
    // cin >> n;

    // vector<pair<int,int>> points;
    // for (int i = 0; i < n; i++) {
    //     int x, y;
    //     cin >> x >> y;
    //     points.push_back({x, y});
    // }

    int numPoints = 50000;
    int xRange = 100;
    int yRange = 100;

    vector<pair<int, int>> randp = generateRandomPoints(numPoints, xRange, yRange);

    auto start = high_resolution_clock::now();

    // Compute the convex hullS
    Hull hull = Hull(randp);
    auto end = high_resolution_clock::now();

    // Calculate the elapsed time
    auto duration = duration_cast<microseconds>(end - start);

    // Output the time in microseconds
    cout << "Elapsed time: " << duration.count() << " microseconds" << endl;

    return 0;
}