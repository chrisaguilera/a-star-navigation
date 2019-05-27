#include <iostream>
#include "PartiallyKnownGrid.h"
#include "GridPathPlanner.h"

using namespace std;

void Simulate(PartiallyKnownGrid* grid)
{
	// Use "GridPathPlanner planner(grid, true)" to test your Adaptive A* implementation.
	GridPathPlanner planner(grid, true);

	// Start simulation
	int steps = 0;
	int waitCounter = 100; // amount to wait between steps (milliseconds)
	grid->Reset();
	grid->DrawGrid();

	int i = 0;
	int numExpansionsFirst;
	int numExpansionsSecond;
	int totalExpansions = 0;
	while (!grid->GoalReached()) {  // loop until your robot find the target or dies
        xyLoc move_to = planner.GetNextMove(grid);

        // Call the simulator to move your robot and count the steps
        bool valid_move = grid->MoveTo(move_to);
        if (!valid_move) {
        	cout<<"Stopping simulation due to invalid move..."<<endl;
        	return;
        }

        steps++;
        grid->DrawGrid();

		totalExpansions+=planner.GetNumExpansions();
		if (i == 0) {
			numExpansionsFirst = planner.GetNumExpansions();
		} else if (i == 1) {
			numExpansionsSecond = planner.GetNumExpansions();
		}

        struct timespec req, rem;
        req.tv_nsec = 1000000*waitCounter;
        req.tv_sec = 0;
        nanosleep(&req, &rem);
				i++;
  }
	cout << "Target found in " << steps << " steps !!!" << endl;
	std::cout << "Number of expansions during first search: " << numExpansionsFirst << std::endl;
	std::cout << "Number of expansions during second search: " << numExpansionsSecond << std::endl;
	double avExpansions = totalExpansions/i;
	std::cout << "Average number of expansions: " << avExpansions << std::endl;

}

int main() {
	PartiallyKnownGrid grid("map.txt");
	Simulate(&grid);

	return 0;
}
