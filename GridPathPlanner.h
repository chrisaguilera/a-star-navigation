#ifndef GRID_PATH_PLANNER_H
#define GRID_PATH_PLANNER_H
#include <map>
#include <vector>

#include "PartiallyKnownGrid.h"

class GridPathPlanner{
public:
	GridPathPlanner(PartiallyKnownGrid* grid, bool use_adaptive_a_star = false);
	~GridPathPlanner();

	xyLoc GetNextMove(PartiallyKnownGrid* grid);
	int GetNumExpansions();
	void SetHeuristics();
	int GetG(xyLoc);
	int GetF(xyLoc);

private:
	bool using_adaptive;
	int gridHeight;
	int gridWidth;
	xyLoc goalLocation;
	std::map<xyLoc,xyLoc> parentMap;
	std::vector<std::vector<int> > heuristics;
	int numExpansions;

};

#endif
