#include "GridPathPlanner.h"
#include <stdlib.h>
#include <map>
#include <set>
#include <stack>
#include <vector>

GridPathPlanner::GridPathPlanner(PartiallyKnownGrid* grid, bool use_adaptive_a_star) {
	using_adaptive = use_adaptive_a_star;
	gridHeight = grid->GetHeight();
	gridWidth = grid->GetWidth();
	goalLocation = grid->GetGoalLocation();

	// Initialize heuristics
	heuristics.resize(gridWidth);
	for (int i = 0; i < gridWidth; ++i)
    heuristics[i].resize(gridHeight);
	SetHeuristics();

}
GridPathPlanner::~GridPathPlanner(){
	// TODO
}

xyLoc GridPathPlanner::GetNextMove(PartiallyKnownGrid* grid) {

	numExpansions = 0;
	std::set<xyLoc> openSet;
	std::set<xyLoc> closedSet;
	std::set<xyLoc>::iterator it;
	xyLoc start = grid -> GetCurrentLocation();
	xyLoc curr;
	openSet.insert(start);
	xyLoc startParent(-1, -1);
	parentMap.insert(std::pair<xyLoc, xyLoc>(start, startParent));

	while(curr != goalLocation) {
		// Get the smallest F value from the Open Set of states
		for (it = openSet.begin(); it != openSet.end(); ++it) {
			if (it == openSet.begin() || GetF(*it) <= GetF(curr)) {
				// If both states have the same F Value
				if (it != openSet.begin() && GetF(*it) == GetF(curr)) {
					// Prioritize higher G Value
					if (GetG(*it) > GetG(curr)) {
						curr = (*it);
					}
					// If both states have the same G Value
					else if (GetG(*it) == GetG(curr)) {
						// Prioritize the smaller xyLoc
						if ((*it) < curr) {
							curr = (*it);
						}
					}
				}
				// If F Value of (*it) < curr, choose it
				else {
					curr = (*it);
				}
			}
		}

		// Break if the Goal Location has been reached
		if (curr == goalLocation) {
			break;
		}

		// Remove the current location from the Open Set and add it to the Closed Set (Expanded)
		// Increment numExpansions when a state is expanded
		it = openSet.find(curr);
		openSet.erase(it);
		closedSet.insert(curr);
		numExpansions++;

		//Get the compass neighbors
		std::vector<xyLoc> neighbors;
		neighbors.push_back(xyLoc(curr.x+1, curr.y));
		neighbors.push_back(xyLoc(curr.x-1, curr.y));
		neighbors.push_back(xyLoc(curr.x, curr.y+1));
		neighbors.push_back(xyLoc(curr.x, curr.y-1));

		// Remove invalid neighbors
		for (int i = 0; i < neighbors.size(); i++) {
			xyLoc n = neighbors[i];
			if (!grid->IsValidLocation(n) || grid->IsBlocked(n)) {
				neighbors[i] = neighbors.back();
				neighbors.pop_back();
				i--;
			}
		}

		for (int i = 0; i < neighbors.size(); i++) {

			// If already in the Open Set, check if path through current is more cost effective than previous
			if (openSet.find(neighbors[i]) != openSet.end()) {
				if (GetG(neighbors[i]) > GetG(curr) + 1) {
					parentMap[neighbors[i]] = curr;
				}
			}

			// If not in the Open Set, and hasn't been expanded yet (Closed Set), insert into Open Set
			if (openSet.find(neighbors[i]) == openSet.end() && closedSet.find(neighbors[i]) == closedSet.end()) {
				openSet.insert(neighbors[i]);
				parentMap.insert(std::pair<xyLoc, xyLoc>(neighbors[i], curr));
			}

		}

	}

	//If using using_adaptive a *, update heuristics
	if (using_adaptive) {
		for (int i = 0; i < gridWidth; i++) {
			for (int j = 0; j < gridHeight; j++) {
				xyLoc s(i,j);
				heuristics[i][j] = GetG(goalLocation) - GetG(s);
			}
		}
	}

	// Create path from Start location to goal location and return the next location to move to
	std::stack<xyLoc> path;
	while (parentMap[curr] != startParent) {
		path.push(curr);
		curr = parentMap[curr];
	}
	parentMap.clear();
	return path.top();

}

int GridPathPlanner::GetNumExpansions() {
	return numExpansions;
}

void GridPathPlanner::SetHeuristics() {
	for (int i = 0; i < gridWidth; i++) {
		for (int j = 0; j < gridHeight; j++) {
			heuristics[i][j] = abs(goalLocation.x - i) + abs(goalLocation.y - j);
		}
	}
}


int GridPathPlanner::GetG(xyLoc loc) {
	if (loc.x == -1 && loc.y == -1)
		return 0;
	else
		return 1 + GetG(parentMap[loc]);
}

int GridPathPlanner::GetF(xyLoc loc) {
	return GetG(loc) + heuristics[loc.x][loc.y];
}
