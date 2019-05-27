CXX = g++
CXXFLAGS = -Wall -g

test: main.o PartiallyKnownGrid.o GridPathPlanner.o
	$(CXX) $(CXXFLAGS) -o test main.o PartiallyKnownGrid.o GridPathPlanner.o

main.o: main.cpp PartiallyKnownGrid.h GridPathPlanner.o
	$(CXX) $(CXXFLAGS) -c main.cpp

PartiallyKnownGrid.o: PartiallyKnownGrid.h

GridPathPlanner.o: PartiallyKnownGrid.h GridPathPlanner.h