# Roadmap
Find the shortest path between two nodes where all nodes in the path satisfy a condition.
A single file java program that takes a given map and calculates the shortest path between two nodes where all the in between nodes satisfy the condition.

It is modelled as a roadmap for electric vehicles, where the condition is that the node or location has a charging station.

A method is provided for checking if two locations are connected with a path that includes charging stations.
A depth first search is used, adding neighbours to a stack (using a Deque) and checking the most recently added node first,
acheiving a time complexity of O(vertices+edges) for any given map.

Another method is given for computing the shortest path between a given start and end node, where
all the nodes in between have charging stations.
The solution implements a depth first search with a binary heap minimum priority queue to find the optimal path.


Usage commands -   
1. Read a map and print information: java RoadMap -i <MapFile>");
2. Read a map and find shortest path between two vertices with charging stations: java RoadMap -s <MapFile> <StartVertexIndex> <EndVertexIndex>");

Two sample map files are provided in the repository. 
