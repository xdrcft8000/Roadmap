/*
Tayyeb Rafique
Algorithms and Data Structures Coursework
Final mark - 100/100

The program models a roadmap with multiple locations, some locations have charging
stations available.

A method is provided for checking if two locations are connected with a path that includes charging stations.
A depth first search is used, adding neighbours to a stack (using a Deque) and checking the most recently added node first,
acheiving a time complexity of O(vertices+edges) for any given map.

Another method is given for computing the shortest path between a given start and end node, where
all the nodes in between have charging stations.
The solution implements a depth first search with a binary heap minimum priority queue to find the optimal path.
 */

 
import java.util.*;
import java.io.*;

class Vertex {

	// Constructor: Create a point on the map.
	// Set name, chargingStation and index according to given values,
	// initilaize incidentRoads as empty array
	public Vertex(String placeName, boolean chargingStationAvailable, int idx) {
		name = placeName;
		incidentRoads = new ArrayList<Edge>();
		index = idx;
		chargingStation = chargingStationAvailable;
	}

	public String getName() {
		return name;
	}

	public boolean hasChargingStation() {
		return chargingStation;
	}

	public ArrayList<Edge> getIncidentRoads() {
		return incidentRoads;
	}

	// Add a road to the array incidentRoads
	public void addIncidentRoad(Edge road) {
		incidentRoads.add(road);
	}

	public int getIndex() {
		return index;
	}

	public void setIndex(int newIndex){
		index = newIndex;
	}

	public PVertex getPVertex(){
		return PVertex;
	}
	public void setPVertex(PVertex newPVertex){
		PVertex = newPVertex;
	}

	private PVertex PVertex = null; // Representation of the vertex that can hold priority value for the minimum priority heap
	private String name; // Name of the place
	private ArrayList<Edge> incidentRoads; // Incident edges
	private boolean chargingStation; // Availability of charging station
	private int index; // Index of this vertex in the vertex array of the map
}

class Edge {
	public Edge(int roadLength, Vertex firstPlace, Vertex secondPlace) {
		length = roadLength;
		incidentPlaces = new Vertex[] { firstPlace, secondPlace };
	}

	public Vertex getFirstVertex() {
		return incidentPlaces[0];
	}
	public void setFirstVertex(Vertex newV) {
		incidentPlaces[0] = newV;
	}
	public Vertex getSecondVertex() {
		return incidentPlaces[1];
	}
	public void setSecondVertex(Vertex newV) {
		incidentPlaces[1] = newV;
	}

	public int getLength() {
		return length;
	}

	private int length;
	private Vertex[] incidentPlaces;
}

//Represents the priority of each vertex in the minimum priority queue
class PVertex{
	private Vertex place;
	private boolean valid = true;
	private int priority;

	public PVertex(Vertex place, int priority){
		this.place = place;
		this.priority = priority;
		place.setPVertex(this);
	}

	public Vertex getVertex(){
		return place;
	}

	public int getPri(){
		return priority;
	}

	public void setInvalid(){
		this.valid = false;
	}

	public boolean isValid(){
		return valid;
	}
}

class PriorityQueue{

	public PriorityQueue() {
		heap = new PVertex[5];
	}

	public void insert(Vertex newVertex, int priority){
		if (isFull()){
			resize(2 * heap.length);
		}
		PVertex newPVertex = new PVertex(newVertex, priority);
		heap [++position] = newPVertex;
		siftUp();
	}

//Marks the old node as invalid and adds a new node with the updated priority into the heap.
	public void update(Vertex vertex, int priority){
		if (vertex.getPVertex() == null) insert(vertex,priority);
		else{
			vertex.getPVertex().setInvalid();
			insert(vertex,priority);
		}
	}

//returns the minimum priority vertex
	public Vertex getMin(){
		if (heap.length==0){
			return null;
		}
		PVertex min = heap[0];
		//Moves through invalid nodes before returning the next popped node.
		while(min!=null && !min.isValid()){
			heap[0] = heap[position--];
			heap[position + 1] = null;
			siftDown(position);
			min = heap[0];
		}
		if (position==-1){
			return null;
		}
		heap[0] = heap[position--];
		heap[position + 1] = null;
		siftDown(position);
		min.getVertex().setPVertex(null);

		return min.getVertex();
	}

	public boolean isEmpty(){
		if (position == -1)return true;
		else return false;
	}

//Performs binary tree node restructuring from the top down
	private void siftDown(int endIndex){
		if (endIndex == -1) return;
		int index = 0;
		while (index <= endIndex) {
			int leftChildIndex = 2 * index +1;
			int rightChildIndex = 2 * index +2;
			if (leftChildIndex > endIndex) break;

			int childToSwap = rightChildIndex > endIndex ? leftChildIndex : heap[leftChildIndex].getPri() < heap[rightChildIndex].getPri() ? leftChildIndex : rightChildIndex;

			if (heap[index].getPri() < heap[childToSwap].getPri()) break;
			swap(index, childToSwap);
			index = childToSwap;
		}
	}

	//Performs binary tree node restructuring from the bottom up
	private void siftUp(){
		int index = position;
		int parentIndex = (index - 1) / 2;
		while (parentIndex >= 0 && (heap[index].getPri()) < (heap[parentIndex].getPri())){
			swap(index, parentIndex);
			index = parentIndex;
			parentIndex = (index -1) / 2;
		}
	}

	private void swap(int firstIndex, int secondIndex){
		PVertex holder = heap[firstIndex];
		heap[firstIndex] = heap[secondIndex];
		heap[secondIndex] = holder;
	}

	private void resize(int newSize){
		System.arraycopy(heap, 0, heap = new PVertex[newSize], 0, position+1);
	}
	private boolean isFull(){
		return position == heap.length -1;
	}
	private PVertex[] heap;
	private int position = -1;
}


// A class that represents a sparse matrix
public class RoadMap {

	// Default constructor
	public RoadMap() {
		places = new ArrayList<Vertex>();
		roads = new ArrayList<Edge>();
	}

	// Auxiliary function that prints out the command syntax
	public static void printCommandError() {
		System.err.println("ERROR: use one of the following commands");
		System.err.println(" - Read a map and print information: java RoadMap -i <MapFile>");
		System.err.println(
				" - Read a map and find shortest path between two vertices with charging stations: java RoadMap -s <MapFile> <StartVertexIndex> <EndVertexIndex>");
	}

	public static void main(String[] args) throws Exception {
		if (args.length == 2 && args[0].equals("-i")) {
			RoadMap map = new RoadMap();
			try {
				map.loadMap(args[1]);
			} catch (Exception e) {
				System.err.println("Error in reading map file");
				System.exit(-1);
			}

			System.out.println("Read road map from " + args[1] + ":");
			map.printMap();
		} else if (args.length == 4 && args[0].equals("-s")) {
			RoadMap map = new RoadMap();
			map.loadMap(args[1]);
			System.out.println("Read road map from " + args[1] + ":");
			map.printMap();

			int startVertexIdx = -1, endVertexIdx = -1;
			try {
				startVertexIdx = Integer.parseInt(args[2]);
				endVertexIdx = Integer.parseInt(args[3]);
			} catch (NumberFormatException e) {
				System.err.println("Error: start vertex and end vertex must be specified using their indices");
				System.exit(-1);
			}

			if (startVertexIdx < 0 || startVertexIdx >= map.numPlaces()) {
				System.err.println("Error: invalid index for start vertex");
				System.exit(-1);
			}

			if (endVertexIdx < 0 || endVertexIdx >= map.numPlaces()) {
				System.err.println("Error: invalid index for end vertex");
				System.exit(-1);
			}

			Vertex startVertex = map.getPlace(startVertexIdx);
			Vertex endVertex = map.getPlace(endVertexIdx);
			if (!map.isConnectedWithChargingStations(startVertex, endVertex)) {
				System.out.println();
				System.out.println("There is no path connecting " + map.getPlace(startVertexIdx).getName() + " and "
						+ map.getPlace(endVertexIdx).getName() + " with charging stations");
			} else {
				ArrayList<Vertex> path = map.shortestPathWithChargingStations(startVertex, endVertex);
				System.out.println();
				System.out.println("Shortest path with charging stations between " + startVertex.getName() + " and "
						+ endVertex.getName() + ":");
				map.printPath(path);
			}

		} else {
			printCommandError();
			System.exit(-1);
		}
	}



	// Load matrix entries from a text file

	int numVertices;

	public void loadMap(String filename) {
		File file = new File(filename);
		places.clear();
		roads.clear();

		try {
			Scanner sc = new Scanner(file);

			// Read the first line: number of vertices and number of edges
			numVertices = sc.nextInt();
			int numEdges = sc.nextInt();
			for (int i = 0; i < numVertices; ++i) {
				// Read the vertex name and its charing station flag
				String placeName = sc.next();
				int charginStationFlag = sc.nextInt();
				boolean hasChargingStataion = (charginStationFlag == 1);

				Vertex vx = new Vertex(placeName, hasChargingStataion, i);
				places.add(vx);

			}

			for (int j = 0; j < numEdges; ++j) {
				// Read the edge length and the indices for its two vertices
				int vtxIndex1 = sc.nextInt();
				int vtxIndex2 = sc.nextInt();
				int length = sc.nextInt();
				Vertex vtx1 = places.get(vtxIndex1);
				Vertex vtx2 = places.get(vtxIndex2);

				Edge ed = new Edge(length, vtx1,vtx2);
				roads.add(ed);
				vtx1.addIncidentRoad(ed);
				vtx2.addIncidentRoad(ed);

			}

			sc.close();

		} catch (Exception e) {
			e.printStackTrace();
			places.clear();
			roads.clear();
		}
	}


	// Return the shortest path between two given vertex, with charging stations on
	// each itermediate vertex.

	public ArrayList<Vertex> shortestPathWithChargingStations(Vertex startVertex, Vertex endVertex) {

		// Initialize an empty path
		ArrayList<Vertex> path = new ArrayList<Vertex>();

		// Sanity check for the case where the start vertex and the end vertex are the
		// same
		if (startVertex.getIndex() == endVertex.getIndex()) {
			path.add(startVertex);
			return path;
		}

		//keeps track of total distances using the vertex index as a key
		HashMap<Integer, Integer> distance = new HashMap<Integer, Integer>();
		HashMap<Integer, Vertex> previousVector = new HashMap<Integer, Vertex>();

		//uses a binary heap to keep track of the minimum priority vertex
		PriorityQueue minPQ = new PriorityQueue();
		boolean[] visited = new boolean[numVertices];

		distance.put(startVertex.getIndex(),0);
		minPQ.insert(startVertex, 0);

		//sets the distance for all nodes to ~infinity apart form the start node
		for (int i=0; i<numVertices; i++){
			if (places.get(i) != startVertex && (places.get(i).hasChargingStation() || places.get(i) == endVertex)){
				distance.put(places.get(i).getIndex(),Integer.MAX_VALUE);
			}
		}


		while (!minPQ.isEmpty()){
			Vertex currentVertex = minPQ.getMin();
			if (currentVertex == null) break;
			for (int i = 0 ; i < currentVertex.getIncidentRoads().size(); i ++){
				Vertex neighbour;
				Edge currentEdge = currentVertex.getIncidentRoads().get(i);
				if (currentEdge.getFirstVertex() == currentVertex){
					neighbour = currentEdge.getSecondVertex();
				}
				else{
					neighbour = currentEdge.getFirstVertex();
				}
				if (!visited[neighbour.getIndex()] && (neighbour.hasChargingStation() || neighbour == endVertex)){
					//checks if the new path to the node is shorter than its existing path
					int altPath = distance.get(currentVertex.getIndex()) + currentEdge.getLength();
					if (altPath < distance.get(neighbour.getIndex())){
						distance.put(neighbour.getIndex(), altPath);
						previousVector.put(neighbour.getIndex(), currentVertex);
						minPQ.update(neighbour, altPath);
					}
				}
			}
		}
		Vertex tracker = endVertex;
		while (tracker!= startVertex){
			path.add(0,tracker);
			Vertex prev = previousVector.get(tracker.getIndex());
			tracker = prev;
		}
		path.add(0,tracker);
		return path;
	}

	// Check if two vertices are connected by a path with charging stations on each itermediate vertex.
	// Return true if such a path exists; return false otherwise.


	public boolean isConnectedWithChargingStations(Vertex startVertex, Vertex endVertex) {
		// Sanity check
		if (startVertex.getIndex() == endVertex.getIndex()) {
			return true;
		}
		boolean[] visited = new boolean[numVertices];
		Deque<Vertex> stack = new LinkedList<>();

		//adds the start node's neighbours first to bypass the charging station check.
		for (int i=0; i<startVertex.getIncidentRoads().size(); i++){
			if (startVertex.getIncidentRoads().get(i).getFirstVertex() == startVertex){
				stack.push(startVertex.getIncidentRoads().get(i).getSecondVertex());
			}
			else {
				stack.push(startVertex.getIncidentRoads().get(i).getFirstVertex());
			}
		}
		visited[startVertex.getIndex()]= true;

		while (!stack.isEmpty()){
			Vertex current = stack.pop();
			if (current == endVertex){
				return true;
			}
			if (!visited[current.getIndex()]){
				visited[current.getIndex()] = true;
				if (current.hasChargingStation()){
					for (int i=0; i<current.getIncidentRoads().size(); i++){
							Vertex AdjVertex;
						if (current.getIncidentRoads().get(i).getFirstVertex() == current){
							AdjVertex = current.getIncidentRoads().get(i).getSecondVertex();
						}
						else {
							AdjVertex = current.getIncidentRoads().get(i).getFirstVertex();
						}
						if (!visited[AdjVertex.getIndex()]){
								stack.push(AdjVertex);
						}
					}
				}
			}
		}
		return false;
	}



	public void printMap() {
		System.out.println("The map contains " + this.numPlaces() + " places and " + this.numRoads() + " roads");
		System.out.println();

		System.out.println("Places:");

		for (Vertex v : places) {
			System.out.println("- name: " + v.getName() + ", charging station: " + v.hasChargingStation());
		}

		System.out.println();
		System.out.println("Roads:");

		for (Edge e : roads) {
			System.out.println("- (" + e.getFirstVertex().getName() + ", " + e.getSecondVertex().getName()
					+ "), length: " + e.getLength());
		}
	}

	public void printPath(ArrayList<Vertex> path) {
		System.out.print("(  ");

		for (Vertex v : path) {
			System.out.print(v.getName() + "  ");
		}

		System.out.println(")");
	}

	public int numPlaces() {
		return places.size();
	}

	public int numRoads() {
		return roads.size();
	}

	public Vertex getPlace(int idx) {
		return places.get(idx);
	}

	private ArrayList<Vertex> places;
	private ArrayList<Edge> roads;

}
