package pathfinding;

import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

import java.util.ArrayList;
import java.util.Collections;

import annotations.MatrixCoordinates;

/**
 * Implementation of the A* search algorithm for ftc pathfinding
 *
 * Video on A*:
 * - https://www.youtube.com/watch?v=-L-WgKMFuhE
 * Wikipedia:
 * - https://en.wikipedia.org/wiki/A*_search_algorithm
 *
 * Steps:
 * - Create two node sets, OPEN and CLOSED, and set up map
 *      OPEN - the nodes to be evaluated as possible current positions for new paths
 *      CLOSED - the nodes already evaluated
 *      - Create a 2d array to be filled with Node objects. All spots without Nodes are null
 *      - Nodes are created when they're evaluated as neighbors
 * - Create a Node for the starting position and add it to the OPEN set
 *      - add Node to node map
 * - enter loop:
 *      - set CURRENT to Node in OPEN with lowest f_cost
 *      - remove CURRENT from OPEN
 *      - add CURRENT to CLOSED
 *      - if CURRENT is TARGET, return
 *
 *      - Begin looping through NEIGHBORS:
 *          - If neighbor not in NodeMap, create Node
 *          - If neighbor is not passable or is in CLOSED, skip to next neighbor
 *          - Calculate f_cost of neighbor through CURRENT node
 *          - if f_cost is shorter than previous, or neighbor is not in OPEN
 *              - set f_cost of neighbor to calculated value
 *              - change neighbor's parent to current node
 *              - if neighbor is not in OPEN
 *                  - add neighbor to OPEN
 *
 */
public class AStar {

    private static final String TAG = "vuf.test.a_star";

    private final AStarNode[][] nodeMap;
    private final ArrayList<AStarNode> openNodes;
    private final ArrayList<AStarNode> closedNodes;
    private final SpaceMap spaceMap;
    private AStarNode endNode;

    /**
     * Initialize the A_Star pathfinding algorithm
     * @param start_coords the start coordinates on the fieldmap matrix
     * @param target_coords the target coordinates on the fieldmap matrix
     * @param spaceMap a deep copy of the spacemap to pathfind on
     */
    public AStar(@NotNull @MatrixCoordinates int[] start_coords,
                 @NotNull @MatrixCoordinates int[] target_coords,
                 @NotNull SpaceMap spaceMap) {
//        Log.d(TAG, "initializing A_Star");
        this.spaceMap = spaceMap;

        nodeMap = new AStarNode[spaceMap.height][spaceMap.width];
        openNodes = new ArrayList<>();
        closedNodes = new ArrayList<>();

        // The target node is never added to the nodeMap. Instead, its just recreated when the algorithm reaches it
        endNode = new AStarNode(spaceMap.getSpace(target_coords), target_coords, null, null);
        AStarNode startNode = new AStarNode(spaceMap.getSpace(start_coords), start_coords, endNode, null);

        nodeMap[start_coords[0]][start_coords[1]] = startNode;
//        nodeMap[target_coords[0]][target_coords[1]] = targetNode;

        openNodes.add(startNode);
//        Log.d(TAG, "finished initialization");
    }

    /**
     * Finds the path through the fieldMap
     * @return the path, as an arraylist of Nodes from start to finish. If no path was found,
     * the path list is empty
     */
    @NotNull
    public ArrayList<AStarNode> findPath() {
        // calculate the path
        calculatePath();

        ArrayList<AStarNode> nodePath = new ArrayList<>();
        AStarNode previousParent = endNode.getParentNode();
        // if the end node's parent is null, then no path was found, so return an empty list
        if (previousParent == null) {
            return nodePath;
        }
        // otherwise, add all path nodes to the list
        nodePath.add(endNode);
        // add in reverse order so it goes start to end
        while (previousParent != null) {
            nodePath.add(0, previousParent);
            previousParent = previousParent.getParentNode();
        }
        return nodePath;
    }

    /**
     * Creates a space array containing a path
     * Adds Space types representing the path to the space map
     * If no path could be found, the return value will be null
     *
     * @param path the path to generate a space array for
     * @return a space array with the path drawn on, or null if no path was found
     */
    @Nullable
    public SpaceMap createPathFieldMap(@NotNull ArrayList<AStarNode> path) {
        // size of 0 indicates no path, so return null
        if (path.size() == 0) {
            return null;
        }
        // clone space map
        SpaceMap newSpaceMap = new SpaceMap(spaceMap);

        for (AStarNode node : path) {
            int[] mxCoords = node.getMatrixCoords();
            newSpaceMap.setSpace(SpaceMap.Space.PF_PATH, mxCoords, false);
        }

        AStarNode first = path.get(0);
        int[] firstCoords = first.getMatrixCoords();
        newSpaceMap.setSpace(SpaceMap.Space.PF_START, firstCoords, false);

        AStarNode last = path.get(path.size()-1);
        int[] lastCoords = last.getMatrixCoords();
        newSpaceMap.setSpace(SpaceMap.Space.PF_END, lastCoords, false);

        return newSpaceMap;
    }

    /**
     * Calculates the path from the start node to the target node using the A* algorithm
     *
     * Essentially, the start node is first added to the open list and becomes the current node
     * then, all of the current node's neighbors are explored
     * If the neighbor was previously explored or isn't passable, its skipped.
     *
     * if the neighbor can find a better path to the start through the current node, then
     * the current node becomes the neighbor's parent node. Then, if the neighbor node
     * isnt on the open list, its added so it can become the current node later
     *
     * The next current node is the node with the lowest f score, which is the sum of
     * the node's path to the start and the estimated distance to the end
     *
     * When the current node is the end node, then the algorithm has finished and a path is
     * constructed using the parent nodes of each node starting from the end node.
     */
    private void calculatePath() {
        // repeat while there's more nodes in the open list
        // if theres none, no path was found
        while(openNodes.size() > 0) {
            AStarNode currentNode = Collections.min(openNodes);
            openNodes.remove(currentNode);
            closedNodes.add(currentNode);

            // if the current node is the target node, set the target node's parent to the current
            // node's parent and return
            int[] currentCoords = currentNode.getMatrixCoords();
            if (currentNode.equals(endNode)) {
                nodeMap[currentCoords[0]][currentCoords[1]] = currentNode;
                endNode = currentNode;
                return;
            }
            // loop through all neighbors
            for (AStarNode neighborNode : getNeighborNodes(currentNode)) {
                checkNeighbor(currentNode, neighborNode);
            }
        }
    }

    /**
     * Checks the neighbor node to see if the current node provides a more favorable path
     * @param currentNode The current node, ie the node to calculate the neighbor from
     * @param neighborNode the neighbor node to check
     */
    private void checkNeighbor(@NotNull AStarNode currentNode, @NotNull AStarNode neighborNode) {
        // if the node isnt passable, or if the node is closed, dont check
        if (!neighborNode.isPassable() || closedNodes.contains(neighborNode)) {
            return;
        }
        // change the nodes parent if the f cost is lower
        neighborNode.changeParentIfLower(currentNode);
        // if the node isnt in openNodes, add it
        if (!openNodes.contains(neighborNode)) {
            openNodes.add(neighborNode);
        }

    }

    /**
     * Gets a node from the nodemap, creating it if necessary
     * @note this method will crash if matrix coordinates outside the bounds are provided
     *
     * @param matrixCoords the coordinates of the node on the nodemap
     * @param parentNode the parent node of the node to be used in initialization
     *                   if the node doesnt exist
     * @return the node
     */
    @NotNull
    public AStarNode getNode(@NotNull @MatrixCoordinates int[] matrixCoords, @NotNull AStarNode parentNode) {
        AStarNode node = nodeMap[matrixCoords[0]][matrixCoords[1]];
        if (node == null) {
            node = new AStarNode(spaceMap.getSpace(matrixCoords), matrixCoords, endNode, parentNode);
        }
        nodeMap[matrixCoords[0]][matrixCoords[1]] = node;
        return node;
    }

    /**
     * Get all neighbor nodes of the current node
     * If the neighbor doesnt exist yet, create it
     *
     * Also filters out out of bounds nodes
     *
     * @param current the node to get neighbors for
     * @return an arraylist of the neighbor nodes
     */
    @NotNull
    private ArrayList<AStarNode> getNeighborNodes(@NotNull AStarNode current) {
        ArrayList<AStarNode> nNodes = new ArrayList<>();
        for (int[] neighborVal : AStarNode.neighborIncrements) {
            int[] nCoords = current.calculateNeighborCoordinate(neighborVal);

            // gotta be within bounds
            if ((nCoords[0] < nodeMap.length)
                    && (nCoords[1] < nodeMap.length)
                    && (nCoords[0] >= 0)
                    && (nCoords[1] >= 0)) {
                AStarNode neighborNode = getNode(nCoords, current);
                nNodes.add(neighborNode);
            }
//            else {
//                Log.d(TAG, String.format("Coords {%s, %s} out of bounds", nCoords[0], nCoords[1]));
//            }

        }
        return nNodes;
    }



}
