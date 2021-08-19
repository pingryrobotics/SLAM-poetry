package pathfinding;


import android.util.Log;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import java.util.ArrayList;
import java.util.Collections;

import annotations.MatrixCoordinates;

/**
 * Implementation of the D* Lite pathfinding algorithm
 *
 * Paper on D* Lite:
 * http://idm-lab.org/bib/abstracts/papers/aaai02b.pdf
 *
 * Example of D* Lite (near end):
 * https://www.cs.cmu.edu/~motionplanning/lecture/AppH-astar-dstar_howie.pdf
 *
 *
 * parent in D* lite is basically neighbor in A*
 * child is what parent was
 *
 * Steps:
 * - initialize dstar
 *      - Create OPEN_LIST - arraylist of open nodes, or nodes that havent been explored fully
 *      - set minKey to 0
 *      - create 2d array of nodes
 *      - set each node's rhs and g to infinity (or equivalent)
 *      - set rhs of goal to 0
 *      - set node's key to [h_dist(start, goal), 0]
 *      - add node to open list
 * - compute shortest path
 *   - while (minimum key is less than start's key), or (start's rhs is greater than start's g_dist)
 *      - get minimum node, set as CURRENT
 *      - store minimum key as keyOld
 *      - recalculate CURRENT key, set as keyNew
 *      - if keyOld < keyNew
 *          - update CURRENT's key to keyNew
 *      - else, if (CURRENT's g_dist > CURRENT's rhs)
 *          - CURRENT's g_dist = rhs
 *          - remove CURRENT from OPEN_LIST
 *          - for parent : parents of CURRENT,
 *              - if parent is not the goal
 *                  - set parent's rhs to mininum of:
 *                  parent's rhs; (cost of parent -> CURRENT) + CURRENT's g_dist
 *                  UpdateVertex(parent)
 *      - else,
 *          - set CURRENT's g_dist to gOld
 *          - set CURRENT's g_dist to infinity
 *          - for NODE : parent's of CURRENT and CURRENT,
 *              - if NODE's rhs == cost of NODE -> CURRENT + gOld
 *                  - if NODE is not the goal,
 *                      - set NODE's rhs to the minimum of: CHILD : NODE's children
 *                         (cost of NODE -> CHILD) + CHILD's g_dist
 *             - UpdateVertex(NODE)
 * - Update vertex (NODE)
 *      - if NODE's g_dist != NODE's rhs, & NODE is in OPEN_LIST
 *          - update node's key
 *      - else if NODE's g_dist != NODE's rhs, & NODE is not in OPEN_LIST
 *          - update key and add node to OPEN_LIST
 *      - else if NODE's g_dist is NODE's rhs, & NODE is in OPEN_LIST
 *          - remove NODE from OPEN_LIST
 * - pathfind:
 *   - set LAST node to START node (last moved node)
 *   - Intialize()
 *   - ComputeShortestPath()
 *   - while (START != GOAL)
 *      - new START = argmin (argument that returns minimum value) of:
 *          - for CHILD : start's children,
 *              - (cost of START -> CHILD) + g_dist of CHILD
 *      - if any graph costs changed:
 *          - keyMin = keyMin + h_dist(LAST, START)
 *          - LAST = START
 *          - for CHANGED : all changed nodes
 *              - for AFFECTED : all nodes affected by CHANGED
 *                  - costOld = old cost of AFFECTED -> CHANGED
 *                  - update edge cost of AFFECTED -> CHANGED
 *                  - if (costOld > cost AFFECTED -> CHANGED)
 *                      - if AFFECTED != GOAL
 *                          - AFFECTED's rhs = minimum of:
 *                          AFFECTED's rhs; or (cost of AFFECTED -> CHANGED) + CHANGED's g_dist
 *                      - else if AFFECTED's rhs != costOld + g_dist(CHANGED)
 *                          - if (AFFECTED != GOAL)
 *                              - AFFECTED's rhs = minimum of: CHILD : AFFECTED's children:
 *                              (cost of AFFECTED -> CHILD) + CHILD's g_dist
 *                  - UpdateVertex(AFFECTED)
 *              - repeat for CHANGED with all nodes affected by CHANGED
 *          - ComputeShortestPath()
 *
 *
 *
 * Rhs is sort of a measure of consistency.
 * Over-consistent: g > rhs
 * Under-consistent: rhs > g
 *
 *
 * @ FIXME: 8/10/21 dlite sucks and i dont like it but its fast when there's a lot of obstructions,
 *                  so if the code was cleaned up a bit to make sure errors stop happening and aren't
 *                  absolutely godawful to debug, then d* lite would be nice to implement.
 *                  right now though, its super annoying to debug so A* is probably the way to go
 *                  just for simplicity alone
 *
 *
 */
public class DLite {

    private static final String TAG = "vuf.test.dlite";
    // how long openList can remain the same size before assuming there's no solution
    private static final int tolerance = 150;

    // this is a horrible way to do it
    // multiple D_Lite instances will break this
    public static int keyMin;
    private SpaceMap spaceMap;
    private final DLiteNode[][] nodeMap;
    private final ArrayList<DLiteNode> openList = new ArrayList<>();
    private DLiteNode lastNode;
    private DLiteNode startNode;
    private final DLiteNode endNode;

    /**
     * Initialize the D* Lite pathfinding algorithm
     * @param start_coords the start coordinates
     * @param target_coords the end coordinates
     * @param spaceMapDC a deepcopy of the fieldmap's spacemap to calculate the path based off
     */
    public DLite(@NonNull @MatrixCoordinates int[] start_coords,
                 @NonNull @MatrixCoordinates int[] target_coords,
                 SpaceMap spaceMapDC) {
        this.spaceMap = spaceMapDC;
        this.nodeMap = new DLiteNode[spaceMap.height][spaceMap.width];

        this.startNode = new DLiteNode(start_coords, spaceMap.getSpace(start_coords), null, null);
        lastNode = startNode;
        this.endNode = new DLiteNode(target_coords, spaceMap.getSpace(target_coords), startNode, null);
        keyMin = 0; // ik this is bad

        nodeMap[start_coords[0]][start_coords[1]] = startNode;
        nodeMap[target_coords[0]][target_coords[1]] = endNode;

        endNode.setRhs(0);
        endNode.setKey(new int[] {endNode.hDist(startNode), 0});
        openList.add(endNode);
    }

    /**
     * Calculate the path on the spacemap
     * @return the path as a spacemap, or null if none found
     */
    @Nullable
    public SpaceMap calcuatePath() {
        computeShortestPath();
        ArrayList<DLiteNode> nodePath = getPath();
        return createPathFieldMap(nodePath);
    }

    /**
     * Creates a space array containing a path
     * Adds Space types representing the path to the space map
     * @param path the path to generate a space array for
     * @return a space array with the path drawn on
     */
    @Nullable
    private SpaceMap createPathFieldMap(@NonNull ArrayList<DLiteNode> path) {
        if (path.size() == 0) {
            return null;
        }
        SpaceMap newMap = new SpaceMap(spaceMap);
        for (DLiteNode node : path) {
            int[] mxCoords = node.getMatrixCoords();
            newMap.setSpace(mxCoords, SpaceMap.Space.PF_PATH);
        }

        DLiteNode first = path.get(0);
        int[] firstCoords = first.getMatrixCoords();
        newMap.setSpace(firstCoords, SpaceMap.Space.PF_START);

        DLiteNode last = path.get(path.size()-1);
        int[] lastCoords = last.getMatrixCoords();
        newMap.setSpace(lastCoords, SpaceMap.Space.PF_END);

        return newMap;
    }

    /**
     * Get the path from the end node to the start node
     * @return an arraylist of nodes comprising the path. arraylist length will be 0
     * if no path was found
     */
    @NonNull
    private ArrayList<DLiteNode> getPath() {
        ArrayList<DLiteNode> pathNodes = new ArrayList<>();

        DLiteNode childNode = startNode.getChildNode();
        // if start nodes child node is null, then there's no path found
        if (childNode != null) {
            pathNodes.add(startNode);

            while (childNode != null) {
                pathNodes.add(childNode);
                childNode = childNode.getChildNode();
            }
        } else {
            Log.d(TAG, "no startNode child node");
        }
        return pathNodes;
    }

    /**
     * Update current position of the robot
     * Sets the start node to the node at the robots new coords
     * creates a new node if there's no node there already
     * @param robotCoords the robot's coordinates
     */
    private void updatePosition(@Nullable @MatrixCoordinates int[] robotCoords) {
        if (robotCoords != null) {
            startNode = getNode(robotCoords, null, null);
        }
    }

    /**
     * Update the path with a new map
     * @param newMap the new spacemap (doesnt have to be deepcopy)
     * @param robotCoords the new robot coordinates, if any
     * @return the new SpaceMap with the path if it was updated, if not, then the old spacemap
     * if no spacemap was previously calculated, or if no path could be found, then null is returned
     */
    @Nullable
    public SpaceMap update(SpaceMap newMap, @Nullable @MatrixCoordinates int[] robotCoords) {
        updatePosition(robotCoords);

        // find differences, and if there's no differences return null
        ArrayList<int[]> diffCoords = spaceMap.getDifferences(newMap, true);
        Log.d(TAG, "Differences: " + diffCoords.size());
        spaceMap = new SpaceMap(newMap);
        // if theres no updates and the start coord hasnt changed
        if (diffCoords.size() == 0 && startNode.equals(lastNode) && startNode.getChildNode() != null) {
            ArrayList<DLiteNode> nodePath = getPath();
            return createPathFieldMap(nodePath);
        }
        // update keyMin to maintain priority queue and update lastNode
        keyMin += lastNode.hDist(startNode); // IM SORRRY
        lastNode = startNode;

        // change all altered nodes
        for (int[] coords : diffCoords) {
            // switch the space
            DLiteNode changedNode = getNode(coords, null, startNode);
            changedNode.changeSpace(newMap.getSpace(coords));
            // update all changed edges of all affected nodes
            for (DLiteNode affectedNode : getNeighborNodes(changedNode)) {
                updateChangedEdge(changedNode, affectedNode);

            }
        }
        // recalculate shortest route
        return calcuatePath();

    }

    /**
     * Update a changed edge in both directions
     * The order of the parameters doesnt matter since theyre changed in both directions
     *
     * To update a changed edge, we essentially look at both the source and destination node
     * and see how the cost between them has changed.
     *
     * If the cost to the destination increased, we check to see if there's another node that
     * the source can use as a path to the target
     * If the cost decreased, we see if the change has presented the source with a more favorable path
     *
     * @param sourceNode the source node
     * @param destNode the destination node
     */
    private void updateChangedEdge(@NonNull DLiteNode sourceNode, @NonNull DLiteNode destNode) {
        // we need to do this both ways, so loop twice and switch source and dest at the end
        for (int i = 0; i < 2; i++) {
            // get the old and new cost of the transition
            // old cost is before any changes, so using the old spacemap's values
            int c_old = sourceNode.old_cost(destNode);
            int c_new = sourceNode.cost(destNode);

            if (!sourceNode.equals(endNode)) {
                // if the old cost is greater than the new cost, then
                // try changing the source's child to the destination since its a better path
                if (c_old > c_new) {
                    sourceNode.changeChildIfLower(destNode);
//                    Log.d(TAG, String.format("old cost: %s, new cost: %s", c_old, c_new));
                } else if (sourceNode.getRhs() == c_old + destNode.get_gDist()) {
                    // otherwise, if the source's rhs is the same as the old rhs between the two nodes,
                    // try and find a new node to make the new child
                    //@// FIXME: 8/9/21 this is essentally checking if the source node's child is the
                    // destination node, so try replacing this with a child check

                    for (DLiteNode neighborNode : getNeighborNodes(sourceNode)) {
                        sourceNode.changeChildIfLower(neighborNode);
//                        Log.d(TAG, String.format("old cost: %s, new cost: %s", c_old, c_new));
                    }
                }
            }
            // update the nodes' status on the priority queue
            UpdateVertex(sourceNode);
            // swap nodes
            DLiteNode tempNode = sourceNode;
            sourceNode = destNode;
            destNode = tempNode;
        }
    }




    /**
     * Compute the shortest path between the start node and end node using the information available
     *
     * Essentially, we continue computations while the start node is inconsistent or unexplored
     * We get the next node from the priority queue and ensure its key is accurate
     * Then, we determine if the node is consistent or inconsistent
     * If the node is consistent, we expand to its neighbors
     * Otherwise, we recalculate its rhs value and that of its neighbors to ensure their consistency
     */
    private void computeShortestPath() {
        // while the next key has a higher priority than the start key
        // or the start node is inconsistent
        // aka while the start node is inconsistent or unexplored

        int stuckCounter = 0;
        int lastListSize = 0;
        while (startNode.updateKey(startNode).compareTo(Collections.min(openList)) >= 0 ||
                startNode.getRhs() > startNode.get_gDist() || startNode.getChildNode() == null) {


            // get next node from priority queue, ie the one with the lowest key
            DLiteNode current = Collections.min(openList);
            // store the old key and recalculate the key
            int[] keyOld = current.getKey();
            int[] keyNew = current.calculateKey(startNode);

            // if the new key is greater than the old key,
            // update the node with the new key
            if (compareKeys(keyNew, keyOld) > 0) {
                current.updateKey(startNode);
            } else if (current.get_gDist() > current.getRhs()) {
                handleConsistent(current);
            } else {
                handleInconsistent(current);
            }
            // if the open list has been the same size for tolerance iterations, assume we're stuck
            if (openList.size() == lastListSize) {
                if (stuckCounter++ > tolerance) {
                    startNode.setChild(null); // a null startNode child indicates no solution
                    Log.d(TAG, "Assuming no solution, breaking out of loop");
                    break;
                }
            } else {
                stuckCounter = 0;
                lastListSize = openList.size();
            }
        }
        DLiteNode lowestNode = Collections.min(openList);
        Log.d(TAG, String.format("start rhs: %s, start g_dist %s", lowestNode.getRhs(), lowestNode.get_gDist()));
        Log.d(TAG, String.format("start key: {%s, %s}", startNode.getKey()[0], startNode.getKey()[1]));

        Log.d(TAG, String.format("lowest key: {%s, %s}", lowestNode.getKey()[0], lowestNode.getKey()[1]));
        Log.d(TAG, String.format("open list size: %s", openList.size()));

    }

    /**
     * Handle what happens if the node is determined to be consistent
     *
     * If a node is consistent, this indicates that its rhs is an accurate measure of its
     * distance to the goal, and therefore, we can expand off of it
     *
     * To do this, we loop through all neighbors and change the neighbor's child to the current
     * node if it presents the best path
     *
     * We then update the neighbor's position in the priority queue, adding it if necessary
     *
     * @param current the current node to handle consistencies for
     */
    private void handleConsistent(@NonNull DLiteNode current) {
        // otherwise, if the node is over-consistent (g_dist > rhs)
        // set g_dist to rhs and remove node from open list
        current.set_gDist(current.getRhs());
        openList.remove(current);
        // loop through all successors (neighbors) of current node
        for (DLiteNode neighborNode : getNeighborNodes(current)) {
            // if the node isnt the end node, change the neighbor's child to the current
            // node if it presents a more favorable (lower cost) path
            if (!neighborNode.equals(endNode)) {
                neighborNode.changeChildIfLower(current);
            }
            // update the neighbor's state in the open list
            UpdateVertex(neighborNode);
        }
    }

    /**
     * Handle what happens if the node is determined to be inconsistent
     * If the node is inconsistent, that means rhs > g_dist,
     * meaning that the node had a rhs and g_dist value, but the rhs was changed,
     * meaning the node has changed state since being calculated and needs to be reevaluated
     *
     * An inconsistent node has an rhs value that isn't trustworthy, so we need to recalculate its
     * rhs and that of its neighbors
     *
     * If this is the case, we set the g_dist to infinity, so the next time the consistency is
     * checked, it will be consistent
     *
     * We also look at all neighbors of the current node, including the node itself,
     * and check if their rhs is the same as the currently calculated cost of moving between
     * the two nodes + the current node's previous g_dist
     *
     * If this is the case, then we need to check and make sure there isn't a better path around
     * the node, since the path they were on has become inconsistent
     * To do this, we loop through their potential children and change the child if a better path
     * becomes available
     *
     * Then, we update the node's state in the priority queue
     *
     * @param current the node to handle inconsistencies for
     */
    private void handleInconsistent(@NonNull DLiteNode current) {
        // if the node is under-consistent (rhs > g_dist)
        // set the g_dist to infinity(ish)
        int g_old = current.get_gDist();
        current.set_gDist(DLiteNode.blocked_cost);
        // get all neighbor nodes of the current node, including the current node
        ArrayList<DLiteNode> nodeList = getNeighborNodes(current);
        nodeList.add(current);
        // loop through all of these nodes,
        // and if their rhs is the same as the rhs value before changing current's g_dist,
        // then they need to be updated
        for (DLiteNode cNode : nodeList) {
            if (cNode.getRhs() == cNode.cost(current) + g_old) {
                // if the current node isnt the end node, change the node's child to one of
                // the potential children around it
                if (!cNode.equals(endNode)) {
                    for (DLiteNode potentialChild : getNeighborNodes(cNode)) {
                        cNode.changeChildIfLower(potentialChild);
                    }
                }
                // if the node becomes inconsistent, add it to the open list
                UpdateVertex(cNode);
            }
        }
    }

    /**
     * Get all neighbor nodes of the current node
     * If the neighbor doesnt exist yet, create it
     * @param current the node to get neighbors for
     * @return an arraylist of the neighbor nodes
     */
    @NonNull
    private ArrayList<DLiteNode> getNeighborNodes(@NonNull DLiteNode current) {
        ArrayList<DLiteNode> nNodes = new ArrayList<>();
        for (int[] neighborVal : DLiteNode.neighborIncrements) {
            int[] nCoords = current.calculateNeighborCoordinate(neighborVal);

            // gotta be within bounds
            if ((nCoords[0] < nodeMap.length)
                    && (nCoords[1] < nodeMap.length)
                    && (nCoords[0] >= 0)
                    && (nCoords[1] >= 0)) {
                DLiteNode neighborNode = getNode(nCoords, current, startNode);
                nNodes.add(neighborNode);
            }
//            else {
//                Log.d(TAG, String.format("Coords {%s, %s} out of bounds", nCoords[0], nCoords[1]));
//            }

        }
        return nNodes;
    }

    /**
     * Updates a node's position in the openList priority queue
     * @param node the node to update the position of
     */
    private void UpdateVertex(@NonNull DLiteNode node) {
        if (node.get_gDist() != node.getRhs() && openList.contains(node)) {
            node.updateKey(startNode);

        } else if (node.get_gDist() != node.getRhs() && !openList.contains(node)) {
            openList.add(node.updateKey(startNode));

        } else if (node.get_gDist() == node.getRhs()) {
            openList.remove(node);
        }
    }

    /**
     * Gets a node from the nodemap, creating it if necessary
     * @param matrixCoords the coordinates of the node on the nodemap. Must be within bounds
     * @return the node
     */
    @NonNull
    public DLiteNode getNode(@NonNull @MatrixCoordinates int[] matrixCoords,
                             @Nullable DLiteNode childNode,
                             @Nullable DLiteNode startNode) {
        DLiteNode node = nodeMap[matrixCoords[0]][matrixCoords[1]];
        if (node == null) {
            node = new DLiteNode(matrixCoords, spaceMap.getSpace(matrixCoords), startNode, childNode);
        }
        nodeMap[matrixCoords[0]][matrixCoords[1]] = node;
        return node;
    }





    /**
     * Compares two sets of keys
     * @param key1 the first key to compare
     * @param key2 the second key to compare
     * @return > 0 if key1 is greater, <= 0 if key2 is greater
     */
    protected static int compareKeys(@NonNull int[] key1, @NonNull int[] key2) {
        int val1 = key1[0] - key2[0];
        if (val1 == 0) {
            return key1[1] - key2[1];
        }
        return val1;
    }

}
