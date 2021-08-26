package pathfinding;


import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

import java.util.Arrays;

import annotations.MatrixCoordinates;
import localization.SpaceMap;

/**
 * A node in the A* pathfinding algorithm
 */
public class AStarNode implements Comparable<AStarNode> {

    private static final String TAG = "vuf.test.aStarNode";

    private static final int straight_cost = 10; // the cost of moving non-diagonally between squares
    private static final int diag_cost = 14; // the cost of moving diagonally between squares

    /*
     the values to change coordinates by to reach the neighboring nodes
     0 - x, 1 - y, 2 - cost to move in direction
     okay so you could just use the top row and add/subtract to get all values, but honestly
     this is just easier
    */
    public static final int[][] neighborIncrements = new int[][] {
            {0, 1}, {1, 0}, {1, 1}, {1, -1},
            {0, -1}, {-1, 0}, {-1, -1}, {-1, 1}
    };
    @NotNull
    private final SpaceMap.Space space;
    @NotNull @MatrixCoordinates
    private final int[] matrixCoords;

    // 0 indicates uncalculated
    // values cant ever actually be 0
    private int h_dist = 0; // heuristic distance to end node
    private int g_dist = 0; // distance to start node
    private int f_cost = 0; // g cost (distance to start node) + h cost (distance to end node)

    @Nullable
    private AStarNode parentNode;

    /**
     * A node representing a position in the A* algorithm's pathfinding map
     * In the whole algorithm, there will be two end nodes: the initial and the final
     *
     * The initial end node is the end node created as a placeholder in the beginning of the algorithm
     * to calculate h distances relative to, but the h and f costs cant be calculated because there isnt
     * a valid path yet. The initial end node is STRICTLY used for calculating h distances, nothing else.
     *
     * Later in the algorithm, when the path reaches the end node, a new one is created with the
     * necessary properties, which marks the end of the algorithm.
     *
     * @param space the space type of the node
     * @param matrixCoords the coordinates of the node in matrix form
     * @param endNode the end node
     * @param parentNode the nodes parent, null if the node is the start or end node
     */
    public AStarNode(@NotNull SpaceMap.Space space,
                     @NotNull @MatrixCoordinates int[] matrixCoords,
                     @Nullable AStarNode endNode,
                     @Nullable AStarNode parentNode) {
        this.space = space;
        this.matrixCoords = matrixCoords;
        this.parentNode = parentNode; // will be null if the node is the initial end node or start node
        boolean isInitialEndNode = false;
        // if no end node was provided, this node is the initial end node
        if (endNode == null) {
            endNode = this;
            isInitialEndNode = true;
        }

        if (isPassable()) {
            h_dist = hDist(endNode);
            if (parentNode != null) { // dont calculate g_dist or f_cost for initial end node or start node
                g_dist = gDist(parentNode);
                f_cost = fCost(parentNode);
            } else if (!isInitialEndNode) { // if the node is the start node, g_dist is 0
                g_dist = 0;
                f_cost = g_dist + h_dist;
            }
        }
    }

    /* COSTS */

    /* H_DIST */

    /**
     * Get h_dist of this Node
     * Uses octile distance function to calculate
     * h distance, or heuristic distance, is an estimation of the distance of one node
     * to another, usually the target node (in this case the start node)
     * @param otherNode the other node to calculate distance to
     * @return the hDist from this node to the otherNode
     */
    public int hDist(@NotNull AStarNode otherNode) { return octileDistance(this, otherNode); }


    /**
     * Calculate the octile distance between two nodes
     * More:
     * - https://theory.stanford.edu/~amitp/GameProgramming/Heuristics.html
     *
     * @param sourceNode the source node
     * @param destNode the destination node
     * @return the distance between the two, assuming 8 way motion on a grid map
     */
    private static int octileDistance(@NotNull AStarNode sourceNode, @NotNull AStarNode destNode) {
//        Log.d(TAG, String.format("source: {%s, %s}, %s", sourceNode.matrixCoords[0], sourceNode.matrixCoords[1], sourceNode.space));
//        Log.d(TAG, String.format("dest: {%s, %s}, %s", destNode.matrixCoords[0], destNode.matrixCoords[1], destNode.space));
        // get straight distance between both nodes
        int dx = Math.abs(sourceNode.matrixCoords[0] - destNode.matrixCoords[0]);
        int dy = Math.abs(sourceNode.matrixCoords[1] - destNode.matrixCoords[1]);
        // find the distance between the two nodes only moving cardinally,
        // then subtract the steps saved by moving diagonally
        // see link above for more
        return straight_cost * (dx + dy) + (diag_cost - 2 * straight_cost) * Math.min(dx, dy);
    }

    /* END H_DIST */

    /* G_DIST */

    /**
     * Calculate the g distance to another node
     * The g distance is the actual path distance to the start node, calculated
     * through the movement costs between previous nodes
     * @param otherNode the other node to calculate the g_distance with. If the node is null, then
     *                  the current node is presumed to be the start node, so its g distance is 0.
     *                  Ensure that the provided node is not the initial end node, because the end node's
     *                  parent node is also initially null
     * @return the g distance calculated with the provided node
     */
    private int gDist(@NotNull AStarNode otherNode) {
        return otherNode.g_dist + cost(otherNode);
    }

    /* END G_DIST */

    /* F_COST */

    /**
     * Calculate the f_cost with the provided node
     * The f_cost is the sum of the g distance and the h distance
     * @param otherNode the other node to calculate the f_cost with
     * @return the calculated f cost
     */
    private int fCost(@NotNull AStarNode otherNode) {
        return h_dist + gDist(otherNode);
    }

    /**
     * Changes this Node's parent to the provided potentialParent if the f_cost is lower
     * @param potentialParent the potential parent of the node
     * @return True if the parent was changed, otherwise false
     */
    public boolean changeParentIfLower(@NotNull AStarNode potentialParent) {
        // calculate potential f cost
        int new_f = fCost(potentialParent);

        // if the new f cost is lower,
        // make this potentialParent the new parent potentialParent and adjust costs
        if (new_f < f_cost) {
            parentNode = potentialParent;
            // f_cost = h_dist + g_dist
            g_dist = new_f - h_dist;
            this.f_cost = new_f;
            return true;
        }
        return false;
    }

    /* END F_COST */

    /* END COSTS */

    /**
     * Calculates the cost of moving between this node and another node
     * @param otherNode the other node to calculate the cost relative to
     * @return the cost of moving between nodes
     */
    public int cost(AStarNode otherNode) {
        // if its the same node, cost is 0
        if (this.equals(otherNode)) {
            return 0;
        }

        int dx = Math.abs(this.matrixCoords[0] - otherNode.matrixCoords[0]);
        int dy = Math.abs(this.matrixCoords[1] - otherNode.matrixCoords[1]);

        // if dx and dy are equal, its diagonal, otherwise its straight
        return (dx==dy) ? diag_cost : straight_cost;
    }

    /**
     * Get this node's parent node
     * Technically nullable if called on initial end node
     * @return the parent node
     */
    @Nullable
    public AStarNode getParentNode() {
        return parentNode;
    }

    /**
     * Calculate the neighbor coordinates for a set of neighbor increment values
     * @param neighborIncrement the neighbor increment to use
     * @return the neighbor coordinates
     */
    @NotNull
    @MatrixCoordinates
    public int[] calculateNeighborCoordinate(@NotNull int[] neighborIncrement) {
        int nX = matrixCoords[0] + neighborIncrement[0];
        int nY = matrixCoords[1] + neighborIncrement[1];
        return new int[] {nX, nY};
    }

    @NotNull
    @MatrixCoordinates
    public int[] getMatrixCoords() {return matrixCoords; }

    /**
     * Check if the node is passable
     * @return true if the node is passable, else false
     */
    public boolean isPassable() {
        return space.isPassable();
    }

    /**
     * Determine equality of two Node objects
     * Two Nodes are equal if they have the same matrix coordinates
     * @param obj the other object to compare
     * @return True if the objects are equal, otherwise false
     */
    @Override
    public boolean equals(@Nullable Object obj) {
        if (this == obj)
            return true;
        if (obj == null)
            return false;
        if (getClass() != obj.getClass())
            return false;
        AStarNode other = (AStarNode) obj;
        return (Arrays.equals(matrixCoords, other.matrixCoords));
    }


    /**
     * Compares two nodes by first comparing their f_cost, and if both f_costs are equal,
     * then the h_costs are compared
     * If h_costs are equal, then theyre assumed equal because it doesnt really matter
     * @param node the other node to compare
     * @return >1 if this node is greater, <1 if the other node is greater, 0 if equal
     */
    @Override
    public int compareTo(@NotNull AStarNode node) {
        int f_compare = this.f_cost - node.f_cost;
        if (f_compare == 0) {
            return (this.h_dist - node.h_dist > 0) ? 1 : -1;
        }
        return (f_compare > 0) ? 1 : -1;
    }

}
