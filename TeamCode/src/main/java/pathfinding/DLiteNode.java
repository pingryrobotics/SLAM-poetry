package pathfinding;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import java.util.Arrays;

import annotations.MatrixCoordinates;

/**
 * A node in the D* Lite pathfinding algorithm
 */
public class DLiteNode implements Comparable<DLiteNode> {

    private static final String TAG = "vuf.test.dLiteNode";
    private static final int straight_cost = 10;
    private static final int diag_cost = 14;
    public static final int blocked_cost = 1000000; // big boy number

    // is there a cleaner way to do this? maybe, but this is easier
    @NonNull
    public static final int[][] neighborIncrements = new int[][] {
            {0, 1}, {1, 0}, {0, -1}, {-1, 0}, // straight
            {1, 1}, {1, -1},{-1, -1}, {-1, 1} // diagonal
    };
    @Nullable
    private DLiteNode childNode;
    private int rhs;
    private int g_dist;
    private int[] key;
    @MatrixCoordinates
    private final int[] matrixCoords;
    @NonNull
    private SpaceMap.Space space;
    @Nullable
    private SpaceMap.Space oldSpace;

    /**
     * Constructor for D* Lite Node
     * @param matrixCoords the coordinates of the node
     * @param space the space type of the node
     * @param startNode the start node of the algorithm (kinda the end because d* lite)
     * @param childNode this nodes child node
     */
    public DLiteNode(@NonNull @MatrixCoordinates int[] matrixCoords,
                     @NonNull SpaceMap.Space space,
                     @Nullable DLiteNode startNode,
                     @Nullable DLiteNode childNode) {
        this.matrixCoords = matrixCoords;
        this.space = space;

        if (startNode == null) {
            startNode = this;

        }
        this.childNode = childNode;

        this.g_dist = blocked_cost;
        this.rhs = blocked_cost;

    }


    /* RHS */
    /**
     * Calculate rhs distance between two nodes
     * rhs distance is basically the true distance between the nodes, counted from the
     * goal node to this node
     *
     * If the other node has no g_dist, the rhs is infinite
     *
     * @param otherNode the other node to calculate rhs distance with
     * @return the rhs distance between two nodes
     */
    private int rhsDist(@NonNull DLiteNode otherNode) {
            return otherNode.g_dist + cost(otherNode);
    }
    public void setRhs(int rhs) { this.rhs = rhs; } /** Set rhs to provided value */
    public int getRhs() { return this.rhs; } /* Get current rhs */

    /**
     * Changes this Node's child to the provided potentialChild if the f_cost is lower
     * @param potentialChild the potential child of the node
     * @return True if the child was changed, otherwise false
     */
    public boolean changeChildIfLower(@NonNull DLiteNode potentialChild) {
        // calculate potential rhs
        int rhsNew = rhsDist(potentialChild);
        if (childNode != null) {
            rhs = rhsDist(childNode); // recalculate rhs
        } else {
            rhs = blocked_cost + blocked_cost;
        }

        // if the new rhs is lower, then
        // make this potentialChild the new child and adjust values
        if (rhsNew < rhs) {
            childNode = potentialChild;
            rhs = rhsNew;
            return true;
        }
        return false;
    }

    /* END RHS */



    /* G_DIST */
    public void set_gDist(int g_dist) { this.g_dist = g_dist; } /** Set the g_dist to provided value */
    public int get_gDist() { return g_dist; } /* Get the g_dist */
    /* END G_DIST */



    /* H_DIST */
    /**
     * Get h_dist of this Node
     * Uses octile distance function to calculate
     * h distance, or heuristic distance, is an estimation of the distance of one node
     * to another, usually the target node (in this case the start node)
     * @param otherNode the other node to calculate distance to
     * @return  the hDist from this node to the otherNode
     */
    public int hDist(@NonNull DLiteNode otherNode) { return octileDistance(this, otherNode); }

    /* END H_DIST */


    /* KEYS */
    public void setKey(@NonNull int[] key) { this.key = key; }
    @Nullable
    public int[] getKey() { return key; }

    /**
     * Updates the nodes key
     * @return the node that was updated
     */
    @NonNull
    public DLiteNode updateKey(@NonNull DLiteNode startNode) {
        this.key = calculateKey(startNode);
        return this;
    }

    /**
     * Calculates the node's key
     * @return the newly calculated key
     */
    @NonNull
    public int[] calculateKey(@NonNull DLiteNode startNode) {
        int[] key = new int[2];
        key[0] = Math.min(g_dist, rhs) + hDist(startNode) + DLite.keyMin;
        key[1] = Math.min(g_dist, rhs);
        return key;
    }

    /* END KEYS */


    /* CALCULATIONS */
    /**
     * Calculate the neighbor coordinates for a set of neighbor increment values
     * @param neighborIncrement the neighbor increment to use
     * @return the neighbor coordinates
     */
    @NonNull
    public int[] calculateNeighborCoordinate(@NonNull int[] neighborIncrement) {
        int nX = matrixCoords[0] + neighborIncrement[0];
        int nY = matrixCoords[1] + neighborIncrement[1];
        return new int[] {nX, nY};
    }


    /**
     * Calculates the cost of moving between this node and another node
     * @param otherNode the other node to calculate the cost relative to
     * @return the cost of moving between nodes
     */
    public int cost(@NonNull DLiteNode otherNode) {
        int dx = Math.abs(this.matrixCoords[0] - otherNode.matrixCoords[0]);
        int dy = Math.abs(this.matrixCoords[1] - otherNode.matrixCoords[1]);

        // if the either node isnt pasable or the nodes arent adjacent, cost is infinite
        if (dx+dy > 2 || !this.isPassable() || !otherNode.isPassable()) {
            return blocked_cost;
        } else if (this.equals(otherNode)) {
            return 0;
        }
        // if dx and dy are equal, its diagonal, otherwise its straight
        return (dx==dy) ? diag_cost : straight_cost;
    }

    /**
     * Calculate cost between this node and another node for both of their former values
     * If either node doesn't have an old space value, then the current one is used
     * @param otherNode the other node to calculate the former cost for
     * @return the former cost
     */
    public int old_cost(@NonNull DLiteNode otherNode) {
        int dx = Math.abs(this.matrixCoords[0] - otherNode.matrixCoords[0]);
        int dy = Math.abs(this.matrixCoords[1] - otherNode.matrixCoords[1]);

        SpaceMap.Space cSpace = (oldSpace == null) ? space : oldSpace;
        SpaceMap.Space oSpace = (otherNode.oldSpace == null) ? otherNode.space : otherNode.oldSpace;

        // if the either node isnt pasable or the nodes arent adjacent, cost is infinite
        //@// FIXME: 8/9/21 dx or dy could be 2
        if (dx+dy > 2 || !cSpace.isPassable() || !oSpace.isPassable()) {
            return blocked_cost;
        } else if (this.equals(otherNode)) {
            return 0;
        }
        // if dx and dy are equal, its diagonal, otherwise its straight
        return (dx==dy) ? diag_cost : straight_cost;
    }


    /**
     * Calculate the octile distance between two nodes
     * More:
     * - https://theory.stanford.edu/~amitp/GameProgramming/Heuristics.html
     *
     * @param sourceNode the source node
     * @param destNode the destination node
     * @return the distance between the two, assuming 8 way motion on a grid map
     */
    private static int octileDistance(@NonNull DLiteNode sourceNode, @NonNull DLiteNode destNode) {
        // get straight distance between both nodes
        int dx = Math.abs(sourceNode.matrixCoords[0] - destNode.matrixCoords[0]);
        int dy = Math.abs(sourceNode.matrixCoords[1] - destNode.matrixCoords[1]);
        // find the distance between the two nodes only moving cardinally,
        // then subtract the steps saved by moving diagonally
        // see link above for more
        return straight_cost * (dx + dy) + (diag_cost - (2 * straight_cost)) * Math.min(dx, dy);
    }

    /* END CALCULATIONS */


    /* MINOR ACCESSORS AND MODIFIERS */

    /**
     * Change the nodes space to a new space and store the old one in oldSpace
     * @param space the new space
     */
    public void changeSpace(@NonNull SpaceMap.Space space) {
        this.oldSpace = this.space;
        this.space = space;
    }

    /**
     * Clear the old stored space once the map has been updated
     */
    public void clearOldSpace() { this.oldSpace = null; }

    @Nullable
    public DLiteNode getChildNode() { return childNode; }

    @NonNull
    public int[] getMatrixCoords() { return matrixCoords; }

    /**
     * Check if the node is passable
     * @return true if the node is passable, else false
     */
    public boolean isPassable() {
        return space.isPassable();
    }

    // for testing
    public void setChild(@Nullable DLiteNode child) {
        this.childNode = child;
    }

    /* END MINOR ACCESSORS AND MODIFIERS */

    /* OVERRIDES */

    /**
     * Check if two nodes are equal
     * Theyre equal if both matrix coordinates are the same
     * @param obj the other object to compare
     * @return true if equal, otherwise false
     */
    @Override
    public boolean equals(@Nullable Object obj) {
        if (this == obj)
            return true;
        if (obj == null)
            return false;
        if (getClass() != obj.getClass())
            return false;
        DLiteNode other = (DLiteNode) obj;
        return (Arrays.equals(matrixCoords, other.matrixCoords));
    }


    /**
     * Compare two nodes by comparing their keys
     * First, the first keys are compared. If neither is even, compare the second pair
     * if those are even, then choose randomly
     * @param node the other node to compare
     * @return > 0 if this node is greater, <= 0 if the other node is greater
     */
    @Override
    public int compareTo(@NonNull DLiteNode node) {
        return DLite.compareKeys(this.key, node.key);
    }

    /* END OVERRIDES */
}
