package pathfinding;

import android.graphics.Color;

import androidx.annotation.ColorInt;
import androidx.annotation.NonNull;

import java.util.ArrayList;
import java.util.Arrays;

import annotations.AnyCoordinateRange;
import annotations.MatrixCoordinates;

/**
 * The ftc field can be represented as a set of squares of a fixed size containing various
 * types of spaces.
 * The SpaceMap class is a way of organizing this map and making various operations
 * simpler, faster, and more consistent.
 */
public class SpaceMap {

    private final Space[][] spaceMap;
    public final int height;
    public final int width;
    private final int minRange;
    private final int maxRange;

    /**
     * Create new spacemap with specified width and height and fill it with clear spots
     * @param xHeight the height of the spacemap
     * @param yWidth the width of the spacemap
     */
    public SpaceMap(int xHeight, int yWidth) {
        spaceMap = new Space[xHeight][yWidth];
        height = xHeight;
        width = yWidth;
        this.minRange = 0;
        this.maxRange = xHeight-1;
        // fill the array with clear spots
        for(Space[] row : spaceMap) {
            Arrays.fill(row, Space.CLEAR);
        }
    }

    /**
     * Create new spacemap with specified width and height and fill it with clear spots
     * Also provide a minimum and maximum range value to round coordinates down to
     * @param xHeight the height of the spacemap
     * @param yWidth the width of the spacemap
     */
    public SpaceMap(int xHeight, int yWidth, int minRange, int maxRange) {
        spaceMap = new Space[xHeight][yWidth];
        height = xHeight;
        width = yWidth;
        minRange = Math.max(0, minRange);
        maxRange = Math.min(height-1, maxRange);
        this.minRange = minRange;
        this.maxRange = maxRange;

        // fill the array with clear spots
        for(Space[] row : spaceMap) {
            Arrays.fill(row, Space.CLEAR);
        }
    }


    /**
     * Create a new spacemap as a deep copy of another spacemap
     * @param otherMap the spacemap to deepcopy
     */
    public SpaceMap(@NonNull SpaceMap otherMap) {
        this.spaceMap = getSpaceMapDC(otherMap.spaceMap);
        height = otherMap.height;
        width = otherMap.width;
        minRange = otherMap.minRange;
        maxRange = otherMap.maxRange;
    }


    /**
     * Fill a row of the spacemap with a space
     * @param row the row to fill
     * @param space the space to fill with
     */
    public void fillRow(int row, @NonNull Space space) {
        if (row >= 0 && row < height) // make sure its in bounds
            Arrays.fill(spaceMap[row], space);
    }

    /**
     * Fill a column of the spacemap with a space
     * @param column the column to fill
     * @param space the space to fill with
     */
    public void fillColumn(int column, @NonNull Space space) {
        if (column >= 0 && column < width) { // make sure its in bounds
            for(Space[] row : spaceMap) {
                row[column] = space;
            }
        }
    }

    @NonNull
    public Space[][] getRawMap() { return spaceMap; }

    /**
     * Find differences between two space maps
     * Compares local spacemap to new spacemap
     * @param otherMap The other spacemap to compare to
     * @param requireStateChange if true, only spaces where the passable status is changed are included
     * @return an arraylist of all different coordiantes
     */
    @NonNull
    @MatrixCoordinates
    public ArrayList<int[]> getDifferences(@NonNull SpaceMap otherMap, boolean requireStateChange) {
        ArrayList<int[]> diffList = new ArrayList<>();

        for(int r = 0; r < spaceMap.length; r++) {
            SpaceMap.Space[] cRow = spaceMap[r];
            SpaceMap.Space[] oRow = otherMap.spaceMap[r];
            if (!Arrays.equals(cRow, oRow)) {
                for(int c = 0; c < spaceMap[0].length; c++) {
                    if (!cRow[c].equals(oRow[c]) && // if theyre not equal
                            // and if a state change is required, then theyre different
                            (!requireStateChange || oRow[c].passable != cRow[c].passable)) {
                        diffList.add(new int[] {r, c});
                    }
                }
            }
        }
        return diffList;
    }

    /**
     * Set a coordinate to a space
     * @param coords the coordinates to replace
     * @param newSpace the new space to set
     * @return the value previously at that space
     */
    @NonNull
    public Space setSpace(@NonNull @MatrixCoordinates int[] coords, @NonNull Space newSpace) {
        Space oldSpace = getSpace(coords);
        spaceMap[coords[0]][coords[1]] = newSpace;
        return oldSpace;
    }

    /**
     * Get the space at a coordinate
     * @param coords the coordinate of the space to get, in matrix coordinates
     * @return the space at the coordiante
     */
    @NonNull
    public SpaceMap.Space getSpace(@NonNull @MatrixCoordinates int[] coords) {
        return spaceMap[coords[0]][coords[1]];
    }

    /**
     * Gets a deep copy of the space array
     * @return the a deep copy of the space array
     */
    @NonNull
    public SpaceMap.Space[][] getSpaceMapDC(@NonNull Space[][] spaceMap) {
        SpaceMap.Space[][] deepCopy = new SpaceMap.Space[spaceMap.length][];
        for (int r = 0; r < spaceMap.length; r++) {
            deepCopy[r] = spaceMap[r].clone();
        }
        return deepCopy;
    }

    /**
     * Rounds coordinates to within the spacemap's bounds
     * @param coords the coordinates to round
     * @return an integer array of coordinates within the spacemaps bounds
     */
    @NonNull
    @MatrixCoordinates
    public int[] roundToBounds(@AnyCoordinateRange int[] coords) {
        int[] newCoords = (coords == null) ? new int[2] : coords.clone();
        // get median value from 0, field length, and xyPair
        newCoords[0] = Math.max(Math.min(height-1,0), Math.min(Math.max(height-1,0),newCoords[0]));
        newCoords[1] = Math.max(Math.min(width-1,0), Math.min(Math.max(width-1,0),newCoords[1]));
        return newCoords;
    }

    /**
     * Rounds coordinates to within the given range within the spacemap's range
     * If the provided range is out of bounds, it will be rounded down
     * @param coords the coordinates to round
     * @return an integer array of coordinates within the spacemaps range
     */
    @NonNull
    @MatrixCoordinates
    public int[] roundToRange(@AnyCoordinateRange int[] coords) {
        int[] newCoords = (coords == null) ? new int[2] : coords.clone();
        // get median value from max, min, and coordinates
        newCoords[0] = Math.max(Math.min(maxRange, minRange), Math.min(Math.max(maxRange, minRange),newCoords[0]));
        newCoords[1] = Math.max(Math.min(maxRange, minRange), Math.min(Math.max(maxRange, minRange),newCoords[1]));
        return newCoords;
    }


    /**
     * Spaces are types which represent each type of object on the map
     * Each space has a color which is only important for visualization
     * In pathfinding, the robot can move through certain spaces, but has to navigate around others
     * i.e it has to move around obstacles (obviously)
     */
    public enum Space {

        // the wall
        @ColorInt
        WALL(Color.BLACK, false, true),
        // the vuforia image targets
        @ColorInt
        IMAGE_TARGET(Color.CYAN, true, true),

        // clear spot
        @ColorInt
        CLEAR(Color.WHITE, true, false),
        // the robot
        @ColorInt
        ROBOT(Color.BLUE, true, false),
        // a field obstacle
        @ColorInt
        OBSTACLE(Color.BLACK, false, false),

        // a target location on the field
        @ColorInt
        TARGET_LOCATION(Color.GREEN, true, false),


        // pathfinding space values
        // only placed by pathfinding algorithms
        @ColorInt
        PF_PATH(Color.GRAY, true, false),
        @ColorInt
        PF_START(Color.BLUE, true, false),
        @ColorInt
        PF_END(Color.GREEN, true, false);


        private final int color;
        private final boolean passable;
        private final boolean isStaticSpace;

        /**
         * Initialize information for the space types
         * @param color the color that the space gets mapped to for visuals
         * @param passable whether the robot can navigate through the space or not
         */
        Space(int color, boolean passable, boolean isStaticSpace) {
            this.color = color;
            this.passable = passable;
            this.isStaticSpace = isStaticSpace;
        }

        public int getColor() {
            return this.color;
        }

        public boolean isPassable() {
            return this.passable;
        }

        public boolean isStatic() { return this.isStaticSpace; }

    }
}
