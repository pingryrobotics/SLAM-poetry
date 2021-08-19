package pathfinding;


import android.graphics.RectF;
import android.util.Log;
import android.util.Pair;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.util.ArrayList;
import java.util.Hashtable;
import java.util.List;
import java.util.NoSuchElementException;

import annotations.DistanceValues;
import annotations.FieldCoordinates;
import annotations.ImageCoordinates;
import annotations.MatrixCoordinates;
import tf_detection.Detection;
import tf_detection.TFManager;

import static pathfinding.SpaceMap.Space;

/**
 * The FieldMap is how the FTC field will be visualized for pathfinding purposes
 * Essentially, the field is a coordinate system with each space being filled by one of a few things
 * i.e the robot, a clear spot, the walls, a random obstacle
 *
 * To navigate around the field, we can map out what each space is filled by and then
 * use this map to create a path from the robot's current location to its target position
 *
 * This map works in conjunction with vuforia's coordinate system in order to create the map
 * and plot updated positions
 *
 *
 * @ TODO: 8/8/21 error handling and prevention, keep track of null values to prevent those errors
 * @ FIXME: 8/12/21 There should only ever be one robot position space at once
 *
 * Nulls:
 * Hashtable keys with no items in their ArrayList should have the key removed, thereby making the
 * retrieval value null
 *
 *
 */

/*
    Optimizations:
        - D* Lite if A* is too slow
        - pixel per angle calibration for better distance measurements (or turn camera sideways)
        - custom tflite model (https://www.tensorflow.org/lite/inference_with_metadata/task_library/object_detector)
            if provided tflite is trash, and also for robot and image target detection
        - storage -> ram allocation (only if necessary, this would be extremely annoying to do tho)
        - angle camera downwards because top half is useless

    Future ideas:
          - tensorflow and vuforia should work together to constantly keep track of the robot's
            position. if vuforia knows where the robot is, then tf can go and find objects and whatnot.
            however, if vf loses track of the robot's position, tf should try and recognize
            image targets, and if it finds one, it should alert vf, which should then zoom in on
            the target to attempt localization.
          - tf should be able to access cameras independently of vuforia, but they cant access
            the same one at the same time. vuforia can use a different camera calibration
            which zooms in the image (possibly) for better recognitions. Either way, we need to
            find a way to get vuforia to be able to recognize images from farther distances
          - the robot knows its own heading because it has a sensor or something in the control hub.
            Therefore, if we can use that in conjunction with the motor encoder, then we could
            potentially figure out how far and in which direction the robot has travelled, thereby
            decreasing our dependency on vuforia for localization.
 */
public class FieldMap {

    static final File captureDirectory = AppUtil.ROBOT_DATA_DIR;
    static final String TAG = "vuf.test.fieldmap";

    // amount to extend reverse recognition image coordinates rects by
    // see isObjectGone
    private static final int pxTolerance = 50;

    private static final int scale = 75; // value to scale the field values down by
    // 10 is good for mm to cm for testing, 75 is nice for actual field mm to a managable size (~48x48)

    // the x and y dimensions of the field are the same, so we have fieldSize
    private final int fieldSize;

    // the amount that x and y field coordiantes are transformed for matrix positioning
    private final int fieldTransform;

    private DLite dLite;
    private SpaceMap spaceMap;
    private final TFManager tfManager;
    private OpenGLMatrix robotPositionGL;

    /**
     * Dynamic filled spaces are spaces filled with spots such as the robot, a temporary obstacle,
     * or any other value that is subject to change.
     * Dynamic coordinates can be added, removed, and overridden at any time
     */
    @MatrixCoordinates
    private final Hashtable<Space, ArrayList<int[]>> dynamicFilledSpaces = new Hashtable<>();
    /**
     * Static filled spaces are completely static spots on the field that do not change for
     * any reason. Examples include walls, image targets, stationary game objects, etc
     * Static fills cannot be removed or overridden by any function after being placed
     * However, during placement, coordinates with conflicts may be overridden
     *
     * Static fills should only be added ONCE, during initialization, to ensure
     * continuity
     */
    @MatrixCoordinates
    private final Hashtable<Space, ArrayList<int[]>> staticFilledSpaces = new Hashtable<>();

    /**
     * When field positions are calculated from tensorflow recognitions, they're placed
     * onto the spacemap, but in that process, the exact location is lost through the conversion
     * between GL -> matrix coordinates. This is an issue, because when we check if a certain
     * recognition is still present on the field, we use its field position to do so. We could
     * just convert from matrix back to field, but then we're working with a less precise metric
     * and therefore will have a less precise answer. So, instead, we maintain a list of
     * the recognition's field positions along with their matrix position so we can check
     * when recognitions disappear.
     */
    @FieldCoordinates
    private final Hashtable<int[], OpenGLMatrix> recognitionFieldPositions = new Hashtable<>();

    // region initialization
    /**
     * Initialize the fieldmap with FTC field info
     * For reference on x and y height, see the ftc coordinate system:
     * https://acmerobotics.github.io/ftc-dashboard/official_field_coord_sys.pdf
     *
     * Essentially, y extends outwards from the red wall, and x runs parallel to the red wall
     *
     * The static spaces should be a hashtable with a type of space as the key, and all
     * positions on the field with that specific space. The positions should can provided as type
     * OpenGLMatrix to facilitate use with Vuforia.
     * Static spaces include targets, any unmoving field obstacles or important positions, but
     * not the robot itself or the walls
     * Once static coordinates are placed, they cannot be removed or overridden by any function.
     *
     * If one or both of the coordinate formats arent used, use null
     *
     * @note integer coordinates should be inputted as *field coordinates*. They are translated to
     * matrix coordinates internally
     *
     * @note for faster computation, millimeters are scaled by the scale value
     *
     * @param fieldSizeMM The height and width of the field, in real millimeters
     * @param staticCoordsGL The static spaces on the field as OpenGLMatrix coordinates. Can be null
     * @param staticCoordsInt The static spaces on the field as int array coordinates. Can be null
     * @param tfManager the tensorflow manager to use with the fieldmap.
     */
    public FieldMap(
            int fieldSizeMM,
            @Nullable @FieldCoordinates
            Hashtable<Space, ArrayList<OpenGLMatrix>> staticCoordsGL,
            @Nullable @FieldCoordinates
            Hashtable<Space, ArrayList<int[]>> staticCoordsInt,
            @NonNull TFManager tfManager) {
        // convert mm to cm
        this.fieldSize = fieldSizeMM / scale;
        this.fieldTransform = fieldSizeMM/2;
        this.tfManager = tfManager;
        initialzeFieldMap();
        initializeStaticFills(staticCoordsGL, staticCoordsInt);
    }

    /**
     * Initializes all spots of the field map to clear, then adds walls
     * Also adds wall coordinates to static coordinate hashtable
     */
    private void initialzeFieldMap() {
        Log.d(TAG, "initializing field map");
        // create array of clear spaces with the height and width of the field
        spaceMap = new SpaceMap(fieldSize, fieldSize, fieldSize-2, 1);
        Space[][] spaceArray = spaceMap.getRawMap();

        ArrayList<int[]> wallCoords = new ArrayList<>();

        // first and last row
        spaceMap.fillRow(0, Space.WALL);
        spaceMap.fillRow(fieldSize-1, Space.WALL);

        // first and last column
        for (int r = 0; r < fieldSize; r++) {
            spaceArray[r][0] = Space.WALL;
            spaceArray[r][fieldSize-1] = Space.WALL;
            wallCoords.add(new int[] {0, r});
            wallCoords.add(new int[] {r, 0});
            wallCoords.add(new int[] {fieldSize-1, r});
            wallCoords.add(new int[] {r, fieldSize-1});
        }
        staticFilledSpaces.put(Space.WALL, wallCoords);
        Log.d(TAG, "Field map initialized");

    }

    /**
     * Adds static spots to the field map from OpenGLMatrix and integer coordinates,
     * transforms x and y coordinates from field coordinates to matrix coordinates,
     * and sets staticFilledSpaces to the transformed and complete space/int coord hashtable
     * for all values except WALL, which is added previously
     */
    private void initializeStaticFills(
            @Nullable @FieldCoordinates
            Hashtable<Space, ArrayList<OpenGLMatrix>> coordsGL,
            @Nullable @FieldCoordinates
            Hashtable<Space, ArrayList<int[]>> coordsInt) {
        Log.d(TAG, "Initializing static fills");
        Hashtable<Space, ArrayList<int[]>> joinedCoords =
                CoordinateUtils.joinCoordinateHashtables(coordsInt, coordsGL);
        Log.d(TAG, "Adding statics to spaceArray");
        // loop through all spaces in the coordsInt hashtable
        for (Space space : joinedCoords.keySet()) {
            // x is first, y is second, so take those coords and set the corresponding coords in the spaceArray
            for (int[] xyPair : joinedCoords.get(space)) {
                // x and y coordiantes are transformed for accurate positioning
                // dont round to range because image targets are on walls
                xyPair = fieldToMatrix(xyPair, false);
                Log.d(TAG, String.format("x: %s, y: %s", xyPair[0], xyPair[1]));
                // replace the space
                Space overriddenSpace = spaceMap.setSpace(xyPair, space);
                staticFilledSpaces.put(space, joinedCoords.get(space));
                // if the overridden space was static, remove the overridden coordinates from
                // the hashtable
                if (overriddenSpace.isStatic()) {
                    staticFilledSpaces.get(overriddenSpace).remove(xyPair);
                }

            }
        }
        Log.d(TAG, "Finished initializing static fills");
    }

    // endregion initialization

    // region recognition mapping

    /**
     * Updates the dynamic positions based on tensorflow recognitions
     * Gets the positions of new recognitions and places them on the spacemap
     * Also looks at all recognition field positions and checks if they're still present
     */
    public void updateDynamicPositions() {
        // long boi type
        Pair<Hashtable<Space, ArrayList<int[]>>, Hashtable<int[], OpenGLMatrix>>
                recognitionPositions = getRecognitionPositions();

        Hashtable<Space, ArrayList<int[]>> matrixPositions = recognitionPositions.first;
        Hashtable<int[], OpenGLMatrix> recFieldPositions = recognitionPositions.second;
        recognitionFieldPositions.putAll(recFieldPositions);

        addDynamic(null, matrixPositions, false);
        checkDisappearances();
    }

    /**
     * Checks the recognition field positions list to see if any recognitions have disappeared
     * from their previous position. Essentially, we look at each camera and check the pixels where
     * the recognitions would be on their view. If there's a recognition blocking the pixel,
     * then we can't confirm if its there or not, so we assume it is. If there's nothing there,
     * we know its gone and clear it from the dynamic list.
     */
    public void checkDisappearances() {
        // get float rectangles for all the recognitions on screen
        ArrayList<RectF> recognitionPixels = new ArrayList<>();
        for (Detection recognition : tfManager.getLatestDetections()) {
            recognitionPixels.add(new RectF(recognition.getLeft(),
                    recognition.getTop(), recognition.getRight(), recognition.getBottom()));
        }

        ArrayList<int[]> goneList = new ArrayList<>();

        for (int[] intCoords : recognitionFieldPositions.keySet()) {
            OpenGLMatrix fieldPosition = recognitionFieldPositions.get(intCoords);
            Log.d(TAG, "Checking position: " + VuforiaManager.format(fieldPosition));
            if (isObjectGone(robotPositionGL, fieldPosition, recognitionPixels)) {
                goneList.add(intCoords);
                Log.d(TAG, "Removing position");
            }

        }
        // cleardyn also removes from recognition list so we dont need to worry about it
        clearDynamic(goneList);
    }

    /**
     * Check if an object previously recognized through tensorflow has disappeared
     * @param cameraPosition the position of the camera to check
     * @param fieldPosition the position on the field of the object
     * @param recognitionPixels the list of recognition image coordinates
     * @return true if the object is gone, false if it isnt or if it can't be determined
     */
    private boolean isObjectGone(@NonNull @FieldCoordinates OpenGLMatrix cameraPosition,
                                 @NonNull @FieldCoordinates OpenGLMatrix fieldPosition,
                                 @NonNull @ImageCoordinates ArrayList<RectF> recognitionPixels) {

        float[] imageCoords = getImageCoordsFromFieldPosition(cameraPosition, fieldPosition);



        /*
            FIXME: 8/14/21 sometimes, recognitions have out of bounds coordinates, so the object
                           image coordinates end up out of bounds get get ignored. This might be
                           okay, since we should only be removing if we're pretty sure its not there,
                           but this could also be a source of bugs. One option is to round all
                           recognition coordinates so they're within range, but that might degrade
                           positioning accuracy.
         */
        if (imageCoords == null) {
            Log.d(TAG, "Object is not in camera view");
            return false;
        }
        // make a box around the image coordinates so there's some leeway
        // if the image coordinates arent found, then theyre not in the camera's view
        float top = Math.max(imageCoords[0]-pxTolerance, 0);
        float bottom = (float) Math.min(imageCoords[0]+pxTolerance, tfManager.imageHeight);
        float left = Math.max(imageCoords[1]-pxTolerance, 0);
        float right = (float) Math.min(imageCoords[1]+pxTolerance, tfManager.imageWidth);
        RectF posRect = new RectF(left, top, right, bottom);

        float posArea = posRect.width() * posRect.height();
        Log.d(TAG, String.format("Image coordinates {vert, horz}: {%s, %s}", imageCoords[0], imageCoords[1]));

        // if the object's coordinates are obscured by a recognition, then we dont know if its
        // there or not. It could also BE the recognition, so either way it definitively gone
        for (RectF recRect : recognitionPixels) {
            Log.d(TAG, "Object box: " + CoordinateUtils.rectToString(posRect));
            Log.d(TAG, "Recognition box: " + CoordinateUtils.rectToString(recRect));
            float recArea = recRect.width() * recRect.height();
            if (recRect.intersect(posRect)) {
                /*
                If we find that too many objects are being falsely occluded, then we can
                limit the definition of 'occlusion' to above a certain iou
                 */
                float intersectArea = recRect.width() * recRect.height();
                float unionArea = recArea + posArea - intersectArea;
                Log.d(TAG, "Intersect: " + intersectArea);
                Log.d(TAG, "Union: " + unionArea);
                Log.d(TAG, "Intersect over union: " + intersectArea/unionArea);
                Log.d(TAG, "Object is occluded");
                return false;
            }
        }
        // if we can confirm that it isnt covered by a recognition, but the location
        // where it would be is on screen, then its not there
        Log.d(TAG, "Object is gone");
        return true;
    }

    /**
     * Gets the image coordinates of a field position on a specific camera
     * @param cameraPosition the position of the camera
     * @param fieldPosition the field position to find
     * @return if the image coordinates exist on the camera, then returns them, otherwise null
     * image coordinates are in form {vertical px, horizontal px} because it corresponds to the
     * {straight, side} format that's been used throughout
     */
    @Nullable
    @ImageCoordinates
    private float[] getImageCoordsFromFieldPosition(
            @NonNull @FieldCoordinates OpenGLMatrix cameraPosition,
            @NonNull @FieldCoordinates OpenGLMatrix fieldPosition) {
        float[] distances = CoordinateUtils.calculateDistanceToCamera(cameraPosition, fieldPosition);
        return tfManager.getPixelCoordinatesFromDistances(distances);
    }

    /**
     * Get the position of elements on the field map and map them at their absolute location
     * Rotates the recognitions around the robot to correctly place them on the map so they're
     * position corresponds to the direction the robot was facing when it recognized them
     * Also gets the field coordinates for the recognitions to make confirming locations at later
     * points easier and more accurate
     *
     * @ FIXME: 8/13/21 sort of hardcoded with labels for testing
     * @return a pair of coordinate sets. The first is a hashtable of spaces and their corresponding
     * matrix coordinates, and the second is the matrix coordinates and their OpenGLMatrix coordinates.
     * The GL positions are maintained to reduce loss when converting from GL to matrix.
     */
    @NonNull
    @MatrixCoordinates
    @FieldCoordinates
    private Pair<Hashtable<Space, ArrayList<int[]>>, Hashtable<int[], OpenGLMatrix>> getRecognitionPositions() {
        // create hashtable for all recognitions and arraylists to store each recognition
        Hashtable<Space, ArrayList<int[]>> recognitionCoords = new Hashtable<>();
        ArrayList<int[]> goldCoords = new ArrayList<>();
        ArrayList<int[]> silverCoords = new ArrayList<>();
        // hashtable for matrix position and field position
        Hashtable<int[], OpenGLMatrix> fieldPositions = new Hashtable<>();

        // loop through all recognitions
        List<Detection> recognitionList = tfManager.getLatestDetections();
        for (Detection recognition : recognitionList) {

            float centerPx = (recognition.getLeft() + recognition.getRight())/2;
            Log.d(TAG, String.format("Recognition image coords {horz, vert}: {%s, %s}",
                    Math.round(centerPx), Math.round(recognition.getBottom())));

            // get their position on the field and on the spacemap
            OpenGLMatrix fieldPosition = CoordinateUtils.convertDistancesToFieldPosition(
                    robotPositionGL, tfManager.getDistances(recognition));
            int[] matrixPosition = fieldToMatrix(
                    CoordinateUtils.convertGLtoInt(fieldPosition), true);
            // add matrix pos and its corresponding field position
            fieldPositions.put(matrixPosition, fieldPosition);
            // sort into category
            if (recognition.getLabel().equals("Gold Mineral")) {
                goldCoords.add(matrixPosition);
            } else {
                silverCoords.add(matrixPosition);
            }
        }
        // only add if there's more than one
        if (goldCoords.size() > 0) {
            recognitionCoords.put(Space.TARGET_LOCATION, goldCoords);
        }
        if (silverCoords.size() > 0) {
            recognitionCoords.put(Space.OBSTACLE, silverCoords);
        }
        // pear
        return new Pair<>(recognitionCoords, fieldPositions);
    }




    // endregion recognition mapping


    // region set/add/clear dynamic

    /**
     * Set the robot's position and clears the old position
     * @param position the position as an OpenGLMatrix
     */
    public void setRobotPosition(@NonNull @FieldCoordinates OpenGLMatrix position) {
        robotPositionGL = new OpenGLMatrix(position); // copy because its safer
        setDynamic(Space.ROBOT, position, null, true);
    }


    /**
     * Resets fieldmap dynamic coordinates to the provided values
     * This method takes in hashtables representing the new coordinates for dynamic spaces
     * If a space is included within the hashtable, all other values for that space type are cleared,
     * then the new values are added
     *
     * If you want to add new values to a space, use addDynamic()
     * If you want to clear all dynamic coordinates, or those of a certain type, use clearDynamic()
     *  @param coordsGL the opengl coordinates to set
     * @param coordsInt integer coordinates to set
     * @param convertToMatrix if true, converts coordinates from field coordinates to matrix coordinates
     */
    public void setDynamic(@Nullable @FieldCoordinates @MatrixCoordinates Hashtable<Space, ArrayList<OpenGLMatrix>> coordsGL,
                           @Nullable @FieldCoordinates @MatrixCoordinates Hashtable<Space, ArrayList<int[]>> coordsInt,
                           boolean convertToMatrix) {

        Hashtable<Space, ArrayList<int[]>> coordsIntN =
                CoordinateUtils.joinCoordinateHashtables(coordsInt, coordsGL);

        for (Space space : coordsIntN.keySet()) {
            ArrayList<int[]> newIntCoords = coordsIntN.get(space);
            clearDynamic(space);
            addDynamic(space, newIntCoords, convertToMatrix);
            Log.d(TAG, String.format("Set %s values in %s", newIntCoords.size(), space.name()));
        }
    }


    /**
     * Set a list of coordinate to a space
     * @param space the space to set
     * @param coordsGL the GL coords to set,
     * @param coordsInt the int coords to set
     * @param convertToMatrix if true, converts coordinates from field coordinates to matrix coordinates
     */
    public void setDynamic(@NonNull Space space,
                           @Nullable @FieldCoordinates @MatrixCoordinates ArrayList<OpenGLMatrix> coordsGL,
                           @Nullable @FieldCoordinates @MatrixCoordinates ArrayList<int[]> coordsInt,
                           boolean convertToMatrix) {

        ArrayList<int[]> coordinates = CoordinateUtils.joinCoordinateArrayLists(coordsGL, coordsInt);
        clearDynamic(space);
        addDynamic(space, coordinates, convertToMatrix);
    }

    /**
     * Set a list of coordinate to a space
     * @param space the space to set
     * @param coordsGL the GL coords to set,
     * @param coordsInt the int coords to set
     * @param convertToMatrix if true, converts coordinates from field coordinates to matrix coordinates
     */
    public void setDynamic(@NonNull Space space,
                           @Nullable @FieldCoordinates @MatrixCoordinates OpenGLMatrix coordsGL,
                           @Nullable @FieldCoordinates @MatrixCoordinates int[] coordsInt,
                           boolean convertToMatrix) {
        ArrayList<int[]> intList = new ArrayList<>();
        if (coordsGL != null) {
            int[] convertedGL = CoordinateUtils.convertGLtoInt(coordsGL);
            intList.add(convertedGL);
        }
        if (coordsInt != null) {
            intList.add(coordsInt);
        }
        clearDynamic(space);
        addDynamic(space, intList, convertToMatrix);
    }



    /**
     * Appends new dynamic values to the field map without clearing previously added space values
     * Old values will be kept if there is a conflict
     * @param coordsGL an OpenGLMatrix hashtable of coordinates
     * @param coordsInt an integer hashtable of coordinates
     * @param convertToMatrix if true, converts coordinates from field coordinates to matrix coordinates
     */
    public void addDynamic(
            @Nullable @FieldCoordinates @MatrixCoordinates Hashtable<Space, ArrayList<OpenGLMatrix>> coordsGL,
            @Nullable @FieldCoordinates @MatrixCoordinates Hashtable<Space, ArrayList<int[]>> coordsInt,
            boolean convertToMatrix) {
        Hashtable<Space, ArrayList<int[]>> coordsIntN = CoordinateUtils.joinCoordinateHashtables(coordsInt, coordsGL);

        for (Space space : coordsIntN.keySet()) {
            addDynamic(space, coordsIntN.get(space), convertToMatrix);
        }
    }

    /**
     * Adds new values of the specified space type to the fieldmap from int coordinate array
     * Previous values will not be cleared, and if there's a conflict, the previous value will be kept
     *
     * @note All methods adding to the dynamic coordinates MUST ultimately call this function in order
     * to add the new spaces to the dynamicFillSpaces array. Otherwise, they should do it in that function,
     * but honestly its easier if it just happens here
     * @param space the type of non-static space to add
     * @param coordsInt The field coordinates to add as an integer array
     * @param convertToMatrix if true, converts coordinates from field coordinates to matrix coordinates
     */
    public void addDynamic(@NonNull Space space,
                           @NonNull @FieldCoordinates @MatrixCoordinates ArrayList<int[]> coordsInt,
                           boolean convertToMatrix) {
        if (space.isStatic() || coordsInt.size() == 0)
            return;

        if (dynamicFilledSpaces.get(space) == null) {
            dynamicFilledSpaces.put(space, new ArrayList<int[]>());
        }

        ArrayList<int[]> dynCoords = dynamicFilledSpaces.get(space);


        for (int[] xyPair : coordsInt) {
            if (convertToMatrix) // convert coordinates if convertToMatrix is true
                xyPair = fieldToMatrix(xyPair, true);
            // only add if its clear
            if (spaceMap.getSpace(xyPair) == Space.CLEAR) {
                spaceMap.setSpace(xyPair, space);
                dynCoords.add(xyPair);
                Log.d(TAG, "Added coords to space");
            }

        }
    }

    /**
     * Clear all dynamic coordinates from the fieldmap and set them to CLEAR
     * Static spaces are not cleared
     */
    public void clearDynamic() {
        for (Space space : Space.values()) {
            if (!space.isStatic())
                clearDynamic(space);
        }
    }

    /**
     * Clears a list of matrix coordinates from the space array
     * If the coordinate is a static spot, then it isnt cleared
     *
     * @note all functions that clear dynamic spots should call one of the clearDynamic functions to
     * ensure all dynamic/static coordinate arrays remain updated
     * @param intCoords the arraylist of matrix coordinates to clear
     */
    public void clearDynamic(@NonNull @MatrixCoordinates ArrayList<int[]> intCoords) {

        for (int[] xyPair : intCoords) {
            Space oldSpace = spaceMap.getSpace(xyPair);
            if (oldSpace.isStatic()) // if its static, we dont remove and move on
                continue;

            spaceMap.setSpace(xyPair, Space.CLEAR);
            // remove from recognition list if it exists
            recognitionFieldPositions.remove(xyPair);
            // try and remove coordinate from dyn hashtable, if its not there then itll throw an error
            // which we ignore. if the space is CLEAR, then the error will be thrown and we dont care
            try {
                ArrayList<int[]> dynCoords = dynamicFilledSpaces.get(oldSpace);
                dynCoords.remove(xyPair);
                if (dynCoords.size() == 0)
                    dynamicFilledSpaces.remove(oldSpace);
            } catch (NoSuchElementException | NullPointerException ignored) {}

        }
    }


    /**
     * Clear all dynamic coordinates of a certain type from the fieldmap
     * Also clears from dynamicFillSpaces hashtable and from rec field positions
     *
     * @note all functions that clear dynamic spots should either call this or the other
     * parameterized clearDynamic, since the spots need to be cleared in both the spaceArray and the
     * dynamicFillSpace array
     *
     * @param space the non-static space to clear from the fieldmap
     */
    public void clearDynamic(@NonNull Space space) {
        if (space.isStatic())
            return;

        try {
            ArrayList<int[]> currentCoords = dynamicFilledSpaces.get(space);
            for (int[] xyPair : currentCoords) {
                recognitionFieldPositions.remove(xyPair);
                spaceMap.setSpace(xyPair, Space.CLEAR);
            }
            dynamicFilledSpaces.remove(space);
        } catch (NullPointerException ignored) {}
    }


    // endregion set/add/clear dynamic

    // region coordinate transformations

    /**
     * Transforms field coordinates to matrix coordinates
     * If the transformed value is outside of the field's range, it is rounded off to the nearest edge
     *
     * @note these coordinates are presumed to have originated from a millimeter scale, and
     * therefore are converted to centimeters
     *
     * @param xyPair the coordinates to transform
     * @param roundToRange if true, rounds the coordinates to the spaceMap's range (must be inside walls)
     *                     otherwise, rounds to spacemap's bounds
     * @return the transformed xy coordinates
     */
    @NonNull
    @MatrixCoordinates
    private int[] fieldToMatrix(@NonNull @FieldCoordinates int[] xyPair, boolean roundToRange) {
        xyPair = xyPair.clone();
        xyPair[0] = (fieldTransform - xyPair[0]) / scale;
        xyPair[1] = (fieldTransform - xyPair[1]) / scale;

        if (roundToRange)
            xyPair = spaceMap.roundToRange(xyPair);
        else
            xyPair = spaceMap.roundToBounds(xyPair);

        return xyPair;
    }

    /**
     * Transforms matrix coordinates to field coordinates
     *
     * @note This shouldnt ever get out of bounds, but in case that changes, then add in a check
     * to round to the nearest edge
     *
     * @note these coordinates are converted to field coordinates, and therefore are converted to
     *      millimeters
     * @note these coordinates are inherently less accurate than the initial value used to create
     *      the matrix coordinates. if you need extreme accuracy, consider saving the original
     *      field coordinates. These will be within +/- scale of the originals
     *
     * @param xyPair the coordinates to transform
     * @return the transformed xyPair in field coordinates
     */
    @NonNull
    @FieldCoordinates
    private int[] matrixToField(@NonNull @MatrixCoordinates int[] xyPair) {
        xyPair = xyPair.clone();
        xyPair[0] = fieldTransform - (xyPair[0] * scale);
        xyPair[1] = fieldTransform - (xyPair[1] * scale);

        return xyPair;
    }

    /**
     * Change unscaled distance values to a matrix scale with integer values
     * @param distanceValues the distance values to scale
     * @return the scaled distance values
     */
    @NonNull
    @MatrixCoordinates
    @DistanceValues
    private int[] distancesToMatrixScale(@NonNull @FieldCoordinates @DistanceValues double[] distanceValues) {
        int[] scaledValues = new int[2];
        // we dont need to field transform because we're just converting units, not positions
        scaledValues[0] = (int) Math.round(distanceValues[0] / scale);
        scaledValues[1] = (int) Math.round(distanceValues[1] / scale);
        return scaledValues;
    }



    // endregion coordinate transformations

    // region pathfinding
    /**
     * Pathfind with the A* algorithm on the current fieldmap to the specified endcoords
     * @param end_field_coords the target coordinates, in field coordinates
     * @return a spaceMap with the path plotted out
     */
    @Nullable
    public SpaceMap aStarPathfind(@NonNull @FieldCoordinates int[] end_field_coords) {
        int[] robotCoords = dynamicFilledSpaces.get(Space.ROBOT).get(0);
        Log.d(TAG, String.format("Robot coords: {%s, %s}", robotCoords[0], robotCoords[1]));
        end_field_coords = fieldToMatrix(end_field_coords, true);
        Log.d(TAG, String.format("End coords: {%s, %s}", end_field_coords[0], end_field_coords[1]));

        AStar aStar = new AStar(robotCoords, end_field_coords, new SpaceMap(spaceMap));
        ArrayList<AStarNode> nodeList = aStar.findPath();
        return aStar.createPathFieldMap(nodeList);
    }

    /**
     * Pathfind with the D* Lite algorithm on the current fieldmap to the specified endcoords
     * @param end_field_coords the target coordinates, in field coordinates
     * @return a spaceMap with the path plotted out, or null if no path could be calculated
     */
    @Nullable
    public SpaceMap dLitePathfind(@NonNull @FieldCoordinates int[] end_field_coords) {
        int[] robotCoords = dynamicFilledSpaces.get(Space.ROBOT).get(0);
        Log.d(TAG, String.format("Robot coords: {%s, %s}", robotCoords[0], robotCoords[1]));
        end_field_coords = fieldToMatrix(end_field_coords, true);
        Log.d(TAG, String.format("End coords: {%s, %s}", end_field_coords[0], end_field_coords[1]));

        dLite = new DLite(robotCoords, end_field_coords, new SpaceMap(spaceMap));
        return dLite.calcuatePath();
    }

    /**
     * Determine if D* lite is initialized or not
     * @return true if its been initialized
     */
    public boolean isDLiteInitialized() {
        return !(dLite == null);
    }

    /**
     * Update D* Lite's pathfinding with the latest spacemap
     * @return D* Lite's updated pathfinding based on new information
     */
    @Nullable
    public SpaceMap updateDLite() {
        int[] robotCoords = dynamicFilledSpaces.get(Space.ROBOT).get(0);
        setDynamic(Space.ROBOT, null, robotCoords, true);
        return dLite.update(spaceMap, robotCoords);
    }

    // endregion pathfinding

    // region accessors/modifiers

    @NonNull
    public SpaceMap getSpaceMap() {
        return spaceMap;
    }

    /**
     * Gets a copy of the robot's coordinates
     * @ FIXME: 8/12/21 this *should* be notnull, but it cant be confirmed yet because we need
     *          to ensure there's always a robot position present on the map
     * @return the robot's coordinates, in matrix coords
     */
    @MatrixCoordinates
    private int[] getRobotPosition() {
        return dynamicFilledSpaces.get(Space.ROBOT).get(0).clone();
    }

    /**
     * Gets recognitions from tensorflow
     * @return the most updated list of recognitions
     */
    public List<Detection> getRecognitions() {
        return tfManager.getLatestDetections();
    }

    // endregion accessors/modifiers

}
