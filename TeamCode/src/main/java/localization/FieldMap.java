package localization;


import static localization.SpaceMap.Space;

import android.graphics.Bitmap;
import android.graphics.RectF;
import android.util.Log;
import android.util.Pair;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import org.firstinspires.ftc.robotcore.external.Function;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import annotations.FieldCoordinates;
import annotations.ImageCoordinates;
import annotations.MatrixCoordinates;
import display.Visuals;
import localization.DetectionMapper.PotentialDetection;
import pathfinding.AStar;
import pathfinding.AStarNode;
import pathfinding.DLite;
import pixel_distances.PixelDistances;
import tf_detection.Detection;
import display.DisplaySource;
import tf_detection.TFManager;

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
 * General project-wide to do list
 * @ TODO: 8/12/21 There should only ever be one robot position space at once
 * @ TODO: 8/25/21 Add downwards angles for camera
 * @ TODO: 8/25/21 improve pixel distance accuracy
 * @ TODO: 8/25/21 fix issue where recognitions get remapped because of slight position differences
 *
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

    static final String TAG = "vuf.test.fieldmap";

    // amount to extend reverse recognition image coordinates rects by
    // see isObjectGone
    private static final int pxTolerance = 10;

    private static final int scale = 75; // value to scale the field values down by
    public static final int bitmapDisplaySize = 300; // good size to make bitmaps on the display
    // 10 is good for mm to cm for testing, 75 is nice for actual field mm to a manageable size (~48x48)

    // the x and y dimensions of the field are the same, so we have fieldSize
    private final int fieldSize;

    // the amount that x and y field coordinates are transformed for matrix positioning
    private final int fieldTransform;

    private DLite dLite;
    private SpaceMap spaceMap;
    private final TFManager tfManager;
    private final PixelDistances pixelDistances;
    private DisplaySource displaySource;

    private OpenGLMatrix robotPositionGL;

    private final boolean useDisplay;


    /**
     * When field positions are calculated from tensorflow recognitions, they're placed
     * onto the spacemap, but in that process, the exact location is lost through the conversion
     * between GL -> matrix coordinates. This is an issue, because when we check if a certain
     * recognition is still present on the field, we use its field position to do so. We could
     * just convert from matrix back to field, but then we're working with a less precise metric
     * and therefore will have a less precise answer. So, instead, we maintain a list of
     * the recognition's field positions along with their matrix position so we can check
     * when recognitions disappear.
     *
     * Additionally, to prevent a recognition from being mapped repeatedly but in slightly
     * different positions, we check if a new recognition is within a square (~scale mm) of
     * a mapped recognition. If it is, we update that recognition's position rather than adding
     * in a whole new one for a slightly different position
     *
     */
//    @FieldCoordinates
//    private final ArrayList<DetectionMapper.MappedDetection> mappedDetectionList = new ArrayList<>();
    private final DetectionMapper detectionMapper = new DetectionMapper();

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
     *@param fieldSizeMM The height and width of the field, in real millimeters
     * @param staticCoordsGL The static spaces on the field as OpenGLMatrix coordinates. Can be null
     * @param staticCoordsInt The static spaces on the field as int array coordinates. Can be null
     * @param tfManager the tensorflow manager to use with the fieldmap.
     * @param pixelDistances the pixel distance finder to determine distances to pixels on screen
     * @param useDisplay if true, projects the field map onto the robot's display      */
    public FieldMap(
            int fieldSizeMM,
            @Nullable @FieldCoordinates HashMap<Space, ArrayList<OpenGLMatrix>> staticCoordsGL,
            @Nullable @FieldCoordinates HashMap<Space, ArrayList<int[]>> staticCoordsInt,
            @NonNull TFManager tfManager,
            @NonNull PixelDistances pixelDistances,
            boolean useDisplay) {
        // convert mm to cm
        this.fieldSize = fieldSizeMM / scale;
        this.fieldTransform = fieldSizeMM/2;
        this.tfManager = tfManager;
        this.pixelDistances = pixelDistances;
        this.useDisplay = useDisplay;
        initializeDisplay(useDisplay);
        initializeFieldMap();
        initializeStaticFills(staticCoordsGL, staticCoordsInt);
    }

    /**
     * Initialize the display source for projecting field map visuals to the robot's screen
     * @param useDisplay whether to use the display or not
     */
    private void initializeDisplay(boolean useDisplay) {
        int displayId = 0;
        if (useDisplay) {
            displayId = AppUtil.getDefContext().getResources().getIdentifier(
                    "extraMonitorViewId", "id", AppUtil.getDefContext().getPackageName());
        }
        this.displaySource = new DisplaySource(displayId);
    }

    /**
     * Initializes all spots of the field map to clear, then adds walls
     * Also adds wall coordinates to static coordinate hashtable
     */
    private void initializeFieldMap() {
        Log.d(TAG, "initializing field map");
        // create array of clear spaces with the height and width of the field
        spaceMap = new SpaceMap(fieldSize, fieldSize, 1, fieldSize-2);

        // first and last row and column
        spaceMap.setWalls();
        Log.d(TAG, "Field map initialized");

    }

    /**
     * Transforms x and y coordinates from field coordinates to matrix coordinates and
     * adds them to the field map from OpenGLMatrix and integer coordinates.
     *
     */
    private void initializeStaticFills(
            @Nullable @FieldCoordinates
            HashMap<Space, ArrayList<OpenGLMatrix>> coordsGL,
            @Nullable @FieldCoordinates
            HashMap<Space, ArrayList<int[]>> coordsInt) {
        Log.d(TAG, "Initializing static fills");
        Map<Space, ArrayList<int[]>> joinedCoords =
                CoordinateUtils.joinCoordinateHashtables(coordsInt, coordsGL);
        Log.d(TAG, "Adding statics to spaceArray");

        joinedCoords.forEach((space, coordsList) -> {
            for (int[] xyPair : coordsList) {
                // x and y coordinates are transformed for accurate positioning
                // dont round to range because image targets are on walls
                xyPair = fieldToMatrix(xyPair, false);
                Log.d(TAG, String.format("x: %s, y: %s", xyPair[0], xyPair[1]));
                // replace the space
                spaceMap.setSpace(space, xyPair, true);
            }
        });

        Log.d(TAG, "Finished initializing static fills");
    }

    // endregion initialization

    // region recognition mapping

    /**
     * Update the field map.
     * Updates the robot's position, the object detector recognitions, the mappings of said
     * recognitions, and the display, if enabled.
     * @param robotPositionGL the robot's position
     */
    public void update(OpenGLMatrix robotPositionGL) {
        setRobotPosition(robotPositionGL);
        updateDynamicPositions();
        updateDisplay();
    }

    /**
     * Update the display with the current spacemap
     */
    public void updateDisplay() {
        if (useDisplay) {
            Bitmap bitmap = Visuals.spaceMapToBitmap(spaceMap.getRawMap());
            bitmap = Bitmap.createScaledBitmap(bitmap, bitmapDisplaySize, bitmapDisplaySize, false);
            displaySource.updateImageView(bitmap);
        }
    }

    /**
     * Updates the dynamic positions based on tensorflow recognitions
     * Gets the positions of new recognitions and places them on the spacemap
     * Also looks at all recognition field positions and checks if they're still present
     */
    public void updateDynamicPositions() {
        checkDisappearances(); // check disappearances beforehand
        List<PotentialDetection> potentialDetections = getPotentialDetections();
        Pair<ArrayList<int[]>, HashMap<Space, ArrayList<int[]>>> updatedCoords =
                detectionMapper.update(potentialDetections);
        spaceMap.clearSpace(updatedCoords.first, false);
        spaceMap.addSpace(updatedCoords.second, false);
    }


    /**
     * Checks the recognition field positions list to see if any recognitions have disappeared
     * from their previous position. Essentially, we look at each camera and check the pixels where
     * the recognitions would be on their view. If there's a recognition blocking the pixel,
     * then we can't confirm if its there or not, so we assume it is. If there's nothing there,
     * we know its gone and we can remove it.
     */
    public void checkDisappearances() {
        // get float rectangles for all the recognitions on screen
        List<RectF> recognitionPixels = new ArrayList<>();
        for (Detection recognition : tfManager.getLatestDetections()) {
            recognitionPixels.add(recognition.getRectF());
        }

        // give the detection mapper a function to use to check if an object is gone or not
        // we need to provide it since the dm cant use fieldmap instance methods, nor should it directly
        detectionMapper.markDisappearances(new Function<OpenGLMatrix, Boolean>() {
            @Override
            public Boolean apply(OpenGLMatrix arg) {
                return isObjectGone(robotPositionGL, arg, recognitionPixels);
            }
        });
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
                                 @NonNull @ImageCoordinates List<RectF> recognitionPixels) {

        float[] imageCoords = getImageCoordsFromFieldPosition(cameraPosition, fieldPosition);
        // make a box around the image coordinates so there's some leeway
        // if the image coordinates arent found, then theyre not in the camera's view
        float top = Math.max(imageCoords[0]-pxTolerance, 0);
        float bottom = (float) Math.min(imageCoords[0]+pxTolerance, tfManager.imageHeight);
        float left = Math.max(imageCoords[1]-pxTolerance, 0);
        float right = (float) Math.min(imageCoords[1]+pxTolerance, tfManager.imageWidth);
        RectF posRect = new RectF(left, top, right, bottom);

        // if the object's coordinates are obscured by a recognition, then we dont know if its
        // there or not. It could also BE the recognition, so either way its not definitively gone
        for (RectF recRect : recognitionPixels) {
            if (recRect.intersect(posRect)) {
                /*
                If we find that too many objects are being falsely occluded, then we can
                limit the definition of 'occlusion' to above a certain iou (intersect/union)
                 */
                return false;
            }
        }
        // if we can confirm that it isnt covered by a recognition, but the location
        // where it would be is on screen, then its not there
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
    @NonNull
    @ImageCoordinates
    private float[] getImageCoordsFromFieldPosition(
            @NonNull @FieldCoordinates OpenGLMatrix cameraPosition,
            @NonNull @FieldCoordinates OpenGLMatrix fieldPosition) {
        float[] distances = CoordinateUtils.calculateDistanceToCamera(cameraPosition, fieldPosition);
        return pixelDistances.getPixelsFromDistances(distances[0], distances[1]);
    }

    /**
     * Get the position of elements on the field map and map them at their absolute location
     * Rotates the recognitions around the robot to correctly place them on the map so they're
     * position corresponds to the direction the robot was facing when it recognized them
     * Also gets the field coordinates for the recognitions to make confirming locations at later
     * points easier and more accurate
     *
     * @ FIXME: 8/13/21 sort of hardcoded with labels for testing
     * @return A list of potential recognition object representing the potential recognitions' coordinates
     */
    @SuppressWarnings("FeatureEnvy")
    @NonNull
    @MatrixCoordinates
    @FieldCoordinates
    private List<PotentialDetection> getPotentialDetections() {

        List<PotentialDetection> potentialRecognitions = new ArrayList<>();

        // loop through all recognitions
        List<Detection> recognitionList = tfManager.getLatestDetections();
        for (Detection detection : recognitionList) {
            // get their position on the field
            OpenGLMatrix fieldPosition = CoordinateUtils.convertDistancesToFieldPosition(
                    robotPositionGL, pixelDistances.getDistancesToPixels(detection.getBottom(), detection.getCenterX()));
            // convert field position to matrix coords
            int[] matrixPosition = fieldToMatrix(fieldPosition);
            // target if gold, obstacle if silver
            Space space = (detection.getLabel().equals("Gold Mineral"))
                    ? Space.TARGET_LOCATION : Space.OBSTACLE;
            // add to list of potential recognitions
            potentialRecognitions.add(new PotentialDetection(fieldPosition, matrixPosition, space));
        }
        return potentialRecognitions;
    }


    // endregion recognition mapping


    // region set/add/clear dynamic

    /**
     * Set the robot's position and clears the old position
     * @param position the position as an OpenGLMatrix
     */
    public void setRobotPosition(@NonNull @FieldCoordinates OpenGLMatrix position) {
        robotPositionGL = new OpenGLMatrix(position); // copy because its safer
        int[] robotCoords = fieldToMatrix(robotPositionGL);
        spaceMap.setRobotPosition(robotCoords);
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
     * Converts gl coordinates to matrix coordinates by converting gl to integer coords, then to matrix
     * Rounds to within range
     * @param glCoords the gl coords to convert
     * @return the new matrix coordinates
     */
    private int[] fieldToMatrix(@NonNull @FieldCoordinates OpenGLMatrix glCoords) {
        return fieldToMatrix(CoordinateUtils.convertGLtoInt(glCoords), true);
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



    // endregion coordinate transformations

    // region pathfinding
    /**
     * Pathfind with the A* algorithm on the current fieldmap to the specified end coords
     * @param end_field_coords the target coordinates, in field coordinates
     * @return a spaceMap with the path plotted out, or null if there's no path or no robot position
     */
    @Nullable
    public SpaceMap aStarPathfind(@NonNull @FieldCoordinates int[] end_field_coords) {
        int[] robotCoords = spaceMap.getRobotPosition();
        if (robotCoords == null)
            return null;
        Log.d(TAG, String.format("Robot coords: {%s, %s}", robotCoords[0], robotCoords[1]));
        end_field_coords = fieldToMatrix(end_field_coords, true);
        Log.d(TAG, String.format("End coords: {%s, %s}", end_field_coords[0], end_field_coords[1]));

        AStar aStar = new AStar(robotCoords, end_field_coords, new SpaceMap(spaceMap));
        ArrayList<AStarNode> nodeList = aStar.findPath();
        return aStar.createPathFieldMap(nodeList);
    }

    /**
     * Pathfind with the D* Lite algorithm on the current fieldmap to the specified end coords
     * @param end_field_coords the target coordinates, in field coordinates
     * @return a spaceMap with the path plotted out, or null if no path could be calculated or
     * if there's no robot position
     */
    @Nullable
    public SpaceMap dLitePathfind(@NonNull @FieldCoordinates int[] end_field_coords) {
        int[] robotCoords = spaceMap.getRobotPosition();
        if (robotCoords == null)
            return null;
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
        return (dLite != null);
    }

    /**
     * Update D* Lite's pathfinding with the latest spacemap
     * @return D* Lite's updated pathfinding based on new information. if there's no robot position,
     * or there's no changes, null is returned
     */
    @Nullable
    public SpaceMap updateDLite() {
        int[] robotCoords = spaceMap.getRobotPosition();
        if (robotCoords == null)
            return null;
        return dLite.update(spaceMap, robotCoords);
    }

    // endregion pathfinding

    // region accessors/modifiers

    @NonNull
    public SpaceMap getSpaceMap() {
        return spaceMap;
    }


    /**
     * Gets recognitions from tensorflow
     * @return the most updated list of recognitions
     */
    public List<Detection> getRecognitions() {
        return tfManager.getLatestDetections();
    }

    public static int getScale() { return scale; }

    // endregion accessors/modifiers

}
