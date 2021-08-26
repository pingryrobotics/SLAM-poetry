package opmodes;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.teamcode.GamepadController;
import org.firstinspires.ftc.teamcode.GamepadController.ButtonState;
import org.firstinspires.ftc.teamcode.GamepadController.ToggleButton;

import java.util.ArrayList;
import java.util.HashMap;

import display.Visuals;
import localization.FieldMap;
import localization.SpaceMap;
import localization.VuforiaManager;
import pixel_distances.FocalDistances;
import pixel_distances.PixelDistances;
import tf_detection.TFManager;


@TeleOp(name="Pathfinding: Pathfinding OpMode", group="Testing")
public class PathfindingOpMode extends OpMode {
    // field declarations
    private static final int nanoToMilliseconds = 1000000;
    private GamepadController movementController;
    private GamepadController mechanismController;
    private FieldMap fieldMap;
    private PixelDistances pixelDistances;
    private VuforiaManager vuforiaManager;
    private static final String TAG = "vuf.test.pathfindingOp";

    // total field length
    // 0,0 is a coordinate, keep that in mind for testing calculations
    private static final int fieldLength = 310;

    @Override
    public void init() {
        movementController = new GamepadController(gamepad1);
        vuforiaManager = new VuforiaManager(hardwareMap, fieldLength, false);

        HashMap<SpaceMap.Space, ArrayList<OpenGLMatrix>> staticCoordsGL = new HashMap<>();
        staticCoordsGL.put(SpaceMap.Space.IMAGE_TARGET, vuforiaManager.getTrackablePositions());
        TFManager tfManager = new TFManager(hardwareMap, vuforiaManager, TFManager.DetectorType.FTC_TFOD, false);
        pixelDistances = new FocalDistances(0, vuforiaManager.getCameraCalibration());
        fieldMap = new FieldMap(fieldLength, staticCoordsGL, null, tfManager, pixelDistances, false);
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {

        runControls();
    }

    /**
     * Does controls for the gamepads
     */
    public void runControls() {

        for (VuforiaManager.ImageTarget key : VuforiaManager.ImageTarget.cachedValues()) {
            telemetry.addData(key.name(), vuforiaManager.isTrackableVisible(key) ? "Visible" : "Not Visible");
        }

        telemetry.addData("Last robot position: ", VuforiaManager.format(vuforiaManager.getUpdatedRobotPosition()));


        movementController.updateButtonStates();

        if (movementController.getButtonState(ToggleButton.A) == ButtonState.KEY_DOWN) {
            //do something when the button is pressed
            Visuals.fieldMapToImage(fieldMap, "fieldMap1");
        }

        if (movementController.getButtonState(ToggleButton.B) == ButtonState.KEY_DOWN) {
            OpenGLMatrix location = vuforiaManager.getUpdatedRobotPosition();
            if (location != null) {

                int[] targetCoords = new int[] {75, 75};
                fieldMap.setRobotPosition(location);
                fieldMap.getSpaceMap().setSpace(SpaceMap.Space.TARGET_LOCATION, targetCoords, true);
                Visuals.fieldMapToImage(fieldMap, "A_pre");
                long startTime = System.nanoTime();

                SpaceMap pathSpaceMap = fieldMap.aStarPathfind(targetCoords);

                long endTime = System.nanoTime();
                long duration = (endTime - startTime)/nanoToMilliseconds;
                Log.d(TAG, String.format("A* completed in %s ms", duration));

                if (pathSpaceMap != null) {
                    Visuals.fieldMapToImage(pathSpaceMap.getRawMap(), "A_post");
                }

            }
        }

        if (movementController.getButtonState(ToggleButton.X) == ButtonState.KEY_DOWN) {


            int[] targetCoords = new int[] {75, 75};

            OpenGLMatrix location = vuforiaManager.getUpdatedRobotPosition();
            if (location != null) {
                try {
                    fieldMap.setRobotPosition(location);
                    fieldMap.getSpaceMap().setSpace(SpaceMap.Space.TARGET_LOCATION, targetCoords, true);
                    fieldMap.getSpaceMap().setSpace(SpaceMap.Space.OBSTACLE, getObstruction(), true);
                    Visuals.fieldMapToImage(fieldMap, "A_obstruction_pre");

                    long startTime = System.nanoTime();

                    SpaceMap pathSpaceMap = fieldMap.aStarPathfind(targetCoords);

                    long endTime = System.nanoTime();
                    long duration = (endTime - startTime)/nanoToMilliseconds;

                    Log.d(TAG, String.format("A* completed in %s ms", duration));

                    if (pathSpaceMap != null) {
                        Visuals.fieldMapToImage(pathSpaceMap.getRawMap(), "A_obstruction_post");
                        Log.d(TAG, "A* successfully plotted path");
                    } else {
                        Log.d(TAG, "A* could not find path");
                    }


                } catch (Exception e) {
                    Log.e(TAG, e.toString());
                }

            }
        }

        if (movementController.getButtonState(ToggleButton.DPAD_DOWN) == ButtonState.KEY_DOWN) {
            OpenGLMatrix location = vuforiaManager.getUpdatedRobotPosition();


            if (location != null) {
                int[] targetCoords = new int[] {75, 75};
                fieldMap.setRobotPosition(location);
                fieldMap.getSpaceMap().setSpace(SpaceMap.Space.TARGET_LOCATION,  targetCoords, true);


                SpaceMap pathSpaceMap;

                if (!fieldMap.isDLiteInitialized()) {
                    Log.d(TAG, "not initialized");
                    Visuals.fieldMapToImage(fieldMap, "D_complete_pre");

                    long startTime = System.nanoTime();

                    pathSpaceMap = fieldMap.dLitePathfind(targetCoords);

                    long endTime = System.nanoTime();
                    long duration = (endTime - startTime)/nanoToMilliseconds;

                    Log.d(TAG, String.format("D* Lite move planned in %s ms", duration));
                } else {

                    fieldMap.getSpaceMap().setSpace(SpaceMap.Space.OBSTACLE, getObstruction(), true);

                    Visuals.fieldMapToImage(fieldMap, "D_complete_pre");
                    long startTime = System.nanoTime();

                    pathSpaceMap = fieldMap.updateDLite();

                    long endTime = System.nanoTime();
                    long duration = (endTime - startTime)/nanoToMilliseconds;

                    Log.d(TAG, String.format("D* Lite move updated in %s ms", duration));


                }
                if (pathSpaceMap != null) {
                    Log.d(TAG, "valid spacemap, saving");
                    Visuals.fieldMapToImage(pathSpaceMap.getRawMap(), "D_complete_post");
                } else {
                    Log.d(TAG, "no path found");
                }


            }

        }

        telemetry.update();
    }

    /**
     * Helper function to generate an obstruction
     * @return the obstructions coordinates
     */
    public ArrayList<int[]> getObstruction() {
        ArrayList<int[]> obstructionList = new ArrayList<>();

            for (int i = 0; i < 160; i++) {
            obstructionList.add(new int[] {i, 50});
            obstructionList.add(new int[] {-10, i});
            obstructionList.add(new int[] {90, i});
                obstructionList.add(new int[] {90, i});
//            obstructionList.add(new int[] {i, -1000});
//            obstructionList.add(new int[] {-500, i-10});
        }
//            int[] newCoords = new int[] {
//                    ThreadLocalRandom.current().nextInt(-1824, 1824 + 1),
//                    ThreadLocalRandom.current().nextInt(-1824, 1824 + 1)};


        return obstructionList;
    }



    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
