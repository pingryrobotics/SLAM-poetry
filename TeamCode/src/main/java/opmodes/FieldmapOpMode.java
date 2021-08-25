package opmodes;

import android.annotation.SuppressLint;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.teamcode.GamepadController;
import org.firstinspires.ftc.teamcode.GamepadController.ButtonState;
import org.firstinspires.ftc.teamcode.GamepadController.ToggleButton;

import java.util.ArrayList;
import java.util.HashMap;

import pathfinding.FieldMap;
import pathfinding.SpaceMap;
import pathfinding.VuforiaManager;
import pixel_distances.FocalDistances;
import pixel_distances.PixelDistances;
import tf_detection.Detection;
import tf_detection.TFManager;


@TeleOp(name="Fieldmap: Fieldmap OpMode", group="Testing")
public class FieldmapOpMode extends OpMode {
    // field declarations
    private static final String TAG = "vuf.test.tfOpMode";
    private GamepadController movementController;
    private GamepadController mechanismController;
    private static final long nanoToMilli = 1000000;

    private VuforiaManager vuforiaManager;
    private FieldMap fieldMap;
    private PixelDistances pixelDistances;
    private final double inchesToMM = 25.4;
    private final double toCameraCenter = 1.25; // inches from bottom of logitech c615 to actual camera
    private final double cameraPlatform = 16; // inches
    private final double cameraHeight = (cameraPlatform + toCameraCenter) * inchesToMM;

    private static final int fieldLength = 3660;


    @Override
    public void init() {
        movementController = new GamepadController(gamepad1);
        vuforiaManager = new VuforiaManager(hardwareMap, fieldLength, false);
        TFManager tfManager = new TFManager(hardwareMap, vuforiaManager,
                TFManager.DetectorType.FTC_TFOD, true);

        HashMap<SpaceMap.Space, ArrayList<OpenGLMatrix>> staticCoordsGL = new HashMap<>();
        staticCoordsGL.put(SpaceMap.Space.IMAGE_TARGET, vuforiaManager.getLocTrackablesAsMatrices());
        pixelDistances = new FocalDistances(cameraHeight, vuforiaManager.getCameraCalibration());
        fieldMap = new FieldMap(fieldLength, staticCoordsGL, null, tfManager, pixelDistances, true);
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
    @SuppressLint("DefaultLocale")
    public void runControls() {

        movementController.updateButtonStates();
//        // update map
        OpenGLMatrix location = vuforiaManager.getUpdatedRobotPosition();
        if (location != null) {
            long startTime = System.nanoTime();
            fieldMap.update(location);
            long duration = (System.nanoTime() - startTime)/nanoToMilli;
            Log.i(TAG, "Finished updating map in " + duration + " ms");
            telemetry.addData("Robot position", VuforiaManager.format(location));
        }  else {
            Log.d(TAG, "no location");
        }
        // get trackable status
        for (VuforiaManager.LocalizationTrackable trackable : VuforiaManager.LocalizationTrackable.cachedValues()) {
            telemetry.addData(trackable.name(), vuforiaManager.isTrackableVisible(trackable) ? "Visible" : "Not Visible");
        }

        if (movementController.getButtonState(ToggleButton.A) == ButtonState.KEY_DOWN) {
            location = vuforiaManager.getUpdatedRobotPosition();
            if (location != null) {
                long startTime = System.nanoTime();
                fieldMap.update(location);
                long duration = (System.nanoTime() - startTime)/nanoToMilli;
                Log.i(TAG, "Finished updating map in " + duration + " ms");
            } else {
                Log.d(TAG, "no location");
            }

        }

        if (movementController.getButtonState(ToggleButton.B) == ButtonState.KEY_DOWN) {
            location = vuforiaManager.getUpdatedRobotPosition();
            if (location != null) {
                Log.d(TAG, "printing transformations");
                fieldMap.setRobotPosition(location);
                fieldMap.checkDisappearances();
            }
        }

        if (movementController.getButtonState(ToggleButton.DPAD_UP) == ButtonState.KEY_DOWN) {
            fieldMap.getSpaceMap().catalog();
        }

        if (movementController.getButtonState(ToggleButton.X) == ButtonState.KEY_DOWN) {
            fieldMap.updateDisplay();
        }

        for (Detection detection : fieldMap.getRecognitions()) {
            telemetry.addData(detection.getLabel(), "Visible");
        }

        telemetry.update();
    }


    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }


}
