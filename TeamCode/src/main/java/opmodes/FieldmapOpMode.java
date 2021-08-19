package opmodes;

import android.annotation.SuppressLint;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.teamcode.GamepadController;
import org.firstinspires.ftc.teamcode.GamepadController.ButtonState;
import org.firstinspires.ftc.teamcode.GamepadController.ToggleButton;

import java.util.ArrayList;
import java.util.Hashtable;

import pathfinding.FieldMap;
import pathfinding.SpaceMap;
import pathfinding.Visuals;
import pathfinding.VuforiaManager;
import tf_detection.Detection;
import tf_detection.TFManager;


@TeleOp(name="Fieldmap: Fieldmap OpMode", group="Testing")
public class FieldmapOpMode extends OpMode {
    // field declarations
    private static final String TAG = "vuf.test.tfOpMode";
    private GamepadController movementController;
    private GamepadController mechanismController;

    private VuforiaManager vuforiaManager;
    private FieldMap fieldMap;
    private final double inchesToMM = 25.4;
    private final double toCameraCenter = 1.25; // inches from bottom of logitech c615 to actual camera
    private final double cameraPlatform = 10.5;
    private final double cameraHeight = (cameraPlatform + toCameraCenter) * inchesToMM;

    private static final int fieldLength = 3660;


    @Override
    public void init() {
        movementController = new GamepadController(gamepad1);
        vuforiaManager = new VuforiaManager(hardwareMap, fieldLength, false);
        TFManager tfManager = new TFManager(hardwareMap, cameraHeight, vuforiaManager,
                TFManager.DetectorType.FTC_TFOD, false);

        Hashtable<SpaceMap.Space, ArrayList<OpenGLMatrix>> staticCoordsGL = new Hashtable<>();
        staticCoordsGL.put(SpaceMap.Space.IMAGE_TARGET, vuforiaManager.getLocTrackablesAsMatrices());

        fieldMap = new FieldMap(fieldLength, staticCoordsGL, null, tfManager);
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

        for (VuforiaManager.LocalizationTrackable key : VuforiaManager.LocalizationTrackable.cachedValues()) {
            /*
             * getUpdatedRobotLocation() will return null if no new information is available since
             * the last time that call was made, or if the trackable is not currently visible.
             * getRobotLocation() will return null if the trackable is not currently visible.
             */
            VuforiaTrackable trackable = vuforiaManager.getLocalizationTrackable(key);
            telemetry.addData(trackable.getName(), ((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible() ? "Visible" : "Not Visible");
        }

        if (movementController.getButtonState(ToggleButton.A) == ButtonState.KEY_DOWN) {
            OpenGLMatrix location = vuforiaManager.getUpdatedRobotPosition();
            if (location != null) {
                Log.d(TAG, "mapping recognitions");
                fieldMap.setRobotPosition(location);
                fieldMap.updateDynamicPositions();
                Visuals.fieldMapToImage(fieldMap.getSpaceMap().getRawMap(), "recognitionMap");
            } else {
                Log.d(TAG, "no location");
            }

        }

        if (movementController.getButtonState(ToggleButton.B) == ButtonState.KEY_DOWN) {
            OpenGLMatrix location = vuforiaManager.getUpdatedRobotPosition();
            if (location != null) {
                Log.d(TAG, "printing transformations");
                fieldMap.setRobotPosition(location);
                fieldMap.checkDisappearances();
            }
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
