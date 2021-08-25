package opmodes;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.GamepadController;
import org.firstinspires.ftc.teamcode.GamepadController.ButtonState;
import org.firstinspires.ftc.teamcode.GamepadController.ToggleButton;

import java.util.List;

import pathfinding.VuforiaManager;
import pixel_distances.FocalDistances;
import pixel_distances.PixelDistances;
import tf_detection.Detection;
import tf_detection.TFManager;


@TeleOp(name="Distance: Distance OpMode", group="Testing")
public class DistanceOpMode extends OpMode {
    // tag is used in logcat logs (Log.d()) to identify where the log is coming from
    // logcat is basically like System.out.print (standard output) except through adb
    private static final String TAG = "vuf.test.dist_opmode"; // put the name of the opmode

    // put any outside classes you need to use here
    private GamepadController movementController;
    private GamepadController mechanismController;
    private VuforiaManager vuforiaManager;
    private TFManager tfManager;
    private PixelDistances focalDistances;


    // put any measurements here
    private final double inchesToMM = 25.4; // this is correct
    private final double toCameraCenter = 0.5; // inches from bottom of logitech c920 to actual camera
    private final double cameraPlatform = 10.5; // random value
    private final double cameraHeightMM = (cameraPlatform + toCameraCenter) * inchesToMM;
    private static final int fieldLength = 3660; // mm (this is correct)



    // code to run once when driver hits init on phone
    @Override
    public void init() {
        movementController = new GamepadController(gamepad1);
        mechanismController = new GamepadController(gamepad2);
        vuforiaManager = new VuforiaManager(hardwareMap, fieldLength, false);
        tfManager = new TFManager(hardwareMap, vuforiaManager, TFManager.DetectorType.FTC_TFOD, true);
        focalDistances = new FocalDistances(cameraHeightMM, vuforiaManager.getCameraCalibration());
    }

    // code to loop after init is pressed and before start is pressed
    @Override
    public void init_loop() {
    }

    // code to run once when driver hits start
    @Override
    public void start() {
    }

    // code to loop while opmode is running
    @Override
    public void loop() {

        runControls();


        // update telemetry at the end of the loop
        telemetry.update();
    }

    /**
     * Updates buttons and does controls when buttons are pressed
     */
    public void runControls() {

        // button states need to be updated each loop for controls to work
        movementController.updateButtonStates();

        List<Detection> updatedDetections = tfManager.getLatestDetections();
        // get updated recognitions, but only if there's changes
        for (Detection detection : updatedDetections) {
            telemetry.addData(detection.getLabel(),
                    "Top: %s, Bottom: %s, Left: %s, Right: %s ",
                    detection.getTop(), detection.getBottom(), detection.getLeft(), detection.getRight());
        }

        // do something when A is pressed
        if (movementController.getButtonState(ToggleButton.A) == ButtonState.KEY_DOWN) {
            Log.d(TAG, "button a pressed");
            if (updatedDetections.size() > 0) {
                Detection detection = updatedDetections.get(0);
                double[] distances = focalDistances.getDistancesToPixels(detection.getBottom(), detection.getCenterX());
                Log.d(TAG, String.format("FD distances {straight, side}: {%s, %s}", distances[0], distances[1]));
            } else {
                Log.d(TAG, "No recognitions");
            }
        }

        if (movementController.getButtonState(ToggleButton.B) == ButtonState.KEY_DOWN) {
            Log.d(TAG, "button b pressed");
            float[][] testCoordinates = new float[][] {
                    {0, 0},
                    {357, 220},
                    {309, 220},
                    {283, 220}, // bad
                    {269, 220}, // bad
                    {260, 220}, // bad
            };

            for (int i = 0; i < testCoordinates.length; i++) {
                float[] coordinate = testCoordinates[i];
                double[] distances = focalDistances.getDistancesToPixels(coordinate[0], coordinate[1]);
                Log.i(TAG, String.format("Distance to %s, {straight, side}: {%s, %s}", (i+1) * 609.6, distances[0], distances[1]));
            }
        }

        if (movementController.getButtonState(ToggleButton.X) == ButtonState.KEY_DOWN) {
            Log.d(TAG, "button x pressed");
            double[] distances = focalDistances.getSetupDistances();
            Log.d(TAG, String.format("Calibration distances {bottom dist, middle dist}: {%s, %s}", distances[0], distances[1]));
        }

        if (movementController.getButtonState(ToggleButton.Y) == ButtonState.KEY_DOWN) {
            Log.d(TAG, "saving frame");
            vuforiaManager.captureFrameToFile();
        }



    }


    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }


}
