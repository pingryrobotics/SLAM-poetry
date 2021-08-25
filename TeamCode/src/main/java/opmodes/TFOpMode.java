package opmodes;

import static android.content.Context.ACTIVITY_SERVICE;

import android.annotation.SuppressLint;
import android.app.ActivityManager;
import android.graphics.Bitmap;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.GamepadController;
import org.firstinspires.ftc.teamcode.GamepadController.ButtonState;
import org.firstinspires.ftc.teamcode.GamepadController.ToggleButton;

import java.util.ArrayList;
import java.util.List;

import pathfinding.Visuals;
import pathfinding.VuforiaManager;
import tf_detection.Detection;
import tf_detection.TFManager;


@TeleOp(name="TF: TF OpMode", group="Testing")
public class TFOpMode extends OpMode {
    // field declarations
    private static final String TAG = "vuf.test.tfOpMode";
    private GamepadController movementController;
    private GamepadController mechanismController;
    private VuforiaManager vuforiaManager;
    private TFManager tfManager;
    private final double toCameraCenter = 1.25; // inches from bottom of logitech c615 to actual camera
    private final double cameraPlatform = 10.5;
    private final double cameraHeight = cameraPlatform + toCameraCenter;
    private final int fieldLength = 3660;
    private final ArrayList<Bitmap> bitmapList = new ArrayList<>();


    @Override
    public void init() {
        movementController = new GamepadController(gamepad1);
        vuforiaManager = new VuforiaManager(hardwareMap, fieldLength, false);
        tfManager = new TFManager(hardwareMap, vuforiaManager,
                TFManager.DetectorType.FTC_TFOD, false);
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

//        boolean visible = vuforiaObj.isTrackableVisible(Vuforia.LocalizationTrackable.FIRST);
//        if (visible != wasVisible) {
//            Log.d("vuf.test", "Visible: " + visible);
//            wasVisible = visible;
//        }

        movementController.updateButtonStates();
        if (movementController.getButtonState(ToggleButton.A) == ButtonState.KEY_DOWN) {
            ActivityManager.MemoryInfo mi = new ActivityManager.MemoryInfo();
            ActivityManager activityManager = (ActivityManager) AppUtil.getDefContext()
                    .getSystemService(ACTIVITY_SERVICE);

            long largeHeapSize = activityManager.getLargeMemoryClass();
            long defaultHeapSize = activityManager.getMemoryClass();
            long lowMemThreshold = mi.threshold;
            boolean lowMemory = mi.lowMemory;

            Log.d(TAG, "Default heap size: " + defaultHeapSize);
            Log.d(TAG, "Large heap size: " + largeHeapSize);
            Log.d(TAG, "Low memory threshold: " + lowMemThreshold);
            Log.d(TAG, "Low on memory?: " + lowMemory);
        }

        if (movementController.getButtonState(ToggleButton.RIGHT_TRIGGER) == ButtonState.KEY_HOLD) {
            bitmapList.add(Visuals.loadImageBitmap());
            Log.d(TAG, "bitmap list size: " + bitmapList.size());
        }

        List<Detection> updatedDetections = tfManager.getLatestDetections();
        // get updated recognitions, but only if there's changes
        if (updatedDetections != null) {
            for (Detection detection : updatedDetections) {
                telemetry.addData(detection.getLabel(),
                        "Top: %s, Bottom: %s, Left: %s, Right: %s ",
                        detection.getTop(), detection.getBottom(), detection.getLeft(), detection.getRight());
            }
        }

        for (VuforiaManager.LocalizationTrackable key : VuforiaManager.LocalizationTrackable.cachedValues()) {
            telemetry.addData(key.name(),
                    vuforiaManager.isTrackableVisible(key) ? "Visible" : "Not Visible");
        }

        OpenGLMatrix location = vuforiaManager.getUpdatedRobotPosition();

        if (location != null) {
            telemetry.addData("Current position", VuforiaManager.format(location));
        }





//        float left_stick_x = movementController.getButtonState(FloatButton.LEFT_STICK_X);
//        float left_stick_y = movementController.getButtonState(FloatButton.LEFT_STICK_Y);
        telemetry.update();
    }


    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }


}
