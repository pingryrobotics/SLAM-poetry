package opmodes;

import static android.content.Context.ACTIVITY_SERVICE;

import android.app.ActivityManager;
import android.graphics.Bitmap;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.GamepadController;
import org.firstinspires.ftc.teamcode.GamepadController.ButtonState;
import org.firstinspires.ftc.teamcode.GamepadController.ToggleButton;

import java.util.ArrayList;
import java.util.List;

import display.Visuals;
import tf_detection.Detection;
import tf_detection.TFManager;
import localization.VuforiaManager;


@TeleOp(name="TF: Task Lib OpMode", group="Testing")
public class StressTestOpMode extends OpMode {
    // tag is used in logcat logs (Log.d()) to identify where the log is coming from
    // logcat is basically like System.out.print (standard output) except through adb
    private static final String TAG = "teamcode.taskLibOpmode"; // put the name of the opmode

    // put any outside classes you need to use here
    private GamepadController movementController;
    private GamepadController mechanismController;

    private TFManager tfManager;
    private VuforiaManager vuforiaManager;
    private final ArrayList<Bitmap> bitmapList = new ArrayList<>();


    // put any measurements here
    private final double inchesToMM = 25.4; // this is correct
    private final double toCameraCenter = 1.25; // inches from bottom of logitech c615 to actual camera
    private final double cameraPlatform = 10.5; // random value
    private final double cameraHeight = (cameraPlatform + toCameraCenter) * inchesToMM;
    private static final int fieldLength = 3660; // mm (this is correct)



    // code to run once when driver hits init on phone
    @Override
    public void init() {
        movementController = new GamepadController(gamepad1);
        vuforiaManager = new VuforiaManager(hardwareMap);
        tfManager = new TFManager(hardwareMap, vuforiaManager,
                TFManager.DetectorType.ML_Kit, true);

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


        // do something when A is pressed
        if (movementController.getButtonState(ToggleButton.A) == ButtonState.KEY_DOWN) {
            tfManager.logDetectionInfo();

        }

        // do something when B is pressed
        if (movementController.getButtonState(ToggleButton.B) == ButtonState.KEY_DOWN) {
            Log.d(TAG, "button b pressed");
            tfManager.updateDetector();
        }

        List<Detection> updatedDetections = tfManager.getUpdatedDetections();
        // get updated recognitions, but only if there's changes
        if (updatedDetections != null) {
            for (Detection detection : updatedDetections) {
                telemetry.addData(detection.getLabel(),
                        "Top: %s, Bottom: %s, Left: %s, Right: %s ",
                        detection.getTop(), detection.getBottom(), detection.getLeft(), detection.getRight());
            }
        }

//        for (VuforiaManager.LocalizationTrackable key : VuforiaManager.LocalizationTrackable.cachedValues()) {
//            telemetry.addData(key.name(),
//                    vuforiaManager.isTrackableVisible(key) ? "Visible" : "Not Visible");
//        }




    }


    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }


}
