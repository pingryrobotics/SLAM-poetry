package opmodes;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.GamepadController;
import org.firstinspires.ftc.teamcode.GamepadController.ButtonState;
import org.firstinspires.ftc.teamcode.GamepadController.ToggleButton;

import java.util.List;

import tf_detection.Detection;
import tf_detection.TFManager;
import localization.VuforiaManager;


@TeleOp(name="TF: MLKit OpMode", group="Testing")
public class MLKitOpMode extends OpMode {
    // tag is used in logcat logs (Log.d()) to identify where the log is coming from
    // logcat is basically like System.out.print (standard output) except through adb
    private static final String TAG = "teamcode.test_opmode"; // put the name of the opmode

    // put any outside classes you need to use here
    private GamepadController movementController;
    private GamepadController mechanismController;

    private TFManager tfManager;
    private VuforiaManager vuforiaManager;


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
        vuforiaManager = new VuforiaManager(hardwareMap, fieldLength, false);
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
        tfManager.updateDetector();

        // do something when A is pressed
        if (movementController.getButtonState(ToggleButton.A) == ButtonState.KEY_DOWN) {
            tfManager.logDetectionInfo();

        }

        // do something when B is pressed
        if (movementController.getButtonState(ToggleButton.B) == ButtonState.KEY_DOWN) {
            Log.d(TAG, "button b pressed");
            tfManager.getLatestDetections();
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

        for (VuforiaManager.ImageTarget key : VuforiaManager.ImageTarget.cachedValues()) {
            telemetry.addData(key.name(),
                    vuforiaManager.isTrackableVisible(key) ? "Visible" : "Not Visible");
        }




    }


    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }


}
