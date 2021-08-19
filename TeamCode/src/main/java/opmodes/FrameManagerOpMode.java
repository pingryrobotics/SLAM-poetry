package opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.GamepadController;
import org.firstinspires.ftc.teamcode.GamepadController.ButtonState;
import org.firstinspires.ftc.teamcode.GamepadController.ToggleButton;

import frame_source.CameraInstance;
import pathfinding.VuforiaManager;
import tf_detection.TFManager;


@TeleOp(name="FrameManager: FM OpMode", group="Testing")
public class FrameManagerOpMode extends OpMode {
    // tag is used in logcat logs (Log.d()) to identify where the log is coming from
    // logcat is basically like System.out.print (standard output) except through adb
    private static final String TAG = "teamcode.fm_opmode"; // put the name of the opmode

    // put any outside classes you need to use here
    private GamepadController movementController;


    // put any measurements here
    private final double inchesToMM = 25.4; // this is correct
    private final double toCameraCenter = 1.25; // inches from bottom of logitech c615 to actual camera
    private final double cameraPlatform = 10.5; // random value
    private final double cameraHeight = (cameraPlatform + toCameraCenter) * inchesToMM;
    private static final int fieldLength = 3660; // mm (this is correct)
    private TFManager tfManager;
    private VuforiaManager vuforiaManager;



    // code to run once when driver hits init on phone
    @Override
    public void init() {
        movementController = new GamepadController(gamepad1);
//        vuforiaManager = new VuforiaManager(hardwareMap, fieldLength, true);
        tfManager = new TFManager(hardwareMap, cameraHeight, null,
                TFManager.DetectorType.CAMERA_STREAM, true);
        tfManager.startCameraStream(CameraInstance.FRONT);
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

//        for (VuforiaManager.LocalizationTrackable key : VuforiaManager.LocalizationTrackable.cachedValues()) {
//            telemetry.addData(key.name(), vuforiaManager.isTrackableVisible(key) ? "Visible" : "Not Visible");
//        }

        // button states need to be updated each loop for controls to work
        movementController.updateButtonStates();
        tfManager.updateCameraStream();
        if (movementController.getButtonState(ToggleButton.A) == ButtonState.KEY_DOWN) {

        }

        if (movementController.getButtonState(ToggleButton.RIGHT_TRIGGER) == ButtonState.KEY_HOLD) {

//            tfManager.switchCamera(CameraInstance.FRONT);
        }

        if (movementController.getButtonState(ToggleButton.X) == ButtonState.KEY_DOWN) {
            tfManager.switchCamera(CameraInstance.FRONT);
        }
//
        if (movementController.getButtonState(ToggleButton.Y) == ButtonState.KEY_DOWN) {
            tfManager.switchCamera(CameraInstance.BACK);
        }



//        if (movementController.getButtonState(ToggleButton.DPAD_UP) == ButtonState.KEY_DOWN) {
//            vuforiaManager.switchCamera(CameraInstance.FRONT);
//        }
//
//        if (movementController.getButtonState(ToggleButton.DPAD_DOWN) == ButtonState.KEY_DOWN) {
//            vuforiaManager.switchCamera(CameraInstance.BACK);
//        }

    }


    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }


}
