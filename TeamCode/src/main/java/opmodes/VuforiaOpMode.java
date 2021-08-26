package opmodes;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.teamcode.GamepadController;
import org.firstinspires.ftc.teamcode.GamepadController.ButtonState;
import org.firstinspires.ftc.teamcode.GamepadController.ToggleButton;

import java.util.Hashtable;

import localization.VuforiaManager;


@TeleOp(name="Vuforia: Vuforia OpMode", group="Testing")
public class VuforiaOpMode extends OpMode {
    // field declarations
    private static final String TAG = "vuf.test.vuforiaOp";
    private GamepadController movementController;
    private GamepadController mechanismController;
    private VuforiaManager vuforiaManager;
    private int mmFieldLength = 301;

    private Hashtable<VuforiaManager.ImageTarget, Telemetry.Item> trackableLogs;





    @Override
    public void init() {
        movementController = new GamepadController(gamepad1);
        vuforiaManager = new VuforiaManager(hardwareMap, 301, false);
        trackableLogs = new Hashtable<>();
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {

        for (VuforiaManager.ImageTarget trackable : VuforiaManager.ImageTarget.cachedValues()) {
            trackableLogs.put(trackable, telemetry.addData(trackable.name(), ""));
        }

    }

    @Override
    public void loop() {

        runControls();
    }

    /**
     * Does controls for the gamepads
     */
    public void runControls() {

//        boolean visible = vuforiaObj.isTrackableVisible(Vuforia.LocalizationTrackable.FIRST);
//        if (visible != wasVisible) {
//            Log.d("vuf.test", "Visible: " + visible);
//            wasVisible = visible;
//        }



        for (VuforiaManager.ImageTarget key : VuforiaManager.ImageTarget.cachedValues()) {
            trackableLogs.get(key)
                    .setValue(vuforiaManager.isTrackableVisible(key) ? "Visible" : "Not Visible");
        }



        movementController.updateButtonStates();
        if (movementController.getButtonState(ToggleButton.A) == ButtonState.KEY_DOWN) {
            Log.d(TAG, "saving frame");
            vuforiaManager.captureFrameToFile();
        }

        if (movementController.getButtonState(ToggleButton.B) == ButtonState.KEY_HOLD) {
            OpenGLMatrix location = vuforiaManager.getUpdatedRobotPosition();
            if (location != null) {
                  telemetry.addData("Last robot position: ", VuforiaManager.format(location));
            }
        }

//        if (movementController.getButtonState(ToggleButton.X) == ButtonState.KEY_DOWN) {
//
//
//        }


//        float left_stick_x = movementController.getButtonState(FloatButton.LEFT_STICK_X);
//        float left_stick_y = movementController.getButtonState(FloatButton.LEFT_STICK_Y);
//        telemetry.update();
    }


    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }


}
