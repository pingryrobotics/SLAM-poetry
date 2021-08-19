package memory_tests;

/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.teamcode.GamepadController;

import java.util.Hashtable;

import pathfinding.VuforiaManager;


@TeleOp(name="Linear Vuforia Opmode", group ="Memory")
@Disabled
public class LinearVuforiaOpMode extends LinearOpMode {

    private static final String TAG = "vuf.test.vuforiaOp";
    private GamepadController movementController;
    private GamepadController mechanismController;
    private VuforiaManager vuforiaManager;

    private Hashtable<VuforiaManager.LocalizationTrackable, Telemetry.Item> trackableLogs;



    @Override public void runOpMode() {

        movementController = new GamepadController(gamepad1);
        vuforiaManager = new VuforiaManager(hardwareMap, 301, false);
        trackableLogs = new Hashtable<>();

//        VuforiaManager vuforiaManager = new VuforiaManager(hardwareMap);
        telemetry.addData("Waiting", "waiting for start");

        waitForStart();

        for (VuforiaManager.LocalizationTrackable trackable : VuforiaManager.LocalizationTrackable.cachedValues()) {
            trackableLogs.put(trackable, telemetry.addData(trackable.name(), ""));
        }

        while (!isStopRequested()) {

            runControls();

        }
    }

    public void runControls() {

//        boolean visible = vuforiaObj.isTrackableVisible(Vuforia.LocalizationTrackable.FIRST);
//        if (visible != wasVisible) {
//            Log.d("vuf.test", "Visible: " + visible);
//            wasVisible = visible;
//        }



        for (VuforiaManager.LocalizationTrackable key : VuforiaManager.LocalizationTrackable.cachedValues()) {
            /*
             * getUpdatedRobotLocation() will return null if no new information is available since
             * the last time that call was made, or if the trackable is not currently visible.
             * getRobotLocation() will return null if the trackable is not currently visible.
             */

            trackableLogs.get(key)
                    .setValue(vuforiaManager.isTrackableVisible(key) ? "Visible" : "Not Visible");
//            telemetry.addData(trackable.getName(), ((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible() ? "Visible" : "Not Visible");
        }



        movementController.updateButtonStates();
        if (movementController.getButtonState(GamepadController.ToggleButton.A) == GamepadController.ButtonState.KEY_DOWN) {
            Log.d(TAG, "saving frame");
            vuforiaManager.captureFrameToFile();
        }

        if (movementController.getButtonState(GamepadController.ToggleButton.B) == GamepadController.ButtonState.KEY_HOLD) {
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

}