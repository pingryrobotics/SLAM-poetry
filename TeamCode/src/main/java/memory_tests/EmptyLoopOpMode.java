package memory_tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name="Loop OpMode", group="Memory")
@Disabled
public class EmptyLoopOpMode extends OpMode {



    // code to run once when driver hits init on phone
    @Override
    public void init() {
//        VuforiaManager vuforiaManager = new VuforiaManager(hardwareMap);
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
//        telemetry.update();
    }

    /**
     * Updates buttons and does controls when buttons are pressed
     */
    public void runControls() {
        telemetry.addData("caption", "value");
    }


    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }


}
