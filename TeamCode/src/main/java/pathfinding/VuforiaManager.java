package pathfinding;

import android.graphics.Bitmap;
import android.util.Log;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.ThreadPool;
import com.vuforia.Frame;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.SwitchableCamera;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.jetbrains.annotations.TestOnly;

import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Date;
import java.util.Hashtable;
import java.util.List;
import java.util.Locale;

import frame_source.CameraInstance;
import frame_source.FrameManager;


/**
 * Generalized vuforia class
 * Wrapper class for vuforia functionality
 * Adapted from here:
 * https://github.com/FIRST-Tech-Challenge/FtcRobotController/blob/master/FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/ConceptVuforiaNavigation.java
 *  ^ comments in that file explain this a lot more in depth
 *  FTC coordinate system:
 *  https://acmerobotics.github.io/ftc-dashboard/official_field_coord_sys.pdf
 *
 *  2020-2021 targets
 *  https://firstinspiresst01.blob.core.windows.net/first-game-changers/ftc/navigation-images-us-letter.pdf
 *  found on the ftc resource page
 */
public class VuforiaManager {
    public static final String TAG = "vuf.test.vufObj";

    private VuforiaLocalizer vuforiaLocalizer;
    private OpenGLMatrix lastLocation = null;
    private final HardwareMap hardwareMap;
    private final VuforiaLocalizer.CameraDirection cameraDirection;
    private final FrameManager frameManager;
    private List<CameraInstance> availableCameras;

    private static final String vuforiaLicenseKey = "AVnPa5X/////AAABmUhfO30V7UvEiFRLKEAy25cwZ/uQDK2M0Z8GllUIhUOhFey2tkv1iKXqY4JdAjTHq4vlEUqn4F9sgeh+1ZiBsoPbGnSCdRnnHyQKmIU1hRoCyh24OvMfaG+6JQnpWlHorMoGWAqcEGt1+GXI9x3v2GLwooT1Dv/biDVn2DKar6tKms7EEEwIWkMN5YVaiQo53rbSSajpWuEROYYIrUrgzmgyorf4ngUWmjPrWHPES0OkUW6YVrZXoGT3Rwkiyl0Y7j5Rc5qT7iFBmI4v6E9udfPpnIsYrGzlhcL7GqHBntY8TuMYMTNIcklCO+ATWT4guojTwEOaNK+bVHG3XXxJsodhBK+Tbf7QX262rIbWvQto";

    private OpenGLMatrix phoneLocationOnRobot;

    private Hashtable<LocalizationTrackable, VuforiaTrackable> localizationTrackablesList;
    private Hashtable<LocalizationTrackable, VuforiaTrackableDefaultListener> listenersList;

    // robot measurements that need to get changed yearly
    private static final float inchesBotWidth = 18; // width of robot in inches. SET MANUALLY

    // constants for robot/field measurements
    private static final float mmPerInch        = 25.4f; // constant for mm to inches
    private static final float mmBotWidth       = inchesBotWidth * mmPerInch; // width of robot to mm

    private static final float mmFTCFieldWidth  = (12*12 - 2) * mmPerInch; // width of field in mm. SET MANUALLY
    // ^ idk whats happening with this calculation

    // 0,0 is a coordinate, keep that in mind for testing calculations
    private float mmFieldLength;

    private SwitchableCamera switchableCamera;

    /* BEGIN INITIALIZATIION */

    /**
     * Initialize the vuforia localizer, trackable locations, and everything else
     * @param hardwareMap the hardware map
     * @param useDisplay if true, then the camera stream is displayed on the robot
     */
    public VuforiaManager(HardwareMap hardwareMap, int mmFieldLength, boolean useDisplay) {
        this(hardwareMap, useDisplay);
        this.mmFieldLength = mmFieldLength;
        setPhoneLocation();
        initializeLocalizationTrackables();

    }

    /**
     * Initialize vuforia without initializing trackables. Used exclusively as a frame source
     * @param hardwareMap the hardware map
     */
    public VuforiaManager(HardwareMap hardwareMap) {
        this(hardwareMap, false);
    }

    /**
     * Initialize basic vuforia requirements
     * @param hardwareMap the hardware map to use
     * @param useDisplay if true, enables the display
     */
    public VuforiaManager(HardwareMap hardwareMap, boolean useDisplay) {
        this.hardwareMap = hardwareMap;
        frameManager = new FrameManager(hardwareMap);
        cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        initializeVuforia(useDisplay);
    }

    /**
     * Initialize the vuforia localizer object
     * @param useDisplay if true, the display is enabled on the robot with the camera stream
     */
    private void initializeVuforia(boolean useDisplay) {
        int cameraMonitorViewId = 0; // 0 indicates no display
        if (useDisplay)
            cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId",
                "id", hardwareMap.appContext.getPackageName());

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = vuforiaLicenseKey;
        parameters.useExtendedTracking = true; // disable for memory testing

        // set camera direction
        parameters.cameraDirection = cameraDirection;
        // get switchable camera and initialize vuforia
        // see FrameManager for info on this
//        availableCameras = frameManager.requestSwitchableCamera(new Function<SwitchableCameraName, VuforiaLocalizer>() {
//            @Override
//            public VuforiaLocalizer apply(SwitchableCameraName arg) {
//                parameters.cameraName = arg;
//                vuforiaLocalizer = ClassFactory.getInstance().createVuforia(parameters);
//                return vuforiaLocalizer;
//            }
//        });
        // switchable cameras if we want to do that
//        CameraName[] cameraNames = CameraInstance.getAvailableCameraNames()
//                .toArray(new CameraName[0]);
//        Log.d(TAG, "num of camera names: " + cameraNames.length);
//        parameters.cameraName = ClassFactory.getInstance().getCameraManager()
//                .nameForSwitchableCamera(cameraNames);

        vuforiaLocalizer = ClassFactory.getInstance().createVuforia(parameters);
//        switchableCamera = (SwitchableCamera) vuforiaLocalizer.getCamera();


        // instantiate the vuforia localizer

        // enable capturing frames from vuforia
        vuforiaLocalizer.enableConvertFrameToBitmap();
    }

    /**
     * Initialize the trackable objects
     * Custom datasets can be added here:
     * https://developer.vuforia.com/target-manager
     * See the comments in the source github (linked above this class) for more info
     *
     * Currently, this is just using the defaults from the source
     */
    private void initializeLocalizationTrackables() {
        localizationTrackablesList = new Hashtable<>();
        listenersList = new Hashtable<>();

        // custom datasets also use this, but you load a different thing obviously
        VuforiaTrackables localizationTrackables = vuforiaLocalizer.loadTrackablesFromAsset("UltimateGoal");

        // give basic information for each trackable
        List<LocalizationTrackable> trackableLabels = LocalizationTrackable.cachedValues();
        for (int i = 0; i < localizationTrackables.size(); i++) {
            // get each trackable's label from the LocalizationTrackable enum
            LocalizationTrackable label = trackableLabels.get(i);
            // get the corresponding target
            VuforiaTrackable target = localizationTrackables.get(i);
            // put it into the hashtable
            localizationTrackablesList.put(label, target);
            // set the name to the label from the enum
            target.setName(label.name());
            RobotLog.ii(TAG, "Target=%s", "target " + label.name() + " placed");
            // let the listener know where the phone is
            VuforiaTrackableDefaultListener listener = ((VuforiaTrackableDefaultListener)target.getListener());
            listenersList.put(label, listener);

            listener.setPhoneInformation(phoneLocationOnRobot, cameraDirection);

        }

        initializeTrackableMatrices();

        localizationTrackables.activate();
    }

    /**
     * Set an OpenGLMatrix for each localization trackable
     */
    private void initializeTrackableMatrices() {

        // set the location of target 1
        // this has to be manually done for all targets
        int halfFieldLength = (int)mmFieldLength/2;
        int quadFieldLength = halfFieldLength/2;

        VuforiaTrackable redWall = getLocalizationTrackable(LocalizationTrackable.RED_WALL);

        OpenGLMatrix redWallLocation = OpenGLMatrix
                .translation(0, -halfFieldLength, 0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XYZ,
                        AngleUnit.DEGREES, 90, 0, 180));
        redWall.setLocation(redWallLocation);
        RobotLog.ii(TAG, "%s=%s", redWall.getName(), format(redWallLocation));

        VuforiaTrackable blueWall = getLocalizationTrackable(LocalizationTrackable.BLUE_WALL);
        OpenGLMatrix blueWallLocation = OpenGLMatrix
                .translation(0, halfFieldLength, 0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XYZ,
                        AngleUnit.DEGREES, 90, 0, 0));
        blueWall.setLocation(blueWallLocation);
        RobotLog.ii(TAG, "%s=%s", blueWall.getName(), format(blueWallLocation));

        VuforiaTrackable redGoal = getLocalizationTrackable(LocalizationTrackable.RED_GOAL);
        OpenGLMatrix redGoalLocation = OpenGLMatrix
                .translation(halfFieldLength, -quadFieldLength, 0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XYZ,
                        AngleUnit.DEGREES, 90, 0, -90));
        redGoal.setLocation(redGoalLocation);
        RobotLog.ii(TAG, "%s=%s", redGoal.getName(), format(redGoalLocation));

        VuforiaTrackable blueGoal = getLocalizationTrackable(LocalizationTrackable.BLUE_GOAL);
        OpenGLMatrix blueGoalLocation = OpenGLMatrix
                .translation(halfFieldLength, quadFieldLength, 0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XYZ,
                        AngleUnit.DEGREES, 90, 0, -90));
        blueGoal.setLocation(blueGoalLocation);
        RobotLog.ii(TAG, "%s=%s", blueGoal.getName(), format(blueGoalLocation));

        VuforiaTrackable frontWall = getLocalizationTrackable(LocalizationTrackable.FRONT_WALL);
        OpenGLMatrix frontWallLocation = OpenGLMatrix
                .translation(-halfFieldLength, 0, 0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XYZ,
                        AngleUnit.DEGREES, 90, 0, 90));
        frontWall.setLocation(frontWallLocation);
        RobotLog.ii(TAG, "%s=%s", frontWall.getName(), format(frontWallLocation));
    }

    /**
     * Sets the position of the phone on the robot
     * SEE THE GITHUB FOR HOW TO DO THIS, ITS REALLY DETAILED
     * GO READ IT.
     *
     * https://github.com/FIRST-Tech-Challenge/FtcRobotController/blob/master/FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/ConceptVuforiaNavigation.java
     */
    private void setPhoneLocation() {
        // setting camera as the center of the robot for testing
        // also setting degrees to 0 for testing
        phoneLocationOnRobot = OpenGLMatrix
                .translation(0,0,0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 90, 90));

        RobotLog.ii(TAG, "phone=%s", format(phoneLocationOnRobot));
    }

    /* END INITIALIZATION */


    /**
     * Extracts positioning info from a transformation matrix and returns it as a readable string
     * Taken from the github
     * @param transformationMatrix The matrix to get info from
     * @return the information as a readable string
     */
    public static String format(OpenGLMatrix transformationMatrix) {
        String base = (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
        return String.format("{Orientation}, {Translation}: %s", base);
    }

    /**
     * Determines whether a given trackable is visible
     * @param item the trackable to check
     * @return whether the trackable is visible
     */
    public boolean isTrackableVisible(LocalizationTrackable item) {
        return listenersList.get(item).isVisible();
    }

    /**
     * Gets the position of the robot if there's a new position
     * @return the updated robot position
     */
    public OpenGLMatrix getUpdatedRobotPosition() {
        for (LocalizationTrackable item : LocalizationTrackable.cachedValues()) {
            if (isTrackableVisible(item)) {
                OpenGLMatrix currentPosition = listenersList.get(item).getUpdatedRobotLocation();
                if (currentPosition != null) {
                    lastLocation = currentPosition;
                }
                break; // stop once we have one
            }
        }
        return lastLocation;
    }

    /**
     * Gets the list of all trackables
     * @return the list of trackables as a hashtable with their names and values
     */
    public Hashtable<LocalizationTrackable, VuforiaTrackable> getLocalizationTrackablesList(){
        return localizationTrackablesList;
    }

    /**
     * Get a trackable from the trackables list
     * @param item the trackable to get
     * @return the trackable as a VuforiaTrackable object
     */
    public VuforiaTrackable getLocalizationTrackable(LocalizationTrackable item) {
        return localizationTrackablesList.get(item);
    }


    /**
     * Gets all locationalization trackables and returns them in a list
     * @return the list of localization trackables
     */
    public ArrayList<OpenGLMatrix> getLocTrackablesAsMatrices() {
        ArrayList<OpenGLMatrix> matrices = new ArrayList<>();
        for (VuforiaTrackable trackable : localizationTrackablesList.values()) {
            matrices.add(trackable.getLocation());
        }
        return matrices;

    }

    /**
     * Gets the vuforia localizer to be used in tensorflow
     * this function should ONLY be used for tfod, and nothing should directly edit the localizer
     * @return the vuforia localizer object
     */
    public VuforiaLocalizer getVuforiaLocalizer() {
        return vuforiaLocalizer;
    }


    /**
     * Gets a bitmap from vuforia and allows the caller to perform an operation with it
     * through a consumer
     */
    public void getBitmapFromFrame(final Consumer<Bitmap> bitmapConsumer) {
        vuforiaLocalizer.getFrameOnce(Continuation.create(ThreadPool.getDefault(), new Consumer<Frame>()
        {
            @Override public void accept(Frame frame)
            {
                Log.d(TAG, "frame null? " + (frame == null));
                if (frame != null) {
                    Bitmap bitmap = convertFrameToBitmap(frame);
                    bitmapConsumer.accept(bitmap);
                }
            }

        }));
    }

    /**
     * Converts a vuforia frame to a bitmap.
     * Literally a direct copy paste of {@link VuforiaLocalizer#convertFrameToBitmap(Frame)} so that
     * the pixel format priority can be changed when needed, since ImageView needs RGBA_88888
     * @param frame the frame to convert
     * @return a bitmap representing the frame
     */
    public static Bitmap convertFrameToBitmap(@NonNull Frame frame)
    {
        int[] pixelFormats = new int[] { PIXEL_FORMAT.RGB565, PIXEL_FORMAT.RGBA8888, };

        for (int pixelFormat : pixelFormats)
        {
            for (int i = 0; i < frame.getNumImages(); i++)
            {
                Image image = frame.getImage(i);
                if (image.getFormat() == pixelFormat)
                {
                    Bitmap.Config config;
                    switch (pixelFormat)
                    {
                        case PIXEL_FORMAT.RGBA8888: config = Bitmap.Config.ARGB_8888; break;
                        case PIXEL_FORMAT.RGB565: config = Bitmap.Config.RGB_565; break;
                        default:
                            continue;
                    }

                    Bitmap bitmap = Bitmap.createBitmap(image.getWidth(), image.getHeight(), config);
                    bitmap.copyPixelsFromBuffer(image.getPixels());
                    return bitmap;
                }
            }
        }

        return null;
    }

    /**
     * Save frame of camera through vuforia and save to /storage/emulated/0/FIRST/data
     */
    public void captureFrameToFile() {
        getBitmapFromFrame(new Consumer<Bitmap>() {
            @Override
            public void accept(Bitmap value) {
                Log.d(TAG, "bitmap null? " + (value == null));
                Log.d(TAG, "converted frame to bitmap");
                String timeStamp = new SimpleDateFormat("dd-HH-mm-ss", Locale.US).format(new Date());
                Log.d(TAG, "time string: " + timeStamp);
                Visuals.captureBitmapToFile(value, String.format("cam/camera_view_%s", timeStamp));
            }
        });
    }

    /**
     * Switches the vuforia camera to the provided camera instance, if its available
     * this function is for testing only at this stage
     * @param newCamera the camera to switch to
     */
    @TestOnly
    private void switchCamera(CameraInstance newCamera) {
//        if (newCamera.isAvailable())
//            frameManager.switchSwitchableCamera(newCamera);
        if (newCamera.isAvailable()) {
            Log.d(TAG, "Vuforia switching camera to " + newCamera.name());
            switchableCamera.setActiveCamera(newCamera.getWebcamName());
        }
    }







    // enum of targets to make everything easier
    // add new trackables to here first then add the trackable object to the hashtable
    // note, this has to be changed to be in order of the targets in the vuforia xml database
    public enum LocalizationTrackable {
        BLUE_GOAL,
        RED_GOAL,
        RED_WALL,
        BLUE_WALL,
        FRONT_WALL;

        // this is used to avoid memory issues when repeatedly iterating over enums
        // since a new copy of the array is returned each time .values() is called
        private static final List<LocalizationTrackable> cachedList =
                Arrays.asList(LocalizationTrackable.values());

        public static List<LocalizationTrackable> cachedValues() {
            return cachedList;
        }
        // consider adding fake .values method to mess up attempted calls
//        public static List<LocalizationTrackable> values() {
//            return cachedList;
//        }
    }


}
