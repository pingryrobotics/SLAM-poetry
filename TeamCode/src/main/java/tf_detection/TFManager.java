package tf_detection;

import android.graphics.Bitmap;
import android.util.Log;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ThreadPool;

import org.firstinspires.ftc.robotcore.external.function.Consumer;

import java.util.List;
import java.util.concurrent.ExecutorService;

import pathfinding.CoordinateUtils;
import pathfinding.Visuals;
import pathfinding.VuforiaManager;


// sourced from https://github.com/FIRST-Tech-Challenge/FtcRobotController/blob/master/FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/ConceptTensorFlowObjectDetection.java

/**
 * Class for managing tensorflow object detection, as well as determining distances to detections
 * Note: to get more information on Recognition objects, look at the class file (command b on
 * android studio, or just go to declaration). You can get the label,
 * confidence, estimated angle, coordinate in the image, and other things
 *
 *
 * @ FIXME: 8/11/21 tf manager is currently dependant on vuforia for the frame source. it needs vuforia to switch
 *      cameras, which impedes our ability to view multiple angles at once for different things.
 *      potential fix: different vuforia and tfod for each camera, switch managers between them
 *      depending on necessary angle. probably plausible, but might be slower
 *
 * @ FIXME: 8/11/21 running into memory issues when using tfod and vuforia at once (or really when
 *      having them both fully initialized at once, havent tried actually using them both)
 *
 */

/*
 * Object distances information (separated to be collapseable):
 *
 *  Essentially, object distances are determined using trigonometry and the angle of depression to the
 *  base of a recognition. Tfod does not provide vertical angles of depression to recognitions,
 *  it only provides them horizontally. Due to this, we need to manually calculate this using
 *  the camera's angle of view. See https://en.wikipedia.org/wiki/Angle_of_view for in depth info.
 *
 *  Essentially the camera's angle of view is just what angle range it can see. Humans can see
 *  about 135 degrees horizontally, which is the horizontal angle of view, and about 180 degrees
 *  vertically, i.e. the vertical angle of view.
 *
 *  To find the angle of depression, we need to know the vertical angle of view, which I calculated
 *  using this guide: https://chucklohr.com/808/AOV/ . Specifically, I used the tape measure
 *  and spreadsheet part. This can be used to determine the angle of view by figuring out how
 *  wide the camera can see from a set distance, because the relative size of an object
 *  changes as you get closer and farther from the object. This calculation provides us with both
 *  the horizontal and vertical angles of view.
 *
 *  Once we have these, we can calculate the distance to an object. Our camera needs to be at a
 *  known, fixed height and pointing completely straight. Then, we can take a picture of an object
 *  and assuming we have an image resolution of 1280x720 (which I did for every calculation), then
 *  we can find the y coordinate of the bottom of the object in the image, in pixels. For example,
 *  if we take a picture of a water bottle, and the bottom of the bottle is located at 450x300,
 *  then our y coordinate is 300.
 *
 *  With the vertical angle of view, the center of the image corresponds to an angle of 0. If we look
 *  slightly below the center, then the angle of depression is a bit above 0, maybe at 1 or 2 degrees.
 *  Using the angle of view in conjunction with the image resolution, we can estimate a
 *  pixel per angle value by dividing the image height by the vertical angle of view. The same can
 *  be done with the horizontal angle of view and the image width to get a horizontal pixel per angle
 *  value.
 *
 *  Now, we know our y coordinate is 300 and we have our vertical pixel per angle value. However,
 *  we need to know our y's offset from the center, so lets take the absolute value of 360-300.
 *  So, for this example, 60 is our angle of depression from the camera to the bottom of the water bottle.
 *
 *  Keep in mind that this whole calculation revolves around creating a triangle whose sides we can
 *  solve for. We now have an angle of depression to the object, and our fixed camera height.
 *  Remember, the angle of depression is how far you need to angle your head down to see an object,
 *  starting from staring straight ahead. So, to get a triangle that we can use, we can take the
 *  complement of this angle and find the angle between looking at the object and looking
 *  straight at the ground.
 *  The complement of an angle is just abs(90-angle), so our angle from the camera to the ground
 *  is now 30. Now, we have a triangle with an angle and a side length, which is our
 *  camera height. If we take this angle of elevation as theta, then our camera height is our
 *  adjacent side and the distance to the object is our opposite side.
 *  So, we can do tan(theta) = o/a, then o/a * a, or just a * tan(theta) to get the adjacent side's
 *  length, which is our distance to the object from the camera. woo.
 *
 * Now, we have the distance we would have to move straight to be in line with the object, but
 * we still need the distance we'd need to move to the side to actually get to the object. This
 * is very similar to the previous calculation, except instead of using the vertical pixels per angle,
 * we use the horizontal. Essentially, what we're doing is constructing a triangle with theta as
 * the angle between looking straight ahead and looking at the object, the hypotenuse being the
 * line from our camera to the object, the adjacent side being our previously calculated straight
 * distance, and the opposite side being our main target, which is the distance we need to move to
 * the side to be on top of the object after moving straight.
 *
 * First, we find the pixel (or pixel y value, to be specific) where the center of the object is
 * located on the image, then find the offset from the center. Then, we calculate the horizontal
 * pixels per angle by taking the image width and dividing it by our horizontal angle of view.
 * Now, we can divide our pixel offset by the pixels per angle and get our angle to the object.
 * Once we have this, we can use tan to get the opposite/adjacent ratio, which we can then
 * multiply by our known adjacent side (the straight distance) and get our side distance to the object.
 *
 */


public class TFManager {

    private static final String TAG = "vuf.test.tfmanager";

    // tensorflow min confidence to be considered a recognition
    private static final float minResultConfidence = 0.8f;
    public final double imageHeight; // image height, in pixels
    public final double imageWidth; // image width, in pixels
    private final ExecutorService detectionThreads = ThreadPool.newFixedThreadPool(
            5,
            "teamcode.detector");
    private final DetectorType detectorType;
    private final VuforiaManager vuforiaManager;
    // tfod is the main tensorflow object detection object
    private final HardwareMap hardwareMap;
    private int skippedFrames;
    private CustomDetector customDetector;

    /**
     * Initialize a custom object detector engine
     * @param hardwareMap the hardware map for the robot
     * @param vuforiaManager the vuforia manager to assist in retrieving frames
     * @param detectorType the object detector {@link DetectorType} to use
     * @param useDisplay if true, displays the frames from the object detector on screen
     */
    public TFManager(
            @NonNull HardwareMap hardwareMap,
            @NonNull VuforiaManager vuforiaManager,
            @NonNull DetectorType detectorType, boolean useDisplay)
             {
        this.hardwareMap = hardwareMap;
        this.vuforiaManager = vuforiaManager;
        this.detectorType = detectorType;
        initCustomDetector(useDisplay);

        this.imageHeight = customDetector.getImageHeight();
        this.imageWidth = customDetector.getImageWidth();
    }



    /**
     * initialize the custom detector based on its type
     */
    private void initCustomDetector(boolean useDisplay) {
        int displayId = 0; // display of 0 means dont display
        if (useDisplay) { // we ue the tensorflow monitor because it'll only be in use by object detectors
            displayId = hardwareMap.appContext.getResources().getIdentifier(
                    "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        }
        // using a switch for expandability
        switch (detectorType) {
            case ML_Kit:
                customDetector = new MLKit_Detector(0, displayId);
                break;
            case FTC_TFOD:
                customDetector = new FTC_Detector(minResultConfidence, displayId,
                        vuforiaManager.getVuforiaLocalizer(),
                        FTC_Detector.ModelConfig.ROVER_RUCKUS);
                break;
            case CAMERA_STREAM:
                customDetector = new CameraStream(displayId);
                break;
        }
        // run initialization code for all of them
        customDetector.initializeDetector();
    }

    /**
     * Updates the object detector if it needs to be updated
     *
     * @note FTC tfod wont ever update through this method
     */
    public void updateDetector() {
        // if the detector doesnt need a new frame, dont supply one
        // FTC tfod will always return false
        if (!customDetector.needsNewFrame()) {
            skippedFrames++;
            if (skippedFrames % 10 == 0)
                Log.d(TAG, "Skipped " + skippedFrames + " frames");
            return;
        }
        skippedFrames = 0;
        // since getting the bitmap is an async operation, we need to also be async
        vuforiaManager.getBitmapFromFrame(new Consumer<Bitmap>() {
            @Override
            public void accept(final Bitmap value) {
                detectionThreads.execute(new Runnable() {
                    @Override
                    public void run() {
                        // get the bitmap then start a thread to predict
                        customDetector.predict(value);
                    }
                });

            }
        });
    }

    /**
     * Gets the latest detections
     * @return a list of the latest detections
     */
    @NonNull
    public List<Detection> getLatestDetections() {
        return customDetector.getLatestDetections();
    }

    /**
     * Returns the a list of detections if the list is different from the last call to
     * this method. Otherwise, returns false
     * @return the list of detections, or null if no changes have occurred
     */
    @Nullable
    public List<Detection> getUpdatedDetections() {
        return customDetector.getUpdatedDetections();
    }

    /**
     * Log information about the latest detections and save an image of the bounding boxes and labels drawn
     * onto the latest image
     * No image will be saved for FTC_TFOD because bitmaps arent used through tfmanager
     */
    public void logDetectionInfo() {
        List<Detection> latestDetections = customDetector.getLatestDetections();
        Log.d(TAG, "Total detections: " + latestDetections.size());
        for (Detection detectedObject : latestDetections) {
            Log.d(TAG, "Label: " + detectedObject.getLabel());
            Log.d(TAG, "Bounding box: " + CoordinateUtils.rectToString(detectedObject.getRectF()));
        }
        Bitmap resultBitmap = customDetector.getLatestBitmap();
        if (resultBitmap != null) {
            Visuals.drawMLPredictions(resultBitmap, latestDetections);
            Visuals.captureBitmapToFile(resultBitmap, "ML_Predictions");
        } else {
            Log.d(TAG, "No bitmap available to visualize");
        }

    }


    /**
     * Gets the most recent number of recognitions
     * @return The total number of recognitions
     */
    public int getDetectionCount() {
        return customDetector.getLatestDetections().size();
    }


    /**
     * Enum of custom detector types available for use
     */
    public enum DetectorType {
        ML_Kit,
        FTC_TFOD,
        CAMERA_STREAM,
    }




}
