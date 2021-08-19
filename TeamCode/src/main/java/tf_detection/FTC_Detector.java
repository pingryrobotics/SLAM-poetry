package tf_detection;

import android.graphics.Bitmap;
import android.graphics.RectF;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.List;

public class FTC_Detector implements CustomDetector {

    private static final int imageHeight = 720;
    private static final int imageWidth = 1280;
    private final float minResultConfidence;
    private final VuforiaLocalizer vuforia;
    private final ModelConfig modelConfig;
    private final int displayId;
    private TFObjectDetector tfod;

    /**
     * Initialize the ftc object detector
     * @param minResultConfidence the minimum confidence to consider a successful prediction
     * @param displayId the monitor display to use. for no display, set to 0
     * @param vuforia the vuforia localizer
     * @param modelConfig the model configuration to use (i.e. which assets to use)
     */
    public FTC_Detector(float minResultConfidence, int displayId, @NonNull VuforiaLocalizer vuforia,
                        ModelConfig modelConfig) {
        this.minResultConfidence = minResultConfidence;
        this.displayId = displayId;
        this.vuforia = vuforia;
        this.modelConfig = modelConfig;
    }

    /**
     * Initialize the object detector
     */
    @Override
    public void initializeDetector() {
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(displayId);
        tfodParameters.minResultConfidence = minResultConfidence;
//        tfodParameters.useObjectTracker = false; // disabling because memory issues
//        tfodParameters.tfodMonitorViewParent = null; // disabling because memory issues
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(modelConfig.tflite_asset, modelConfig.first_label, modelConfig.second_label);
        tfod.activate();
    }

    /**
     * Convert recognitions into detection objects
     * @param recognitionList the recognitions to convert
     * @return a list of detections from the recognitions
     */
    @NonNull
    private List<Detection> convertRecognitionToDetection(@NonNull List<Recognition> recognitionList) {

        ArrayList<Detection> detectionList = new ArrayList<>();
        for (Recognition recognition : recognitionList) {
            RectF rectf = new RectF(recognition.getLeft(), recognition.getTop(),
                    recognition.getRight(), recognition.getBottom());
            detectionList.add(new Detection(rectf, recognition.getLabel(), recognition.getConfidence()));
        }
        return detectionList;
    }

    /**
     * Get the most updated predictions, if the result is different than the last call to
     * getUpdatedPredictions(). Otherwise, returns null
     *
     * @return the predictions if they're different, or null
     */
    @Nullable
    @Override
    public List<Detection> getUpdatedDetections() {
        List<Recognition> recognitionList = tfod.getUpdatedRecognitions();
        if (recognitionList == null)
            return null;
        return convertRecognitionToDetection(recognitionList);
    }

    /**
     * Gets the latest predictions from the object detector
     *
     * @return a list of the latest predictions
     */
    @NonNull
    @Override
    public List<Detection> getLatestDetections() {
        List<Recognition> recognitionList = tfod.getRecognitions();
        if (recognitionList == null)
            recognitionList = new ArrayList<>();

        return convertRecognitionToDetection(recognitionList);
    }

    /**
     * This method does absolutely nothing with the ftc detector since it works through vuforia
     *
     * @param bitmap the bitmap to predict on, but this method does nothing
     */
    @Override
    public void predict(@NonNull Bitmap bitmap) { }


    /**
     * Gets the last bitmap that was successfully predicted on.
     *
     * @return always returns null because ftc tfod works through vuforia
     */
    @Override
    @Nullable
    public Bitmap getLatestBitmap() {
        return null;
    }

    /**
     * Check if the detector needs a new frame before supplying to avoid memory issues
     * FTC detector will always be false because it never needs to be updated
     *
     * @return true if the detector can handle a new frame, otherwise false
     */
    @Override
    public boolean needsNewFrame() {
        return false;
    }

    public int getImageHeight() { return imageHeight; }
    public int getImageWidth() { return imageWidth; }

    /**
     * Enum of ftc model assets, mostly previous years, to use with tfod
     */
    enum ModelConfig {
        ULTIMATE_GOAL("UltimateGoal.tflite", "Quad", "Single"),
        SKYSTONE("Skystone.tflite", "Stone", "Skystone"),
        ROVER_RUCKUS("RoverRuckus.tflite", "Gold Mineral", "Silver Mineral");

        public final String tflite_asset;
        public final String first_label;
        public final String second_label;

        ModelConfig(@NonNull String tflite_asset, @NonNull String first_label, @NonNull String second_label) {
            this.tflite_asset = tflite_asset;
            this.first_label = first_label;
            this.second_label = second_label;
        }

    }
}
