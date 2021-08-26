package tf_detection;

import android.graphics.Bitmap;
import android.util.Log;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.google.android.gms.tasks.OnFailureListener;
import com.google.android.gms.tasks.OnSuccessListener;
import com.google.mlkit.vision.common.InputImage;
import com.google.mlkit.vision.objects.DetectedObject;
import com.google.mlkit.vision.objects.ObjectDetection;
import com.google.mlkit.vision.objects.ObjectDetector;
import com.google.mlkit.vision.objects.defaults.ObjectDetectorOptions;

import java.util.ArrayList;
import java.util.List;

import display.DisplaySource;
import display.Visuals;

/**
 * Class for object detection with Google's ML Kit
 * https://developers.google.com/ml-kit/vision/object-detection/android
 */
public class MLKit_Detector extends DisplaySource implements CustomDetector {
    public static final String TAG = "vuf.test.ml_detector";

    private ObjectDetector objectDetector;
    private boolean newDetectionsAvailable;
    private Bitmap latestResultBitmap;
    private static final long nanoToMilli = 1000000;
    private final float minimumConfidence;
    private long latestResultStartTime; // time when the last successful detections' inference started

    private final boolean useDisplay;

    private int predictionQueueLength;
    private final static int maxQueue = 5;

    private final static int imageHeight = 360;
    private final static int imageWidth = 640;



    @NonNull
    private List<DetectedObject> latestDetections;

    /**
     * create a new object detector
     * @param minResultConfidence the minimum confidence to count as a detection
     * @param monitorViewIdParent the monitor id to use for displaying predictions. If set to 0,
     *                            no display will be used
     */
    public MLKit_Detector(float minResultConfidence, int monitorViewIdParent) {
        super(monitorViewIdParent);
        this.minimumConfidence = minResultConfidence;
        latestDetections = new ArrayList<>(); // for non nullability
        newDetectionsAvailable = false;
        useDisplay = (monitorViewIdParent != 0); // true if it isn't 0
        predictionQueueLength = 0;
    }

    /**
     * Initialize the object detector
     */
    public void initializeDetector() {
        // Multiple object detection in static images
        ObjectDetectorOptions options =
                new ObjectDetectorOptions.Builder()
                        .setDetectorMode(ObjectDetectorOptions.STREAM_MODE)
                        .enableMultipleObjects()
                        .enableClassification()  // Optional
                        .build();

        objectDetector = ObjectDetection.getClient(options);
    }


    /**
     * Makes predictions for the provided bitmap
     * @param bitmap the bitmap image to make predictions on
     */
    public void predict(@NonNull Bitmap bitmap) {
        // only predict if the queue isnt maxed out
        if (needsNewFrame()) {
            predictionQueueLength++;
            bitmap = Bitmap.createScaledBitmap(bitmap, imageWidth, imageHeight, true);
            Log.d(TAG, "Predicting");
            InputImage image = InputImage.fromBitmap(bitmap, 0);
            objectDetector.process(image)
                    .addOnSuccessListener(
                            getOnSuccessListener(bitmap))
                    .addOnFailureListener(
                            getOnFailureListener());
        }

    }


    /**
     * Check if the detector needs a new frame. If there's too many items in the queue, then
     * dont supply a new frame
     * @return true if the detector can handle a new frame, otherwise false
     */
    public boolean needsNewFrame() {
        return (maxQueue > predictionQueueLength);
    }

    /**
     * Converts from DetectedObjects to detections
     * Detections are only added to the final list if they're above the minimum confidence
     * @param predictionList the detected objects to convert
     * @return the detections as Detection objects
     */
    @NonNull
    protected List<Detection> convertToDetection(@NonNull List<?> predictionList) {
        ArrayList<Detection> detectionList = new ArrayList<>();

        for (Object object : predictionList) {
            DetectedObject detectedObject = (DetectedObject) object;
            // get highest label
            DetectedObject.Label bestLabel = getBestLabel(detectedObject.getLabels());
            // check if its above minimum confidence
            if (bestLabel.getConfidence() >= minimumConfidence) {
                // make a new detection
                detectionList.add(new Detection(
                        detectedObject.getBoundingBox(),
                        bestLabel.getText(), bestLabel.getConfidence()));
            }
        }
        return detectionList;
    }

    /**
     * Get the most confident label out of the list of labels. If there are no labels,
     * a label with text "No label" and a confidence of 0 is returned
     * @param labelList the list of labels
     * @return the highest scored label
     */
    @NonNull
    private static DetectedObject.Label getBestLabel(@NonNull List<DetectedObject.Label> labelList) {
        DetectedObject.Label highestLabel = new DetectedObject.Label("No label", 0, -1);

        for (DetectedObject.Label label : labelList) {
            if (label.getConfidence() > highestLabel.getConfidence())
                highestLabel = label;
        }

        return highestLabel;
    }

    /**
     * Returns the latest detections, but only if they're different from the last time this
     * method was called or if there haven't been any detections made
     * @return the latest detections, or null if they're the same as last time or non existent
     */
    @Nullable
    @Override
    public List<Detection> getUpdatedDetections() {
        if (!newDetectionsAvailable) {
            return null;
        } else {
            newDetectionsAvailable = false;
            return convertToDetection(latestDetections);
        }
    }

    /**
     * Gets the latest bitmap that detections were successful on
     * @return the latest bitmap
     */
    @Override
    public Bitmap getLatestBitmap() {
        return latestResultBitmap;
    }

    /**
     * Gets the latest detections
     * @return the latest detections
     */
    @NonNull
    public List<Detection> getLatestDetections() {
        return convertToDetection(latestDetections);
    }

    public int getImageHeight() { return imageHeight; }
    public int getImageWidth() { return imageWidth; }

    /**
     * Gets a listener with code to be run when inference fails
     * @return a new failure listener
     */
    @NonNull
    private OnFailureListener getOnFailureListener() {
        return new OnFailureListener() {
            @Override
            public void onFailure(@NonNull Exception e) {
                // Task failed with an exception
                predictionQueueLength--;
                Log.e(TAG, e.toString());
            }
        };
    }

    /**
     * Gets a listener with code to be run when inference succeeds
     * @param bitmap the bitmap that predictions with this listener were made on
     * @return a new success listener
     */
    @NonNull
    private OnSuccessListener<List<DetectedObject>> getOnSuccessListener(final Bitmap bitmap) {
        return new OnSuccessListener<List<DetectedObject>>() {
            // the time when this listener was created. used to track frame order
            private final long startTime = System.nanoTime();
            // the bitmap that predictions with this listener are made on
            private final Bitmap predictionBitmap = bitmap;

            @Override
            public void onSuccess(@NonNull List<DetectedObject> detectedObjects) {
                // only count as successful if this frame's prediction was started after the latest frame
                predictionQueueLength--;
                long duration = (System.nanoTime() - startTime) / nanoToMilli;
                Log.d(TAG, "Inference completed in " + duration + " ms");

                if (startTime > latestResultStartTime) {
//                    Log.d(TAG, "New detections: " + detectedObjects.size());
                    latestDetections = detectedObjects; // update latest detections
                    newDetectionsAvailable = true;
                    latestResultStartTime = startTime; // update latest start

                    latestResultBitmap = predictionBitmap; // update latest bitmap

                    if (useDisplay) {
                        updateDisplay(predictionBitmap, detectedObjects);
                    }

                } else {
                    Log.d(TAG, "Received an out of order frame");
                }

            }
        };
    }

    private void updateDisplay(Bitmap predictedImage, List<DetectedObject> detectionList) {
        if (detectionList.size() > 0)
            Visuals.drawMLPredictions(predictedImage, convertToDetection(detectionList));
        updateImageView(predictedImage);
    }

}
