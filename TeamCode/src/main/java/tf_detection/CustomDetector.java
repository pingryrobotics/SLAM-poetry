package tf_detection;

import android.graphics.Bitmap;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import java.util.List;

/**
 * An interface for custom object detectors to implement
 * Facilitates testing and use of multiple
 */
public interface CustomDetector {

    /**
     * Initialize the object detector
     */
    void initializeDetector();

    /**
     * Get the most updated predictions, if the result is different than the last call to
     * getUpdatedPredictions(). Otherwise, returns null
     * @return the predictions if they're different, or null
     */
    @Nullable
    List<Detection> getUpdatedDetections();

    /**
     * Gets the latest predictions from the object detector
     * @return a list of the latest predictions
     */
    @NonNull
    List<Detection> getLatestDetections();

    /**
     * Predict on the given bitmap
     * @param bitmap the bitmap to predict with
     */
    void predict(@NonNull Bitmap bitmap);

    /**
     * Gets the last bitmap that was successfully predicted on
     * @return the most recent bitmap
     */
    @Nullable
    Bitmap getLatestBitmap();

    /**
     * Check if the detector needs a new frame before supplying to avoid memory issues
     * @return true if the detector can handle a new frame, otherwise false
     */
    boolean needsNewFrame();

    /**
     * Returns the image height that the predictions are made on. This is important because
     * the detection rect's will also correspond to this size
     * @return the image height, in px
     */
    int getImageHeight();

    /**
     * Returns the image width that the predictions are made on. This is important because
     * the detection rect's will also correspond to this size
     * @return the image width, in px
     */
    int getImageWidth();


}
