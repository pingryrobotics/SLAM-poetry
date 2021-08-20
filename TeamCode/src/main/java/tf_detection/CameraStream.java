package tf_detection;

import android.graphics.Bitmap;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import java.util.ArrayList;
import java.util.List;

/**
 * Test class to test FrameManager's camera switching
 */
public class CameraStream extends DisplaySource implements CustomDetector {
    
    public CameraStream(int monitorViewId) {
        super(monitorViewId);
    }



    /**
     * Initialize the object detector
     */
    @Override
    public void initializeDetector() {
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
        return null;
    }

    /**
     * Gets the latest predictions from the object detector
     *
     * @return a list of the latest predictions
     */
    @NonNull
    @Override
    public List<Detection> getLatestDetections() {
        return new ArrayList<>();
    }

    /**
     * Predict on the given bitmap
     *
     * @param bitmap the bitmap to predict with
     */
    @Override
    public void predict(@NonNull Bitmap bitmap) {
        updateImageView(bitmap);
    }

    /**
     * Gets the last bitmap that was successfully predicted on
     *
     * @return the most recent bitmap
     */
    @Nullable
    @Override
    public Bitmap getLatestBitmap() {
        return null;
    }

    /**
     * Check if the detector needs a new frame before supplying to avoid memory issues
     *
     * @return true if the detector can handle a new frame, otherwise false
     */
    @Override
    public boolean needsNewFrame() {
        return true;
    }

    /**
     * Returns the image height that the predictions are made on. This is important because
     * the detection rect's will also correspond to this size
     *
     * @return the image height, in px
     */
    @Override
    public int getImageHeight() {
        return -1;
    }

    /**
     * Returns the image width that the predictions are made on. This is important because
     * the detection rect's will also correspond to this size
     *
     * @return the image width, in px
     */
    @Override
    public int getImageWidth() {
        return -1;
    }
}
