package tf_detection;

import static android.view.View.VISIBLE;
import static android.view.ViewGroup.LayoutParams.MATCH_PARENT;

import android.app.Activity;
import android.graphics.Bitmap;
import android.util.Log;
import android.view.ViewGroup;
import android.widget.ImageView;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.util.List;

import pathfinding.Visuals;

abstract public class LibDetector {

    private final AppUtil appUtil = AppUtil.getInstance();
    private ImageView imageView;
    private final static String TAG = "vuf.test.libdetector";
    /**
     * Initializes an image view for displaying a live feed of frames to the robot
     * @param monitorViewIdParent the id of monitor view to use for displaying frames
     */
    protected void initImageView(int monitorViewIdParent) {
        final Activity activity = appUtil.getRootActivity();
        final ViewGroup imageViewParent = activity.findViewById(monitorViewIdParent);

        if (imageViewParent != null) {
            appUtil.synchronousRunOnUiThread(new Runnable() {
                @Override
                public void run() {
                    imageView = new ImageView(activity);
                    imageView.setScaleType(ImageView.ScaleType.FIT_CENTER);
                    imageView.setLayoutParams(new ViewGroup.LayoutParams(MATCH_PARENT, MATCH_PARENT));
                    imageViewParent.addView(imageView);
                    imageViewParent.setVisibility(VISIBLE);
                }
            });
        }
    }

    /**
     * Updates the image view with a new image
     * @param predictedImage the image to use
     * @param detectionList the detections to draw onto the image
     */
    protected void updateImageView(@NonNull final Bitmap predictedImage, @NonNull List<?> detectionList) {
        if (detectionList.size() > 0)
            Visuals.drawMLPredictions(predictedImage, convertToDetection(detectionList));
        appUtil.synchronousRunOnUiThread(
                new Runnable() {
                    @Override
                    public void run() {
                        imageView.setImageBitmap(predictedImage);
                    }
                });
    }

    /**
     * Converts from the ml library's prediction class into our Detection class
     * @param predictionList the list of native predictions to convert
     * @return the converted predictions, as a list of {@link Detection} objects
     */
    @NonNull
    protected abstract List<Detection> convertToDetection(@NonNull List<?> predictionList);

}
