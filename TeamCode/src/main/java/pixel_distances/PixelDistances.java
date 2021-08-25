package pixel_distances;

import androidx.annotation.NonNull;

import annotations.DistanceValues;
import annotations.FieldCoordinates;
import annotations.ImageCoordinates;

/**
 * Interface for various methods of finding distances to objects based on their pixel location
 * on screen
 */
public interface PixelDistances {

    /**
     * Find the distance to a set of pixels on screen
     * @param verticalPx the vertical pixel to find the distance to
     * @param horizontalPx the horizontal pixel to find the distance to
     * @return the distances to the pixels, in form {straight dist, side dist}
     */
    @NonNull
    @FieldCoordinates
    @DistanceValues
    double[] getDistancesToPixels(@ImageCoordinates float verticalPx, @ImageCoordinates float horizontalPx);

    /**
     * Find the pixels on screen where an object the given distances away would be located.
     * @param straightDist the straight distance to find the pixel for
     * @param sideDist the horizontal distance to find the pixel for
     * @return the pixels where an object a certain distance away would be,
     *         in form {vertical px, horizontal px}. The returned image coordinates
     *         may be offscreen.
     */
    @NonNull
    @ImageCoordinates
    float[] getPixelsFromDistances(
            @FieldCoordinates @DistanceValues double straightDist,
            @FieldCoordinates @DistanceValues double sideDist);


    /**
     * Find the set up distances for the camera. Essentially, the camera should be set up
     * so that the bottom center pixel of the screen is the returned value away from the camera.
     * If this method returns {2 feet, 0 feet}, then the lowest visible point on the camera should be 2 feet
     * away from the camera and located in the horizontal center of the screen.
     * This function uses millimeters.
     * @return the distances for setting up the camera, in form {straight dist, side dist}
     */
    @NonNull
    @FieldCoordinates
    @DistanceValues
    double[] getSetupDistances();
}
