package pixel_distances;

import static java.lang.Math.atan;
import static java.lang.Math.tan;
import static java.lang.Math.toRadians;

import android.util.Log;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;

import java.util.Arrays;

import annotations.DistanceValues;
import annotations.FieldCoordinates;
import annotations.ImageCoordinates;

/**
 * Class for finding distances to objects based on their pixel location using the focal length
 * of the camera
 */
public class FocalDistances implements PixelDistances {
    private static final String TAG = "vuf.test.focal_dist";
    public static final int toComplement = 90;
    private final CameraCalibration camCal;
    private final double cameraHeightMM;

    /**
     * Initialize the focal distance detector.
     * @param cameraHeightMM the camera height from the ground, in millimeters
     * @param camCal the camera calibration of the camera in use
     */
    public FocalDistances(double cameraHeightMM, CameraCalibration camCal) {
        this.camCal = camCal;
        this.cameraHeightMM = cameraHeightMM;
    }

    /**
     * Calculate the distances to a set of pixels
     * @param straightPx the vertical pixel to find the straight distance to
     * @param sidePx the horizontal pixel to find the side distance to
     * @return the straight and side distances
     */
    @Override
    @NonNull
    @FieldCoordinates
    @DistanceValues
    public double[] getDistancesToPixels(float straightPx, float sidePx) {
        Log.d(TAG, String.format("Calibration cameraInfo: %s", camCal));
        Log.d(TAG, String.format("Distortion Coefficients: %s", Arrays.toString(camCal.distortionCoefficients)));

        double straightDist = getStraightDistance(straightPx);
        double sideDist = getSideDistance(sidePx, straightDist);
        return new double[] {straightDist, sideDist};
    }

    /**
     * Find the pixels on screen where an object the given distances away would be located.
     *
     * @param straightDist the straight distance to find the pixel for
     * @param sideDist     the horizontal distance to find the pixel for
     * @return the pixels where an object a certain distance away would be,
     * in form {vertical px, horizontal px}. The returned image coordinates
     * may be offscreen.
     */
    @Override
    @NonNull
    @ImageCoordinates
    public float[] getPixelsFromDistances(double straightDist, double sideDist) {
        float straightPx = getVerticalPixel(straightDist);
        float sidePx = getHorizontalPixel(straightDist, sideDist);
        return new float[] {straightPx, sidePx};
    }


    /**
     * Get distance to the nearest visible point on the camera for calibration
     * The camera should be adjusted so the bottom of the camera can see this point
     * For example, if the distances are (609.6, 0), then the bottom of the camera's frame should
     * be able to see 2 feet from the camera and should be centered
     * @return the distances for calibration
     */
    @Override
    @NonNull
    @FieldCoordinates
    @DistanceValues
    public double[] getSetupDistances() {
        float bottomPx = camCal.getSize().getHeight();
        float middlePx = ((float)camCal.getSize().getWidth())/2;
        return getDistancesToPixels(bottomPx, middlePx);
    }

    // region distances to pixels
    /**
     * Get straight distance to a detection based on the angle of depression to the object and the
     * camera height.
     * This is the distance we would need to move forward to be horizontally in line
     * with the object.
     *
     * @param straightPx the pixel to find the distance to
     * @return the straight distance to the object, in mm
     */
    @FieldCoordinates
    @DistanceValues
    private double getStraightDistance(float straightPx) {
        double angle = toComplement - getVerticalAngle(straightPx);
        Log.d(TAG, "vertical angle to object: " + angle);
        double ratio = tan(toRadians(angle));
        return cameraHeightMM * ratio;
    }

    /**
     * Get the side distance to the detection based on the straight distance and the
     * horizontal angle to the object. The side distance is the distance we would need to move
     * horizontally to have the object right in front of us.
     *
     * @param straightDist the straight distance to the detection
     * @return the calculated side distance. Left is negative, right is positive
     */
    @FieldCoordinates
    @DistanceValues
    private double getSideDistance(float sidePx, double straightDist) {
        double angle = getHorizontalAngle(sidePx);
        Log.d(TAG, "horizontal angle to object: " + angle);
        double ratio = tan(toRadians(angle));
        return straightDist * ratio;
    }

    /**
     * Get the horizontal angle to a pixel using the focal length
     * The focal length is the distance from the camera lens to the point where the light
     * converges. Using this length, which we have in pixels, we can construct a triangle using this
     * as the adjacent side and the distance from the center of the camera to the object (in
     * pixels) as our opposite side. Then, we can use inverse tan to get the angle to the object
     * from the camera lens.
     * @param sidePx The center x pixel of the object on screen
     * @return the angle to the object
     */
    private double getHorizontalAngle(float sidePx) {
        double adjacentSide = camCal.focalLengthX;
        double oppositeSide = ((double)camCal.getSize().getWidth() / 2) - sidePx;
        return AngleUnit.DEGREES.fromRadians(Math.atan(oppositeSide/adjacentSide));
    }

    /**
     * @see #getHorizontalAngle(float) for how this works, since its literally the same thing
     * @param straightPx the bottom y pixel of the object on screen. Basically the pixel where the
     *                   object hits the ground
     * @return the vertical angle to the object
     */
    private double getVerticalAngle(float straightPx) {
        double adjacentSide = camCal.focalLengthY;
        double oppositeSide = straightPx - ((double)camCal.getSize().getHeight() / 2);
        return AngleUnit.DEGREES.fromRadians(Math.atan(oppositeSide/adjacentSide));
    }

    // endregion distances to pixels

    // region pixels to distances


    /**
     * Get the vertical pixel from a straight distance
     * @param straightDist the distance to find the pixel y value for
     * @return the pixel y value for the given distance
     */
    private float getVerticalPixel(double straightDist) {
        double angleToObject = Math.toDegrees(atan(straightDist/cameraHeightMM));
        double angleOfDepression = toComplement - angleToObject;
        double pixelsToObject = tan(toRadians(angleOfDepression)) * camCal.focalLengthY;
        return (float) (((double)camCal.getSize().getHeight() / 2) + pixelsToObject);
    }

    /**
     * Get the horizontal pixel from a straight distance
     * @param straightDist the straight distance to find the pixel x value for
     * @param sideDist the distance to find the pixel x value for
     * @return the pixel x value for the given distance
     */
    private float getHorizontalPixel(double straightDist, double sideDist) {
        double angleToObject = Math.toDegrees(atan(sideDist/straightDist));
        double pixelsToObject = tan(toRadians(angleToObject)) * camCal.focalLengthX;
        return (float) (((double)camCal.getSize().getWidth() / 2) - pixelsToObject);
    }



    // endregion pixels to distances


}
