package pixel_distances;

import static java.lang.Math.abs;
import static java.lang.Math.atan;
import static java.lang.Math.tan;
import static java.lang.Math.toRadians;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;

/**
 * Class for finding distances to objects on screen through angle of view calculations
 */
public class aovDistances implements PixelDistances {

    private static final int toComplement = 90;
    private final CameraCalibration camCal;
    private final double cameraHeightMM;
    private final double vPPD; // vertical pixels per degree
    private final double hPPD; // horizontal pixels per degree


    /**
     * Initialize the aov distance detector
     * @param cameraHeightMM the camera height, in mm
     * @param camCal the camera calibration to use
     * @param vAOV the vertical angle of view of the camera in use
     * @param hAOV the horizontal angle of view of the camera in use
     */
    public aovDistances(double cameraHeightMM, @NonNull CameraCalibration camCal, double vAOV, double hAOV) {
        this.camCal = camCal;
        this.cameraHeightMM = cameraHeightMM;
        this.hPPD = camCal.getSize().getWidth()/hAOV;
        this.vPPD = camCal.getSize().getHeight()/vAOV;
    }


    /**
     * Find the distance to a set of pixels on screen
     *
     * @param verticalPx   the vertical pixel to find the distance to
     * @param horizontalPx the horizontal pixel to find the distance to
     * @return the distances to the pixels, in form {straight dist, side dist}
     */
    @NonNull
    @Override
    public double[] getDistancesToPixels(float verticalPx, float horizontalPx) {
        double straightDist = getStraightDistance(verticalPx);
        double sideDist = getSideDistance(horizontalPx, straightDist);
        return new double[] {straightDist, sideDist};
    }

    /**
     * Get straight distance to a detection based on the angle of depression to the object and the
     * camera height.
     * This is the distance we would need to move forward to be horizontally in line
     * with the object.
     * @param verticalPixel the pixel to find the straight distance to
     * @return the distance to the object, in mm (or the unit camera height was specified in)
     */
    private double getStraightDistance(float verticalPixel) {
        double pxToObj = abs((float)camCal.getSize().getHeight()/2 - verticalPixel); // pixel offset from center of image
        double angleOfDepression = pxToObj/vPPD; // Use the pixels per degree to get the angle of dep.
        // units: px/1 * 1/D/px (or px/D) = angle
        // calculate complement of angle
        double angleToObject = toComplement - angleOfDepression;
        return cameraHeightMM * tan(toRadians(angleToObject));
    }


    /**
     * Get the side distance to the detection based on the straight distance and the
     * horizontal angle to the object. The side distance is the distance we would need to move
     * horizontally to have the object right in front of us.
     *
     * @param horizontalPx the horizontal px distance to the detection
     * @param straightDist the straight distance to the object
     * @return the calculated side distance. Left is negative, right is positive
     */
    private double getSideDistance(float horizontalPx, double straightDist) {
        // pixel offset from center.
        // subtracting by side -> left, so left needs to be positive
        double pxToObj = ((float)camCal.getSize().getWidth()/2) - horizontalPx;
        double angleToObj = pxToObj/hPPD;
        // distance to object
        return straightDist * tan(toRadians(angleToObj));
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
    @NonNull
    @Override
    public float[] getPixelsFromDistances(double straightDist, double sideDist) {
        float verticalPx = getVerticalPixel(straightDist);
        float horizontalPx = getHorizontalPixel(straightDist, sideDist);
        return new float[] {verticalPx, horizontalPx};
    }

    /**
     * Gets the pixel y value (vertical) on an image from a distance
     *
     * This calculation is just the inverse of the pixel to distance calculation.
     * We use atan, or inverse tan, to get the angle from the ratio of the opposite and adjacent sides,
     * then subtract from 90 to get the complement of that, then multiply by pixels per angle
     * to get the number of pixels from that angle. Then, we add half the image height
     * since that value was the offset from the center.
     *
     * @param straightDist the straight distance to get the pixel value for
     * @return the pixel value
     */
    private float getVerticalPixel(double straightDist) {
        double angleToObject = Math.toDegrees(atan(straightDist/cameraHeightMM));
        double angleOfDepression = toComplement - angleToObject;
        double pixelsToObject = vPPD * angleOfDepression;
        double verticalPixel = pixelsToObject + ((float)camCal.getSize().getHeight()/2);
        return (float) verticalPixel;
    }

    /**
     * Gets the pixel x value (horizontal) on an image from a distance
     * see getStraightPixelCoordinate comment for cameraInfo on this calculation. This one is
     * slightly different because the side ratio is different and we don't need the complement.
     *
     * @param straightDist the straight value. used in calculations because we need opposite/adjacent side
     * @param sideDist the side value to get the distance for
     * @return the pixel value
     */
    private float getHorizontalPixel(double straightDist, double sideDist) {
        double angleToObject = Math.toDegrees(atan(sideDist/straightDist));
        double pixelsToObject = hPPD * angleToObject;
        double horizontalPixel = ((float)camCal.getSize().getWidth()/2) - pixelsToObject;
        return (float) horizontalPixel;
    }

    /**
     * Find the set up distances for the camera. Essentially, the camera should be set up
     * so that the bottom center pixel of the screen is the returned value away from the camera.
     * If this method returns {2 feet, 0 feet}, then the lowest visible point on the camera should be 2 feet
     * away from the camera and located in the horizontal center of the screen.
     * This function uses millimeters.
     *
     * @return the distances for setting up the camera, in form {straight dist, side dist}
     */
    @NonNull
    @Override
    public double[] getSetupDistances() {
        return getDistancesToPixels(camCal.getSize().getHeight(), (float)camCal.getSize().getWidth()/2);
    }
}
