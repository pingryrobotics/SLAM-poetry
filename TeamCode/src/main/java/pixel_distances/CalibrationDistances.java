package pixel_distances;

import static java.lang.Math.atan;
import static java.lang.Math.tan;
import static java.lang.Math.toRadians;

import androidx.annotation.NonNull;

import org.jetbrains.annotations.TestOnly;

/**
 * Class for estimating distances based on object pixel position in photos
 * Distances are estimated based on an optimized function
 * this is for testing only, dont use it its not good
 */
@TestOnly
public class CalibrationDistances implements PixelDistances {
    private CameraType cameraType;

    public CalibrationDistances(CameraType cameraType) {
        this.cameraType = cameraType;
    }

    /**
     * Enum for managing camera calibrations for specific cameras and angles
     * each camera should be named CAMERA_TYPE_MIN_DISTANCE
     * the minimum distance is where the nearest visible distance is on the camera
     *
     * IMPORTANT: ALL DISTANCES ARE CALIBRATED ON 1280x720 RESOLUTIONS
     */
    public enum CameraType {
        // c615 with nearest visible distance at 20 inches
        // hAOV is horizontal angle of view
        LOGITECH_C615_MIN_20INCH(
                FunctionType.RECIPROCAL,
                298.45,
                66.1,
                19352828.2, -1.66043073),

        LOGITECH_C615_MIN_41INCH(FunctionType.RECIPROCAL, 298.45, 66.1, -1,
                -3.47820177);

        private static final double imageHeight = 720;
        private static final double imageWidth = 1280;
        private final long logitechA = 4852035810000L;

        private final FunctionType functionType;
        private final double hPPA; // horizontal pixels per angle
        private final double[] parameters;

        CameraType(FunctionType functionType, double cameraHeightMM, double hAOV, double... parameters) {
            // if there's not enough parameters, throw an error
            // this would be thrown really early in the program if it was wrong
            if (parameters.length != functionType.numParameters) {
                throw new IllegalArgumentException(this.name() + " has not been provided with the " +
                        "correct amount of function parameters for function type " +
                        functionType.name() + ". \n" + "Supplied: " + parameters.length +
                        " of required: " + functionType.numParameters);
            }
            this.functionType = functionType;
            this.hPPA = imageWidth/hAOV;
            this.parameters = parameters;
        }

        public double[] getParameters() {
            return parameters;
        }
    }

    /**
     * Enum for which functions are used to determine distances
     * Different functions take in different parameters which are determined by the cam type
     */
    private enum FunctionType {
        RECIPROCAL(2); // dist = A * (px ^ B)

        private final int numParameters;

        FunctionType(int numParameters) {
            this.numParameters = numParameters;
        }
    }

    /**
     * Calculate the distance to a vertical pixel based on its position on the camera
     * @param straightPx the vertical pixel
     * @param sidePx the horizontal pixel
     * @return the distances to the pixel based on the calibration, in mm and {straight, side} form
     */
    @NonNull
    @Override
    public double[] getDistancesToPixels(float straightPx, float sidePx) {
        if (cameraType.functionType == FunctionType.RECIPROCAL) {
            double[] functionParameters = cameraType.getParameters();
            double paramA = functionParameters[0];
            if (paramA == -1) {
                paramA = cameraType.logitechA;
            }
            // apply the function to get straight dist
            double straightDist = paramA * Math.pow(straightPx, functionParameters[1]);
            // calculate side dist using straight dist
            double sideDist = calculateSideDistance(straightDist, sidePx);
            return new double[] {straightDist, sideDist};
        }
        throw new IllegalArgumentException("Camera type " + cameraType.name() + " has no " +
                "valid functions.");
    }

    /**
     * Calculate the vertical pixel position based on a given distance
     * Essentially, if an object were to be x distance away, where would it be on screen
     * @param straightDist the straight distance to the object
     * @param sideDist the side distance to the object
     * @return the pixel positions on screen, in format {straightPx, sidePx}
     */
    @NonNull
    @Override
    public float[] getPixelsFromDistances(double straightDist, double sideDist) {
        if (cameraType.functionType == FunctionType.RECIPROCAL) {
            double[] functionParameters = cameraType.getParameters();
            float straightPx = (float) Math.pow((functionParameters[0]/straightDist), (1/ (-1 * functionParameters[1])));
            float sidePx = calculateSidePixel(straightDist, sideDist);
            return new float[] {straightPx, sidePx};
        }
        throw new IllegalArgumentException("Camera type " + cameraType.name() + " has no " +
                "valid functions.");
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
        return getDistancesToPixels(720, 640);
    }

    /**
     * Calculate the side distance to an object using trig
     * Since we know the straight distance and we can find out the angle to the object using
     * the side pixel and pixels per degree/angle value, we can construct a triangle and find out
     * the side distance ot the object
     * @param straightDist the straight distance to the object
     * @param sidePx the side pixel to the object
     * @return the side distance to the object
     */
    private float calculateSideDistance(double straightDist, double sidePx) {
        // pixel offset from center.
        // subtracting by side -> left, so left needs to be positive
        double sidePxOffset = CameraType.imageWidth/2 - sidePx;
        // use pixels to object and pixels per degree to get degrees to object
        double angleToObj = sidePxOffset/cameraType.hPPA;
        return (float)(straightDist * tan(toRadians(angleToObj)));
    }


    /**
     * Gets the pixel x value (horizontal) on an image from a distance
     * We use atan, or inverse tan, to get the angle from the ratio of the opposite and adjacent sides,
     * then multiply by pixels per angle to get the number of pixels from that angle.
     * Then, we subtract from half the image width since that value was the offset from the center.
     *
     * @param straightDist the straight value. used in calculations because we need opposite/adjacent side
     * @param sideDist the side value to get the distance for
     * @return the side pixel value
     */
    private float calculateSidePixel(double straightDist, double sideDist) {
        double angleToObject = Math.toDegrees(atan(sideDist/straightDist));
        double pixelsToObject = cameraType.hPPA * angleToObject;
        // calculate px
        return (float)((CameraType.imageWidth/2) - pixelsToObject);
    }

}


