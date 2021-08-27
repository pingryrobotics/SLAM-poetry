package display;

import android.graphics.ImageFormat;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCharacteristics;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

/**
 * Enum for different camera locations on the robot and their corresponding cameras
 * Sort of like {@link CameraInstance} except only for getting camera names, nothing about switching
 */
public enum CameraLocation {

    FRONT("FrontCamera"),
    BACK("BackCamera"),
    LEFT("LeftCamera"),
    RIGHT("RightCamera");

    private static final String TAG = "vuf.test.CameraInstance";
    private boolean isAvailable = false;
    private String cameraName;

    CameraLocation(String cameraNameString) {
        this.cameraName = cameraNameString;
    }

    public String getCameraName() { return cameraName; }

    /**
     * Check if the webcam supports the format we need
     * @param webcamName the webcam name to check
     * @return true if its supported, otherwise false
     */
    public static boolean checkFormat(WebcamName webcamName) {
        CameraCharacteristics cameraCharacteristics = webcamName.getCameraCharacteristics();
        for (int format : cameraCharacteristics.getAndroidFormats()) {
            if (format == ImageFormat.YUY2) {
                return true;
            }
        }
        return false;
    }


}
