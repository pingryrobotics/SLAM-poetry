package frame_source;

import android.graphics.Bitmap;
import android.graphics.ImageFormat;
import android.util.Log;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCharacteristics;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Objects;

/**
 * Enum of cameras available for use. Also used as singletons to keep track of each camera's state,
 * queue, and to add items from each camera into the queue
 * @note The camera names have to be manually set per the configuration and match up with the
 * hardware map's camera names
 */
public enum CameraInstance {
    FRONT("FrontCamera"),
    BACK("BackCamera"),
    LEFT("LeftCamera"),
    RIGHT("RightCamera");

    private static final String TAG = "vuf.test.CameraInstance";
    // the amount of time to wait while reopening a camera before giving up
    private static final List<CameraInstance> cachedList = Arrays.asList(CameraInstance.values());
    private static final int imageFormat = ImageFormat.YUY2;
    private static final Object switchLock = new Object();
    private final String cameraNameString;
    private boolean isAvailable; // if the camera is available
    @Nullable
    private FrameManager frameManager;
    private WebcamName webcamName;
    @NonNull
    private final List<FrameManager.FrameSupplier> supplierList;
    private boolean isVfSource;

    // region init


    CameraInstance(String cameraNameString) {
        this.cameraNameString = cameraNameString;
        supplierList = new ArrayList<>();
        isAvailable = false;
        isVfSource = false;
    }

    /**
     * Marks the camera as available and sets its name to the provided webcam name
     * Also initializes a bitmap queue for images to be placed into
     * @param webcamName the webcam name on the hardware map that this camera instance is mapped to
     */
    void setWebcamAsAvailable(@NonNull WebcamName webcamName, @NonNull FrameManager frameManager) {
        // make sure fm is valid when a camera is available
        this.frameManager = Objects.requireNonNull(frameManager,
                        "The frame manager is null or this camera instance is not initialized");
        this.webcamName = webcamName;
        isAvailable = true;
    }

    // endregion init

    // region suppliers

    /**
     * Add a new frame supplier to the camera's list
     * If the camera is closed, itll be opened for the new camera
     * @param newSupplier the new frame supplier to add
     */
    void addSupplier(FrameManager.FrameSupplier newSupplier) {
        // make sure this is only being called when frame manager is valid
        Objects.requireNonNull(frameManager,
                "The frame manager is null or this camera instance is not available");

        synchronized (switchLock) {
            Log.d(TAG, "Adding to instance " + this.name());
            if (!supplierList.contains(newSupplier)) {
                supplierList.add(newSupplier);
                Log.d(TAG, this.name() + " adding supplier");
            } else {
                Log.e(TAG, "Supplier already in list. \nSomething might be wrong");
            }

            if (!isVfSource && !frameManager.hasCameraAttached(this)) {
                Log.d(TAG, this.name() + " requesting camera");
                // try and get a new camera on a different thread because it takes a sec

                frameManager.getFmThreadPool().execute(new Runnable() {
                    @Override
                    public void run() {
                        boolean gotCamera = frameManager.requestCamera(CameraInstance.this);
                        Log.d(TAG, "Successfully got camera? " + gotCamera);
                    }
                });

            }

            Log.d(TAG, "Supplier added. Has camera? " + frameManager.hasCameraAttached(this));
            Log.d(TAG, "Number of suppliers after addition? " + supplierList.size());
        }
    }

    /**
     * Removes a supplier from the list of suppliers
     * If there are no suppliers remaining, the camera is closed
     * @param removedSupplier the supplier to remove
     */
    void removeSupplier(FrameManager.FrameSupplier removedSupplier) {
        synchronized (switchLock) {
            Log.d(TAG, "Removing from instance " + this.name());
            Log.d(TAG, "Number of suppliers before removal? " + supplierList.size());
            Log.d(TAG, "Attempting to remove supplier with id " + removedSupplier.getId());
            boolean removed = supplierList.remove(removedSupplier);
            Log.d(TAG, "Supplier removed? " + removed);
        }
    }

    /**
     * Alert all suppliers that depend on this instance that there's a new frame available
     * @param bitmap the new bitmap frame
     */
    void alertSuppliers(Bitmap bitmap) {
        for (FrameManager.FrameSupplier supplier : supplierList) {
            supplier.onNewFrame(bitmap);
        }
    }

    // endregion suppliers

    // region statics

    /**
     * Creates a list of all currently available cameras and returns them
     * @return a list of all cameras marked as available, aka currently attached to the robot
     */
    static List<CameraInstance> getAvailableCameras() {
        List<CameraInstance> availableCameras = new ArrayList<>();
        for (CameraInstance cameraInstance : values()) {
            if (cameraInstance.isAvailable()) availableCameras.add(cameraInstance);
        }
        return availableCameras;
    }

    /**
     * Gets the {@link CameraName} of all available cameras
     * @return a list of all available cameras
     */
    public static List<CameraName> getAvailableCameraNames() {
        List<CameraName> cameraNames = new ArrayList<>();
        for (CameraInstance cameraInstance : values()) {
            if (cameraInstance.isAvailable()) cameraNames.add(cameraInstance.getWebcamName());
        }
        return cameraNames;
    }



    /**
     * Check if the webcam supports the format we need
     * @param webcamName the webcam name to check
     * @return true if its supported, otherwise false
     */
    static boolean checkFormats(WebcamName webcamName) {
        CameraCharacteristics cameraCharacteristics = webcamName.getCameraCharacteristics();
        for (int format : cameraCharacteristics.getAndroidFormats()) {
            if (format == imageFormat) {
                return true;
            }
        }
        return false;
    }
    // we loop through camera instances a lot so might as well just save a list of them
    // since .values() creates a new array every time
    // woo premature optimization
    public static List<CameraInstance> cachedValues() { return cachedList; }



    // endregion statics

    // region accessors/modifiers
    /**
     * Determine if a camera is available, as in connected to the control hub
     * @return true if the camera is available, otherwise false
     */
    public boolean isAvailable() { return isAvailable; }

    String getCameraNameString() { return cameraNameString; }

    public WebcamName getWebcamName() { return webcamName; }

    /**
     * Try to remove the camera from this instance if its available for taking
     * @return true if the camera was successfully taken, otherwise false
     */
    boolean takeUnusedCamera() {
        return (isAvailable // must be available
                && frameManager != null // fm cant be null
                && frameManager.hasCameraAttached(this) // has to have a camera
                && !isVfSource // has to not be the vf source
                && supplierList.size() == 0); // must not have suppliers
        // although it shouldnt have suppliers anyway if it has no frame source but
        // it could be about to request one so we check anyway
    }

    // endregion accessors/modifiers
}