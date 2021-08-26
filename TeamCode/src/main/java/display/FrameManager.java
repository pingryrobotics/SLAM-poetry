package display;

import android.graphics.Bitmap;
import android.graphics.ImageFormat;
import android.os.Handler;
import android.util.Log;

import androidx.annotation.NonNull;

import com.google.common.collect.BiMap;
import com.google.common.collect.HashBiMap;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ThreadPool;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.android.util.Size;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureRequest;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureSequenceId;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureSession;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCharacteristics;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraException;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraFrame;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraManager;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.SwitchableCamera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.delegating.SwitchableCameraImpl;
import org.firstinspires.ftc.robotcore.internal.collections.EvictingBlockingQueue;
import org.firstinspires.ftc.robotcore.internal.system.ContinuationSynchronizer;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.List;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.ThreadPoolExecutor;
import java.util.concurrent.TimeUnit;

public class FrameManager {

    private static final String TAG = "vuf.test.framemanager";
    private HardwareMap hardwareMap;
    private CameraManager cameraManager;
    private static final int queueCapacity = 1;
    private Handler callbackHandler;
    private final int imageFormat = ImageFormat.YUY2;
    private BiMap<CameraInstance, SwitchableCameraImpl> camBiMap;
    private final Object cameraLock = new Object();
    private CameraInstance currentVFCamera;
    private final ExecutorService fmThreadPool =
            ThreadPool.newFixedThreadPool(4, "teamcode.frame_manager");
    private int availableMonitorId = 0;
    private int supplierIdCount = 0;

    /**
     * Initialize the frame manager
     * @param hardwareMap the hardware map to use
     */
    public FrameManager(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        this.cameraManager = ClassFactory.getInstance().getCameraManager();
        camBiMap = HashBiMap.create();
        initializeCameras();
    }

    /**
     * Initialize all cameras, if possible
     * Each {@link CameraInstance} should have a camera name string preset to it
     * We assume that the hardware map has these camera names set to the corresponding camera,
     * and so we loop through all our camera instances and search the hardware map for a camera
     * with a matching name. If we get one, we set the camera's {@link WebcamName} to the matching
     * webcam name, which marks the CameraInstance as available for use.
     */
    private void initializeCameras() {
        List<WebcamName> webcamNameList = cameraManager.getAllWebcams();
        for (CameraInstance cameraInstance : CameraInstance.cachedValues()) {
            // get the WebcamName from the hardware map
            try {
                WebcamName webcamName = hardwareMap.get(WebcamName.class, cameraInstance.getCameraNameString());
                if (webcamNameList.contains(webcamName) && CameraInstance.checkFormats(webcamName)) {
                    cameraInstance.setWebcamAsAvailable(webcamName, this);
                } else {
                    Log.d(TAG, "Camera instance " + cameraInstance.name() + " was not available");
                }
            } catch (IllegalArgumentException e) {
                Log.d(TAG, "Camera instance " + cameraInstance.name() + " was not found");
            }
        }
    }

    /**
     * Get a frame supplier for the specified camera instance
     * @param cameraInstance the initial camera to get a supplier for
     * @return the frame supplier
     */
    public FrameSupplier getFrameSupplier(CameraInstance cameraInstance, int displayMonitorId) {
        if (cameraInstance.isAvailable()) {
            FrameSupplier frameSupplier = new FrameSupplier(cameraInstance, supplierIdCount++);
            cameraInstance.addSupplier(frameSupplier);
            availableMonitorId = displayMonitorId;
            return frameSupplier;
        }
        return null;
    }

    @NonNull
    ExecutorService getFmThreadPool() { return fmThreadPool; }

    /**
     * Checks the active threads in the frame manager threadpool
     * @return th enumber of active threads, or -1 if it could not be determines
     */
    int checkActiveThreads() {
        if (fmThreadPool instanceof ThreadPoolExecutor) {
            return ((ThreadPoolExecutor) fmThreadPool).getActiveCount();
        }
        return -1;
    }

    /**
     * Switches the supplier's camera source to the next item in its queue
     * @param supplier the supplier to change
     * @return true if it was successfully switched, otherwise false
     */
    synchronized private boolean switchSupplierSource(@NonNull FrameSupplier supplier) {
        CameraInstance newCamera = supplier.switchRequestQueue.poll();
        // only process if the new camera isnt null, is available, and isnt the same camera
        if (newCamera != null && newCamera != supplier.getCameraInstance() && newCamera.isAvailable()) {
            supplier.getCameraInstance().removeSupplier(supplier);
            newCamera.addSupplier(supplier);
            return true;
        }
        return false;
    }

    /**
     * Determine if a camera instance has a camera attached
     * @param cameraInstance the instance to check
     * @return true if there's a camera mapped to it in the hashmap, otherwise false
     */
    boolean hasCameraAttached(CameraInstance cameraInstance) {
        return (camBiMap.get(cameraInstance) != null);
    }


    // region camera requests

    /**
     * Allows a camera instance to request a switchable camera
     * @param callingInstance the calling camera instance
     * @return true if the camera was successfully provided, otherwise false
     */
    synchronized boolean requestCamera(CameraInstance callingInstance) {
        // if the instance is the vfc, it cant have a camera
        if (callingInstance == currentVFCamera) {
            return false;
        }
        // try and find an unused, open camera
        for (CameraInstance cameraInstance : CameraInstance.cachedValues()) {
            if (cameraInstance.takeUnusedCamera()) {
                Log.d(TAG, "Found unused camera " + cameraInstance.name());
                // if we find one, switch it
                SwitchableCameraImpl switchableCamera = camBiMap.remove(cameraInstance);
                if (switchableCamera != null) {
                    Log.d(TAG, "Switching active camera");
                    switchableCamera.setActiveCamera(callingInstance.getWebcamName());
                    camBiMap.put(callingInstance, switchableCamera);
                    return true;
                }
            }
        }
        Log.d(TAG, "No unused camera found");
        // if we dont find one, we need to make a new one
        // no monitor sadge
        SwitchableCameraImpl newSC = getSwitchableCamera();
        Log.d(TAG, "sc impl null? " + (newSC == null));
        availableMonitorId = 0;
        try {
            if (newSC != null) {
                camBiMap.put(callingInstance, newSC);
                initializeSwitchableCamera(newSC);
                return true;
            }
        } catch (CameraException e) {
            camBiMap.remove(callingInstance);
            e.printStackTrace();
        }
        return false;
    }


    // endregion camera requests


    // region switchable camera management

    /**
     * Gets and initializes a switchable camera
     * @return the switchable camera
     */
    public SwitchableCameraImpl getSwitchableCamera() {
//        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(displayMonitorId);
        CameraManager cameraManager = ClassFactory.getInstance().getCameraManager();
        CameraName[] cameraNames = CameraInstance.getAvailableCameraNames()
                .toArray(new CameraName[0]);
        Log.d(TAG, "num of camera names: " + cameraNames.length);
        CameraName scName = cameraManager
                .nameForSwitchableCamera(cameraNames);


//        VuforiaWebcam vuforiaWebcam = new VuforiaWebcam(parameters.webcamCalibrationResources,
//                parameters.webcamCalibrationFiles, parameters.minWebcamAspectRatio,
//                parameters.maxWebcamAspectRatio, parameters.secondsUsbPermissionTimeout,
//                parameters.cameraName);
        Camera camera = cameraManager.requestPermissionAndOpenCamera(new Deadline(5, TimeUnit.SECONDS), scName, null);
        Log.d(TAG, "vuf camera null? " + (camera == null));

        return (SwitchableCameraImpl) ((SwitchableCamera) camera);
    }

    /**
     * Initializes a switchable camera by starting a capture session
     * @param switchableCamera the switchable camera to initialize
     * @throws CameraException if initialization fails
     */
    private void initializeSwitchableCamera(@NonNull SwitchableCameraImpl switchableCamera) throws CameraException {
        synchronized (cameraLock) {
            final ContinuationSynchronizer<CameraCaptureSession> synchronizer = new ContinuationSynchronizer<>();
            CameraCharacteristics cameraCharacteristics = switchableCamera.getCameraName().getCameraCharacteristics();
            switchableCamera.createCaptureSession(
                    Continuation.create(
                            callbackHandler,
                            getCaptureSessionCallback(
                                    cameraCharacteristics, switchableCamera, synchronizer)));
            try {
                synchronizer.await();
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }

            CameraCaptureSession cameraCaptureSession = synchronizer.getValue();
        }
    }

    /**
     * Gets a capture session callback for managing what happens after a capture session is created
     * @param cameraCharacteristics the characteristics of the camera
     * @param switchableCamera the camera to get a callback for
     * @param synchronizer the synchronizer to keep track of the state of the call chain
     * @return the callback
     */
    private CameraCaptureSession.StateCallback getCaptureSessionCallback(
            final CameraCharacteristics cameraCharacteristics,
            final SwitchableCameraImpl switchableCamera,
            final ContinuationSynchronizer<CameraCaptureSession> synchronizer) {
        return new CameraCaptureSession.StateCallbackDefault() {
            @Override
            public void onConfigured(@NonNull CameraCaptureSession session) {
                Size size = cameraCharacteristics.getDefaultSize(imageFormat);
                int fps = cameraCharacteristics.getMaxFramesPerSecond(imageFormat, size);
                try {
                    CameraCaptureRequest captureRequest = switchableCamera
                            .createCaptureRequest(imageFormat, size, fps);
                    beginCapturing(session, captureRequest, switchableCamera);
                } catch (CameraException e) {
                    e.printStackTrace();
                    synchronizer.finish(null);
                }
                synchronizer.finish(session);
            }
        };
    }

    /**
     * Begins capturing frames for the specified camera
     * @param session the capture session to capture through
     * @param captureRequest the capture request, specifying the fps and resolution
     * @param switchableCameraImpl the camera to capture with
     * @throws CameraException if starting the capture session fails
     */
    private void beginCapturing(
            @NonNull CameraCaptureSession session,
            @NonNull final CameraCaptureRequest captureRequest,
            @NonNull final SwitchableCameraImpl switchableCameraImpl)
            throws CameraException {
        // start the capture session with the provided callback
        session.startCapture(captureRequest, new CameraCaptureSession.CaptureCallback() {
            @Override
            public void onNewFrame(
                    @NonNull CameraCaptureSession session,
                    @NonNull CameraCaptureRequest request,
                    @NonNull CameraFrame cameraFrame) {
                // when there's a new frame, convert it to a bitmap and alert suppliers of the new bitmap
                Bitmap bitmap = captureRequest.createEmptyBitmap();
                cameraFrame.copyToBitmap(bitmap);
                // alert the suppliers of the camera instance with this camera
                CameraInstance cameraInstance = camBiMap.inverse().get(switchableCameraImpl);
                if (cameraInstance != null) {
//                    Log.d(TAG, "Alerting suppliers of " + cameraInstance.name());
                    cameraInstance.alertSuppliers(bitmap);
                } else {
                    Log.d(TAG, "no camera attached");
                }
            }
        }, Continuation.create(callbackHandler, new CameraCaptureSession.StatusCallback() {
            @Override
            public void onCaptureSequenceCompleted(
                    @NonNull CameraCaptureSession session,
                    CameraCaptureSequenceId cameraCaptureSequenceId,
                    long lastFrameNumber) {
                // when the capture session is over, we just log for now
                Log.d(TAG, "Camera capture session ended");
            }
        }));
    }

    // endregion switchable camera management



    /**
     * The frame supplier class is used by a consumer of frames to obtain a constant supply
     * of frames when necessary and switch cameras, if possible
     */
    public class FrameSupplier {

        /**
         * Create a queue into which we can place images from the open camera
         * When the queue is full, the oldest item is evicted and disposed of
         * Users can take images from the queue as they become available
         */
        private final EvictingBlockingQueue<Bitmap> bitmapBlockingQueue;
        /**
         * To avoid issues arising from multiple switch request being inputted at once,
         * we create a queue into which we place switch requests as they come.
         * Only one switch request can be processed by {@link FrameManager#switchSupplierSource(FrameSupplier)}
         * at a time. So if we submit a request, then while its being processed, we submit 5 more,
         * when FrameManager gets another request to process, only the last request which was submitted
         * will be processed.
         */
        private final EvictingBlockingQueue<CameraInstance> switchRequestQueue;
        private CameraInstance cameraInstance;
        private int id;

        private FrameSupplier(CameraInstance cameraInstance, int id) {
            this.cameraInstance = cameraInstance;
            this.id = id;
            // create the queue
            bitmapBlockingQueue = new EvictingBlockingQueue<>(new ArrayBlockingQueue<>(queueCapacity));
            // proactively recycle bitmaps that the user doesn't consume to help reduce memory pressure
            bitmapBlockingQueue.setEvictAction(new Consumer<Bitmap>()
            {
                @Override public void accept(Bitmap bitmap) { bitmap.recycle(); }
            });
            // only the most recent switch request will be honored
            switchRequestQueue = new EvictingBlockingQueue<>(new ArrayBlockingQueue<>(queueCapacity));
        }

        int getId() { return id; }

        /**
         * On new frames, we add the new one to the bitmap queue
         * @param bitmap the new bitmap to add
         */
        void onNewFrame(Bitmap bitmap) {
            bitmapBlockingQueue.add(bitmap);
        }

        public CameraInstance getCameraInstance() { return cameraInstance; }

        /**
         * Switch this supplier's frame source to a new camera
         * @param newCamera the new camera to switch to
         * @return true if the switch was successful, otherwise false
         */
        public boolean switchFrameSource(CameraInstance newCamera) {
            switchRequestQueue.add(newCamera);
            if (switchSupplierSource(this)) {
                this.cameraInstance = newCamera;
                return true;
            }
            return false;
        }

        /**
         * Takes a frame from the queue
         * If there's no frame, returns null
         * @return a bitmap, or null if there's no bitmap in the queue
         */
        public Bitmap takeFrame() { return bitmapBlockingQueue.poll(); }


        /**
         * Two suppliers are equal if they have the same id
         * @param o the other object to compare
         * @return true if the suppliers have the same id, otherwise false
         */
        @Override
        public boolean equals(Object o) {
            if (this == o) return true;
            if (o == null || getClass() != o.getClass()) return false;
            FrameSupplier supplier = (FrameSupplier) o;
            return id == supplier.id;
        }
    }









}
