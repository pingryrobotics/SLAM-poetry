package tf_detection;

import android.graphics.Rect;
import android.graphics.RectF;

/**
 * The immutable detection class generalizes all detections from various object detectors
 */
public class Detection {

    private final RectF rectF;
    private final String label;
    private final float confidence;

    /**
     * Create a new immutable detection with the given parameters
     * @param rectF the bounding box of the detection, as a float rectangle
     * @param label the label of the detection. if multiple, use the one with the highest confidence
     * @param confidence the confidence of the detection
     */
    public Detection(RectF rectF, String label, float confidence) {
        this.rectF = rectF;
        this.label = label;
        this.confidence = confidence;
    }

    /**
     * Create a new immutable detection with the given parameters
     * @param rect the bounding box of the detection, as a rectangle
     * @param label the label of the detection. if multiple, use the one with the highest confidence
     * @param confidence the confidence of the detection
     */
    public Detection(Rect rect, String label, float confidence) {
        this.rectF = new RectF(rect);
        this.label = label;
        this.confidence = confidence;
    }

    // accessors because detection is immutable
    public float getTop() {return rectF.top; }
    public float getBottom() {return rectF.bottom; }
    public float getLeft() {return rectF.left; }
    public float getRight() {return rectF.right; }
    public float getCenterX() {return rectF.centerX(); }
    public float getCenterY() {return rectF.centerY(); }
    public String getLabel() {return label; }
    public RectF getRectF() { return new RectF(rectF); } // gets a copy
}
