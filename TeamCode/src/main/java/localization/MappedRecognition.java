package localization;

import android.util.Log;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

import java.util.Arrays;

import annotations.FieldCoordinates;
import annotations.MatrixCoordinates;

/**
 * A class for managing recognitions that are mapped out on the fieldmap
 */
public class MappedRecognition {
    private static final String TAG = "vuf.test.mapped_rec";
    private static final int proximityRange = FieldMap.getScale();
    private OpenGLMatrix fieldPosition;
    private int[] matrixPosition;
    private SpaceMap.Space space;

    /**
     * Initialize a mapped recognition
     * @param fieldPosition the field position of the recognition
     * @param matrixPosition the matrix position of the recognition
     * @param space the space of the recognition
     */
    public MappedRecognition(@FieldCoordinates @NonNull OpenGLMatrix fieldPosition,
                             @MatrixCoordinates @NonNull int[] matrixPosition,
                             @NonNull SpaceMap.Space space) {
        this.fieldPosition = fieldPosition;
        this.matrixPosition = matrixPosition;
        this.space = space;
    }

    /**
     * Updates this recognition if there's a conflict between it and another recognition.
     * If the spaces are of the same type and are close to each other, then this recognition's
     * position is changed to the other one. However, if they're of different types but occupy
     * the same space, then this recognition's space is updated to be the other recognition's.
     * @param otherRecognition the other recognition to check
     * @return true if the recognition was updated, otherwise false
     */
    public boolean updateIfConflict(@NonNull MappedRecognition otherRecognition) {

        if (withinProximityRange(otherRecognition.fieldPosition)) {
            // if they have the same space, update
            if (otherRecognition.space == space) {
                copyRecognition(otherRecognition);
                return true;
            } else if (sameMatrixPosition(otherRecognition)) {
                // if they have different spaces but the same matrix pos, update
                copyRecognition(otherRecognition);
                return true;
            }
        }
        return false;
    }

    /**
     * Copies the position and space of another mapped recognition this recognition
     * @param otherRecognition the recognition to copy data from
     */
    private void copyRecognition(@NonNull MappedRecognition otherRecognition) {
        fieldPosition = otherRecognition.getFieldPosition();
        matrixPosition = otherRecognition.getMatrixPosition();
        space = otherRecognition.getSpace();
    }

    /**
     * Determine if another set of field coordinates is within the range of these coordinates
     * @param otherFieldPos the other field coordinates to test
     * @return true if within range, otherwise false
     */
    private boolean withinProximityRange(@FieldCoordinates @NonNull OpenGLMatrix otherFieldPos) {
        VectorF currentTranslation = fieldPosition.getTranslation();
        VectorF newTranslation = otherFieldPos.getTranslation();
        if (Math.abs(currentTranslation.get(0) - newTranslation.get(0)) <= proximityRange) {
            Log.d(TAG, "X is within range");
            if (Math.abs(currentTranslation.get(1) - newTranslation.get(1)) <= proximityRange) {
                Log.d(TAG, "Y is within range");
                return true;
            }
        }
        return false;
    }

    /**
     * Determine if two recognitions have the same matrix position
     * @param otherRecognition the other recognition to check
     * @return true if they do, otherwise false
     */
    public boolean sameMatrixPosition(@NonNull MappedRecognition otherRecognition) {
        return Arrays.equals(otherRecognition.matrixPosition, matrixPosition);
    }

    public SpaceMap.Space getSpace() { return space; }
    public int[] getMatrixPosition() { return matrixPosition; }
    public OpenGLMatrix getFieldPosition() { return fieldPosition; }




}
