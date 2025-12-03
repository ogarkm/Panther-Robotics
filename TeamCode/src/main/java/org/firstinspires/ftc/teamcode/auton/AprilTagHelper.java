package org.firstinspires.ftc.teamcode.auton; // Ensure this matches your project's root package

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
 * Utility class to initialize the VisionPortal and AprilTagProcessor
 * and retrieve the ID of a detected AprilTag.
 */
public class AprilTagHelper {
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    // Standard blue alliance tags for the backdrop
    private static final int TARGET_TAG_ID_1 = 1;
    private static final int TARGET_TAG_ID_2 = 2;
    private static final int TARGET_TAG_ID_3 = 3;

    /**
     * Initializes the AprilTag processor and VisionPortal.
     * @param hardwareMap The OpMode's hardware map.
     */
    public AprilTagHelper(HardwareMap hardwareMap) {

        // 1. Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()
                .build();

        // 2. Create the Vision Portal.
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")) // IMPORTANT: Match your actual camera name
                .addProcessor(aprilTag)
                .build();
    }

    /**
     * Checks for detected AprilTags and returns a position (1, 2, or 3)
     * corresponding to the detected tag ID, or 0 if no target tag is found.
     * * NOTE: This method should be called repeatedly during the init_loop.
     * * @return The ID of the detected tag (1, 2, or 3), or 0 if none is detected.
     */
    public int getDetectedTagId() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        // Loop through all detected tags
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                int id = detection.id;

                // Return the ID if it matches one of our targets
                if (id == TARGET_TAG_ID_1 || id == TARGET_TAG_ID_2 || id == TARGET_TAG_ID_3) {
                    return id;
                }
            }
        }

        return 0; // Return 0 or a safe default if no target tag is visible
    }

    /**
     * Closes the VisionPortal when the OpMode ends.
     */
    public void close() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}
