package org.firstinspires.ftc.teamcode.user;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.List;

// Class to interact with the Limelight Vision System for detecting targets and controlling robot movements.
public class Limelight {

    // ---------------------------------------------------------------------------
    // PID Constants for Control
    // ---------------------------------------------------------------------------
    // These constants are used to tune the robot's response to visual data, controlling how the robot corrects itself.
    public static double Kpang = -0.5; // P-gain for angle correction (pan) for fine adjustments.
    public static double KprotBase = -0.32; // Base proportional gain for rotational correction.
    public static double Kp = 0.195; // Proportional constant for y-axis (vertical movement) control.
    public static double Kpanginstant = -0.5; // P-gain for immediate angle adjustments (instant).
    public static double KprotBaseinstant = -0.06; // Base rotational gain for immediate corrections.
    public static double Kpinstant = 0.1; // Instant proportional gain for fine y-axis adjustments.

    // ---------------------------------------------------------------------------
    // Distance Scaling Factors
    // ---------------------------------------------------------------------------
    // These factors adjust the control mechanism based on the distance of the target.
    public static double distanceFactorRotBase = 1.9; // Adjusts rotation control based on distance.
    public static double distanceFactorAngBase = 1; // Adjusts angle control based on distance.
    public static double distanceFactorExtBase = 0.065; // Adjusts extension control based on distance.

    // ---------------------------------------------------------------------------
    // Error Variables
    // ---------------------------------------------------------------------------
    // Variables to track errors in the robot's positioning relative to the target.
    private double objectwidth = 0.0; // Width of the detected object to aid in adjusting movement.
    private double xError = 0.0; // Horizontal error in degrees from the target's x-position.
    private double yError = 0.0; // Vertical error in degrees from the target's y-position.

    // ---------------------------------------------------------------------------
    // Thresholds for Error Detection
    // ---------------------------------------------------------------------------
    // still need calibration
    public static double xErrorThreshold = 10002; // Threshold for x-error. The robot stops adjusting if this is exceeded.
    public static double yErrorThreshold = 10002; // Threshold for y-error. The robot stops adjusting if this is exceeded.

    // Limelight hardware interface object to interact with the camera.
    Limelight3A limelight;

    // Constructor to initialize the Limelight camera and set the pipeline.
    public Limelight(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight"); // Get the Limelight camera object from hardwareMap.
        limelight.pipelineSwitch(1); // Switch to the appropriate processing pipeline.
        limelight.start(); // Start the Limelight camera feed.
    }

    // ---------------------------------------------------------------------------
    // Angle Detection:
    // This method adjusts a servo based on the angle of the detected target in the x-direction.
    // ---------------------------------------------------------------------------
    public void angle_detect(Servo angle) {
        LLResult result = limelight.getLatestResult(); // Get the latest vision result from Limelight.

        // Check if the result is valid and contains detection data.
        if (result != null && result.isValid() && !result.getDetectorResults().isEmpty()) {
            List<LLResultTypes.DetectorResult> detectorResults = result.getDetectorResults();

            // Iterate through the detector results, though we expect just one target.
            for (LLResultTypes.DetectorResult fr : detectorResults) {
                xError = fr.getTargetXDegrees(); // Get the horizontal angle error (x-axis).

                // Extract the target corners for width calculation.
                double corner1 = fr.getTargetCorners().get(0).get(0);
                double corner2 = fr.getTargetCorners().get(3).get(0);
                double corner3 = fr.getTargetCorners().get(2).get(0);
                double corner4 = fr.getTargetCorners().get(1).get(0);

                // Calculate the width of the detected object by comparing the x-coordinates of its corners.
                double leftmostX = Math.min(Math.min(corner1, corner2), Math.min(corner3, corner4));
                double rightmostX = Math.max(Math.max(corner1, corner2), Math.max(corner3, corner4));

                // Subtract 100 to adjust the object width for calibration purposes.
                objectwidth = rightmostX - leftmostX - 100;
            }

            // Determine the servo position based on the width of the detected target.
            double maxWidth = 325 - 100; // Max width for normalization.
            double normalizedSignalAng;

            // If the xError exceeds a threshold, adjust the angle set point differently based on object width.
            if (Math.abs(xError) > 8.5) {
                if (objectwidth < 120) normalizedSignalAng = 0.52; // If object is small, use a larger angle adjustment.
                else normalizedSignalAng = 0.25; // Use a smaller adjustment if object is large.
            } else {
                if (objectwidth < 190) normalizedSignalAng = 0.52; // Smaller object -> larger angle adjustment.
                else normalizedSignalAng = 0.25; // Larger object -> smaller adjustment.
            }

            // Set the servo position to align with the target.
            angle.setPosition(normalizedSignalAng);
        }
    }

    // ---------------------------------------------------------------------------
    // Rotation Detection:
    // This method adjusts the servo based on the rotation (yaw) of the detected target.
    // ---------------------------------------------------------------------------
    public void rot_detect(Servo rot) {
        LLResult result = limelight.getLatestResult(); // Get the latest vision result.

        // Check if the result is valid and contains detection data.
        if (result != null && result.isValid() && !result.getDetectorResults().isEmpty()) {
            List<LLResultTypes.DetectorResult> detectorResults = result.getDetectorResults();

            // Iterate through the detected targets (one expected).
            for (LLResultTypes.DetectorResult fr : detectorResults) {
                xError = fr.getTargetXDegrees(); // Get horizontal error (angle).
            }

            // Calculate the distance factor for rotational correction based on the xError.
            double distanceFactorRot = distanceFactorRotBase * Math.abs(xError + 0.01);

            // If both errors are within thresholds, use a default rotation factor.
            if (Math.abs(xError) < xErrorThreshold && Math.abs(yError) < yErrorThreshold) {
                distanceFactorRot = 1;
            }

            double normalizedSignalRot = 0;
            double Kprot = KprotBase * distanceFactorRot;

            // If xError is larger than a threshold, calculate the necessary rotation to correct the position.
            if (Math.abs(xError) > 4) {
                double xErrorMax = 24; // Max allowable xError for rotation.
                double normalizedErrorRot = Math.max(-1.0, Math.min(1.0, (Kprot * xError) / xErrorMax)); // Normalize error.
                double targetPositionRot = rot.getPosition() + (normalizedErrorRot * 0.42); // Calculate the target position.
                normalizedSignalRot = Math.min(0.73, Math.max(0.15, targetPositionRot)); // Clamp the servo position.
            } else {
                normalizedSignalRot = 0.42; // Default position when the xError is small.
            }

            // Set the servo to correct the robot's rotation.
            rot.setPosition(normalizedSignalRot);
        }
    }

    // ---------------------------------------------------------------------------
    // Extension Detection:
    // This method adjusts the servo based on the detected target's vertical (y-axis) position.
    // ---------------------------------------------------------------------------
    public void extend_detect(Servo extend, Servo extendz) {
        LLResult result = limelight.getLatestResult(); // Get the latest result from Limelight.

        // Check if the result is valid and contains detection data.
        if (result != null && result.isValid() && !result.getDetectorResults().isEmpty()) {
            List<LLResultTypes.DetectorResult> detectorResults = result.getDetectorResults();

            // Iterate through the detection results.
            for (LLResultTypes.DetectorResult fr : detectorResults) {
                yError = -(fr.getTargetYDegrees() - 8); // Get vertical error (y-axis).
                xError = fr.getTargetXDegrees(); // Get horizontal error (x-axis).
            }

            // Calculate the distance factor for extension based on the xError.
            double distanceFactorExt = distanceFactorExtBase * Math.abs(xError + 0.01);

            // If the xError is small or the base distance factor is 1, reset the distance factor to 1.
            if (distanceFactorExtBase == 1 || Math.abs(xError) < 15) {
                distanceFactorExt = 1;
            }

            double KpScaled = Kp * distanceFactorExt;

            // Normalize the vertical error (yError) and calculate the target position for extension.
            double yErrorMax = 26;
            double normalizedErrorExt = Math.max(-1, Math.min(1, (KpScaled * yError) / yErrorMax));
            double targetPositionExt = extend.getPosition() + (normalizedErrorExt * 0.8); // Apply correction.
            double normalizedSignalExt = Math.min(1, Math.max(0.69, targetPositionExt)); // Clamp the servo position.

            // Set the servo positions for extension.
            extend.setPosition(normalizedSignalExt);
            extendz.setPosition(normalizedSignalExt);
        }
    }

    // ---------------------------------------------------------------------------
    // Object Detection Check:
    // This method checks whether a target is detected by the Limelight system.
    // ---------------------------------------------------------------------------
    public boolean is_detecting() {
        LLResult result = limelight.getLatestResult(); // Get the latest vision result.

        // If the result is valid and contains detection data, return true.
        if (result != null && result.isValid() && !result.getDetectorResults().isEmpty()) {
            return true;
        }
        // Otherwise, return false indicating no detection.
        else return false;
    }
}
