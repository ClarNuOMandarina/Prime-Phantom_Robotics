package org.firstinspires.ftc.teamcode.user;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

// Configuration settings for dashboard telemetry and opMode setup.
@Config
@TeleOp(name="camera calibration")
public class camera_calibration extends LinearOpMode {

    // Constants for PID controller tuning for camera control
    public static double Kpang = -0.5; // Proportional gain for angular control (rotation)
    public static double KprotBase = -0.32; // Base proportional constant for rotation correction
    public static double Kp = 0.175; // Proportional constant for linear control (extension)
    public static double KprotBaseinstant = -0.06; // Instant correction constant for rotation
    public static double Kpinstant = 0.1; // Instant correction constant for extension

    // Tunable distance-based scaling factors for object detection
    public static double distanceFactorRotBase = 1.9; // Scaling factor for rotation control
    public static double distanceFactorAngBasex = 1; // Scaling factor for angular control
    public static double distanceFactorExtBase = 0.06; // Scaling factor for extension control

    private double objectwidth = 0.0; // Stores the detected object width
    private double xError = 0.0; // The error in the X-axis (horizontal) based on camera feedback
    private double yError = 0.0; // The error in the Y-axis (vertical) based on camera feedback

    // Thresholds for error values (x and y) for better control response
    public static double xErrorThreshold = 10002; // Threshold for X-axis error
    public static double yErrorThreshold = 10002; // Threshold for Y-axis error
    double coral = 0; // Placeholder variable for a potential operation or value (not used in the code)

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the Limelight camera and configure telemetry output
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        colection colection = new colection(hardwareMap); // Initialize the collection mechanism
        extension extension = new extension(hardwareMap); // Initialize the extension mechanism

        // Default positions for the collection system
        colection.gripper_height.setPosition(colection.height_default);
        colection.gripper_rotation.setPosition(colection.gripper_rotation_default);
        extension.extend(extension.extension_retracted); // Retract extension at the start

        colection.detectio_light(); // Activate detection light for the collection system
        telemetry.setMsTransmissionInterval(11); // Set telemetry transmission interval
        limelight.pipelineSwitch(1); // Switch to pipeline 1 for camera processing
        limelight.start(); // Start the camera processing

        boolean locked_on = false; // True if the camera has successfully locked onto the target object
        boolean corectionz = false; // A flag used to trigger a correction process

        // Wait for the start signal from the driver
        waitForStart();

        // Main loop that runs continuously during the opMode
        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult(); // Get the latest result from the Limelight camera

            // Check if the camera result is valid and contains detected objects
            if (result != null && result.isValid() && !result.getDetectorResults().isEmpty()) {
                List<LLResultTypes.DetectorResult> detectorResults = result.getDetectorResults();
                if (!locked_on) {
                    // If the target is not locked on yet, process the detected object
                    for (LLResultTypes.DetectorResult fr : detectorResults) {
                        // Log relevant information from the detection result (such as confidence, X and Y errors)
                        telemetry.addData("Detection", "Class: %s, Confidence: %.2f, X°: %.2f, Y°: %.2f",
                                fr.getClassName(), fr.getConfidence(), fr.getTargetXDegrees(), fr.getTargetYDegrees());

                        // Store the X and Y error values from the detection
                        xError = fr.getTargetXDegrees();
                        yError = -(fr.getTargetYDegrees() - 8); // Adjust Y error based on offset

                        // Calculate object width by analyzing the corners of the detected target
                        double corner1 = fr.getTargetCorners().get(0).get(0);
                        double corner2 = fr.getTargetCorners().get(3).get(0);
                        double corner3 = fr.getTargetCorners().get(2).get(0);
                        double corner4 = fr.getTargetCorners().get(1).get(0);

                        double leftmostX = Math.min(Math.min(corner1, corner2), Math.min(corner3, corner4));
                        double rightmostX = Math.max(Math.max(corner1, corner2), Math.max(corner3, corner4));

                        // Store the calculated object width and display it on telemetry
                        objectwidth = rightmostX - leftmostX - 100;
                        telemetry.addData("Object Width", objectwidth);
                    }
                }

                // **Dynamic distance-based scaling**:
                // Adjust the scaling factors based on the detected xError and yError to improve control
                double distanceFactorRot = distanceFactorRotBase * Math.abs(xError + 0.01);
                double distanceFactorExt = distanceFactorExtBase * Math.abs(xError + 0.01);
                double objectwidthx = objectwidth;

                // If the xError is large, scale the object width to match the detected angle
                if (Math.abs(xError) > 2)
                    objectwidthx *= (distanceFactorAngBasex * Math.abs(xError + 0.01));
                telemetry.addData("Object Widthx", objectwidthx);

                // If yError is small, disable the scaling for xError correction
                if (Math.abs(xError) < 15) {
                    distanceFactorExt = 1; // Disable scaling for extension if the xError is small
                    telemetry.addData("X Error Scaling", "Disabled due to small Y Error");
                }

                // **Apply Distance Factors**:
                // Adjust the scaling factors based on the distance (error) in X and Y directions
                double Kprot = KprotBase * distanceFactorRot; // Scaled rotational correction
                double KpangScaled = Kpang; // Scaled proportional gain for angular control
                double KpScaled = Kp * distanceFactorExt; // Scaled proportional gain for extension control

                // **Rotation Correction (X-axis)**:
                // Calculate and apply a correction for the robot’s rotation based on X error
                double xErrorMax = 24;
                double normalizedErrorRot = Math.max(-1.0, Math.min(1.0, (Kprot * xError) / xErrorMax));
                double targetPositionRot = colection.gripper_rotation.getPosition() + (normalizedErrorRot * 0.42);
                double normalizedSignalRot = Math.min(0.73, Math.max(0.15, targetPositionRot));

                // **Rotation Correction (X-axis) Instant**:
                // Instant correction for X-axis rotation, adjusting the gripper position
                double normalizedErrorRotinst = Math.max(-1.0, Math.min(1.0, (KprotBaseinstant * xError) / xErrorMax));
                double targetPositionRotinst = colection.gripper_rotation.getPosition() + (normalizedErrorRotinst * 0.42);
                double normalizedSignalRotinst = Math.min(0.73, Math.max(0.15, targetPositionRotinst));

                // **Extension Correction (Y-axis)**:
                // Apply a correction to the extension mechanism based on the Y error (vertical control)
                double yErrorMax = 26;
                double normalizedErrorExt = Math.max(-1, Math.min(1, (KpScaled * yError) / yErrorMax));
                double targetPositionExt = extension.left_extension.getPosition() + (normalizedErrorExt * 0.8);
                double normalizedSignalExt = Math.min(1, Math.max(0.69, targetPositionExt));

                // **Angle Correction (Using Object Width)**:
                // Use the calculated object width to adjust the gripper angle (orientation)
                double maxWidth = 325 - 100;
                double normalizedErrorAng = Math.max(-1, Math.min(1.0, KpangScaled * objectwidth / maxWidth));
                double targetPositionAng = 0.52 - (normalizedErrorAng * 1.1);
                double normalizedSignalAng = Math.min(0.52, Math.max(0.25, targetPositionAng));

                // **Debugging Info**:
                // Display various debugging values for the user to monitor during operation
                telemetry.addData("x Error", xError);
                telemetry.addData("y Error", yError);
                telemetry.addData("Object Width", objectwidth);
                telemetry.addData("Normalized Rotation Error", normalizedErrorRot);
                telemetry.addData("Normalized Extension Error", normalizedErrorExt);
                telemetry.addData("Normalized Angle Error", normalizedErrorAng);
                telemetry.addData("Target Rotation", targetPositionRot);
                telemetry.addData("Target Extension", targetPositionExt);
                telemetry.addData("Target Angle", targetPositionAng);
                telemetry.update();

                // **Action on Corection**:
                // If correction is needed (corectionz flag), execute a sequence of actions on collection
                if (corectionz) {
                    colection.gripper_angle.setPosition(normalizedSignalAng);
                    sleep(200);
                    colection.gripper_height.setPosition(colection.height_collecting);
                    sleep(300);
                    colection.gripper_grab();
                    sleep(400);
                    colection.gripper_height.setPosition(colection.height_default);
                    sleep(5000);
                    corectionz = false;
                }

                // **Manual Correction**:
                // If gamepad inputs are detected, apply the necessary corrections based on the gripper settings
                if (gamepad2.right_trigger != 0) {
                    colection.gripper_angle.setPosition(normalizedSignalAng);
                    colection.gripper_rotation.setPosition(normalizedSignalRot);
                    extension.extend(normalizedSignalExt);
                    colection.gripper_release();
                    colection.gripper_height.setPosition(colection.height_scanning);
                    sleep(500);

                    corectionz = true; // Flag for initiating the correction sequence
                }

                // **Instant Extension Adjustments**:
                // Apply immediate extension correction using instant gain values if left trigger is pressed
                if (gamepad2.left_trigger != 0) {
                    colection.gripper_height.setPosition(colection.height_scanning);
                    colection.gripper_angle.setPosition(normalizedSignalAng);
                    colection.gripper_rotation.setPosition(normalizedSignalRotinst);
                    extension.extend(normalizedSignalExt);
                    sleep(100);
                }

                // **Resetting Collection**:
                // Reset all collection mechanisms when the down button is pressed
                if (gamepad1.dpad_down) {
                    colection.gripper_rotation.setPosition(colection.gripper_rotation_default);
                    colection.gripper_angle.setPosition(0.385);
                    extension.extend(extension.extension_retracted);
                    locked_on = false;
                    colection.gripper_release();
                }

                // **Grab Action**:
                // Use the square button to grab a sample using the gripper
                if (gamepad1.square) {
                    colection.gripper_grab();
                }

                telemetry.update(); // Update telemetry for debugging and monitoring during runtime
            }
        }
    }
}
