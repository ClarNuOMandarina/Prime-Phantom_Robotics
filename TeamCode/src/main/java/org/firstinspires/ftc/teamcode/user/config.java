package org.firstinspires.ftc.teamcode.user;

// Import statements for dashboard telemetry, event loop, hardware, and utilities
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

// TeleOp used for setting the values for mechanism positions, can mannualy move each mechanism to desired position
@TeleOp(name="config")
@Config  // Allows the use of the config interface for live variable adjustments during teleop
public class config extends LinearOpMode {

    // Public static variables for controlling the positions of different mechanisms
    public static int slidez = 0;  // Slide position (intended to control vertical slide position)
    public static double extendz = 0.5;  // Extension position for the extension mechanism
    public static double scoring_arm = 0.5;  // Scoring arm position
    public static double colecting_arms = 0.5;  // Collection arm position
    public static double gripz = 0.5;  // Gripper position (grabbing mechanism)
    public static double transfer_gripz = 0.5;  // Transfer gripper position
    public static double gripz_rotation = 0.5;  // Gripper rotation position
    public static double gripz_angle = 0.5;  // Gripper angle position
    public static double extend_armzz = 0.5;  // Extension arm position (used for scoring)
    public static double heightzzzz = 0.5;  // Height for the collection mechanism
    public static double hangz = 0.5;  // Hanging mechanism position
    public static double lightz = 0.5;  // Light mechanism position

    @Override
    public void runOpMode() throws InterruptedException {
        // Telemetry object to send data to both driver station and dashboard
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize subsystems for robot mechanisms
        slides slides = new slides(hardwareMap);  // Slides (for vertical movement)
        colection colection = new colection(hardwareMap);  // Collection system
        scoring scoring = new scoring(hardwareMap);  // Scoring mechanism
        extension extension = new extension(hardwareMap);  // Extension mechanism (e.g., arm extension)
        hanging hanging = new hanging(hardwareMap);  // Hanging system (used for endgame hanging actions)

        // Wait for the start signal from the driver
        waitForStart();

        // Main teleop loop that runs while the opMode is active
        while (opModeIsActive()) {

            // Control the slide mechanism with D-Pad Up button
            if (gamepad1.dpad_up) {
                slides.culisante(slidez);  // Move slides to the configured position 'slidez'
            }

            // Control the extension mechanism with D-Pad Left button
            if (gamepad1.dpad_left) {
                extension.extend(extendz);  // Move extension to 'extendz' position
            }

            // Control the scoring arm position with D-Pad Right button
            if (gamepad1.dpad_right) {
                scoring.score(scoring_arm);  // Move scoring arm to 'scoring_arm' position
            }

            // Control the extension arm for scoring with D-Pad Down button
            if (gamepad1.dpad_down) {
                scoring.scoring_arm_extension.setPosition(extend_armzz);  // Set scoring arm extension position
            }

            // Control gripper for collection mechanism with Gamepad2 D-Pad Right button
            if (gamepad2.dpad_right) {
                colection.gripper.setPosition(gripz);  // Move gripper to 'gripz' position for grabbing
            }

            // Control transfer gripper position with Gamepad2 D-Pad Left button
            if (gamepad2.dpad_left) {
                scoring.grip_transfer.setPosition(transfer_gripz);  // Move transfer gripper to 'transfer_gripz' position
            }

            // Control gripper rotation with Gamepad2 D-Pad Up button
            if (gamepad2.dpad_up) {
                colection.gripper_rotation.setPosition(gripz_rotation);  // Set gripper rotation to 'gripz_rotation'
            }

            // Control gripper angle with Gamepad2 D-Pad Down button
            if (gamepad2.dpad_down) {
                colection.gripper_angle.setPosition(gripz_angle);  // Set gripper angle to 'gripz_angle'
            }

            // Activate scoring configuration when right bumper is pressed on Gamepad1
            if (gamepad1.right_bumper) {
                colection.scoring_config();  // Activate the scoring configuration for collection mechanisms
            }

            // Control hanging mechanism with Gamepad1 Touchpad button
            if (gamepad1.touchpad) {
                hanging.hang(hangz);  // Trigger the hanging mechanism with the 'hangz' position
            }

            // Adjust gripper height with Gamepad1 Triangle button
            if (gamepad1.triangle) {
                colection.gripper_height.setPosition(heightzzzz);  // Set gripper height to 'heightzzzz'
            }

            // Activate light mechanism with Gamepad1 Share button
            if (gamepad1.share) {
                colection.light.setPosition(lightz);  // Set light mechanism to 'lightz' position
            }

            // Telemetry data showing real-time robot values
            telemetry.addData("CULI", slides.left_slide.getCurrentPosition());  // Show position of the left slide
            telemetry.addData("CULI2", slides.right_slide.getCurrentPosition());  // Show position of the right slide
            telemetry.addData("extend", extension.left_extension.getPosition());  // Show current position of the left extension
            telemetry.addData("sensor", colection.senzor.getDistance(DistanceUnit.CM));  // Show sensor distance
            telemetry.addData("sensor", colection.senzor.getConnectionInfo());  // Show sensor connection info

            // Update the telemetry to display the data
            telemetry.update();
        }
    }
}
