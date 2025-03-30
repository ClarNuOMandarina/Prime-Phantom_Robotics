package org.firstinspires.ftc.teamcode.user;

// ---------------------------------------------------------------------------
// Import Statements:
// ---------------------------------------------------------------------------
// These imports include Road Runner classes for trajectory building and actions,
// the Limelight library for vision processing, and FTC SDK classes for opmode control.
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.PinpointDrive;

// ---------------------------------------------------------------------------
// Class Declaration:
// ---------------------------------------------------------------------------
// This class is an Autonomous opmode ,in this opmode we colelct from the submersible autonomosuly with the Limelight sistems
// LinearOpMode so that the runOpMode() method executes sequentially.
@Autonomous(name="auto_basket experimental")
public class auto_basket_experimental extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // -------------------------------------------------------------------
        // Initialization of Starting Pose:
        // -------------------------------------------------------------------
        // Define the initial position and heading (in radians) for the robot.
        // In this case, the robot starts at coordinates (-40, -58) facing 90 degrees.
        Pose2d initialPose = new Pose2d(new Vector2d(-40, -58), Math.toRadians(90));

        // -------------------------------------------------------------------
        // Limelight Initialization:
        // -------------------------------------------------------------------
        // Create a Limelight object to use for vision-based detection.
        // The Limelight is used to automatically detect samples based on their vision coordinates.
        Limelight ll = new Limelight(hardwareMap);
        telemetry.setMsTransmissionInterval(11); // Set telemetry update frequency

        // -------------------------------------------------------------------
        // Subsystem Initialization:
        // -------------------------------------------------------------------
        // Initialize various subsystems of the robot including:
        // - The drive system (using PinpointDrive) for moving the robot.
        // - The collection system ("colection") for grabbing samples.
        // - The extension mechanism for moving arms or other attachments.
        // - The scoring system for placing samples.
        // - The slides system for moving vertical mechanisms.
        PinpointDrive drive = new PinpointDrive(hardwareMap, initialPose);
        colection colection = new colection(hardwareMap);
        extension extension = new extension(hardwareMap);
        scoring scoring = new scoring(hardwareMap);
        slides slides = new slides(hardwareMap);

        // -------------------------------------------------------------------
        // Hardware Configuration and Initialization:
        // -------------------------------------------------------------------
        // Set up the robot's hardware in its starting configuration.
        slides.slide_init(); // Initialize the slides to their starting position.
        colection.init_config(); // Set the initial configuration for the collection mechanism.
        colection.gripper_rotation.setPosition(colection.gripper_rotation_default); // Set the gripper rotation to its default value.
        extension.extend(extension.extension_retracted); // Ensure the extension mechanism is retracted.
        scoring.scoring_arm_default(); // Set the scoring arm to its default position.
        slides.reset_encoder(); // Reset the encoder for the slides.
        scoring.gripper_grab(); // Activate the gripper to grab (it may be preloaded or for calibration).

        // -------------------------------------------------------------------
        // Miscellaneous Variable Initialization:
        // -------------------------------------------------------------------
        ElapsedTime taim = new ElapsedTime(0); // timer to forcefully stop automated colection after a while, we can ignore exceptional cases by using it
        double samp4 = 0; // used for debugging, to save the detected coords of the sample before automated colection
        double x = -4;    // Used to save the last X position of the farthest detected sample so we can save time for further colection cycles
        colection.light.setPosition(colection.detection_light); // Turn on or adjust the light used for detection via Limelight.

        // -------------------------------------------------------------------
        // Trajectory Definitions:
        // -------------------------------------------------------------------
        // Here we build several trajectories using TrajectoryActionBuilder.
        // Each trajectory represents a series of timed actions and movements.

        // --- Trajectory: start_to_score ---
        // This trajectory moves the robot from its initial position to a scoring area.
        // It also triggers slide actions and scoring arm movements at specific times.
        TrajectoryActionBuilder start_to_score = drive.actionBuilder(initialPose)
                .afterTime(0, slides.slide_sample()) // At 0 seconds, begin sample slide action.
                .afterTime(0.4, slides.slide_sample()) // Reinforce slide movement at 0.4 seconds.
                .afterTime(0.6, scoring.sample_score_retracted()) // At 0.6 seconds, retract the scoring arm for sample scoring.
                .afterTime(0.8, colection.collecting_arm_default()) // At 0.8 seconds, set the collecting arm to its default position.
                .strafeToLinearHeading(new Vector2d(-59, -51), Math.toRadians(69)); // Strafe to the specified coordinates with a 69° heading.

        // --- Trajectory: sample_1 ---
        // This trajectory drives the robot from the scoring position to the first sample collection position.
        TrajectoryActionBuilder sample_1 = drive.actionBuilder(new Pose2d(new Vector2d(-59, -51), Math.toRadians(69)))
                .afterTime(0.4, slides.slide_init()) // moves the slides to the init position after 0.4 seconds.
                .afterTime(0.7, slides.slide_init()) // moves the slides to the init position after 0.7 seconds.
                .afterTime(0.1, scoring.sample_collect()) // At 0.1 seconds, trigger sample collection.
                .strafeToLinearHeading(new Vector2d(-45.5, -43), Math.toRadians(90)); // Strafe to a new position with a 90° heading.

        // --- Trajectory: sample_finish ---
        // This trajectory finalizes the sample collection by grabbing, releasing, and scoring the sample.
        TrajectoryActionBuilder sample_finish = drive.actionBuilder(new Pose2d(new Vector2d(-45.5, -43), Math.toRadians(90)))
                .afterTime(0, scoring.gripper_grab()) // Immediately grab the sample.
                .afterTime(0.2, colection.griper_release()) // Release the gripper after 0.2 seconds.
                .afterTime(0.3, slides.slide_sample()) // Trigger slide action at 0.3 seconds.
                .afterTime(0.8, scoring.sample_score_auto()) // At 0.8 seconds, automatically score the sample.
                .strafeToLinearHeading(new Vector2d(-62.5, -50), Math.toRadians(80)); // Strafe to a new position with an 80° heading.

        // --- Trajectory: sample_2 ---
        // This trajectory defines the movement for collecting the second sample.
        TrajectoryActionBuilder sample_2 = drive.actionBuilder(new Pose2d(new Vector2d(-62.5, -50), Math.toRadians(80)))
                .afterTime(0.4, slides.slide_init()) // moves the slides to the init position at 0.4 seconds.
                .afterTime(0.7, slides.slide_init()) //moves the slides to the init position at 0.7 seconds.
                .afterTime(0.5, colection.collecting_arm_default()) // Set collecting arm to default at 0.5 seconds.
                .strafeToLinearHeading(new Vector2d(-61.5, -45), Math.toRadians(83)); // Strafe to second sample position.

        // --- Trajectory: sample_finish_2 ---
        // This trajectory finishes the second sample collection by scoring it.
        TrajectoryActionBuilder sample_finish_2 = drive.actionBuilder(new Pose2d(new Vector2d(-61.5, -45), Math.toRadians(83)))
                .afterTime(0, scoring.gripper_grab()) // Immediately grab the sample.
                .afterTime(0.2, colection.griper_release()) // Release the gripper after 0.2 seconds.
                .afterTime(0.3, slides.slide_sample()) // Slide action at 0.3 seconds.
                .afterTime(0.7, scoring.sample_score_auto()) // Automatically score the sample at 0.7 seconds.
                .strafeToLinearHeading(new Vector2d(-64, -50), Math.toRadians(87)); // Strafe to new heading and position.

        // --- Trajectory: sample_3 ---
        // This trajectory drives to the position for the third sample collection.
        TrajectoryActionBuilder sample_3 = drive.actionBuilder(new Pose2d(new Vector2d(-64, -50), Math.toRadians(87)))
                .afterTime(0.4, slides.slide_init()) // moves the slides to the init position
                .afterTime(0.7, slides.slide_init()) // moves the slides to the init position.
                .afterTime(0.6, colection.third_sample()) // Execute third sample collection routine.
                .strafeToLinearHeading(new Vector2d(-63, -44), Math.toRadians(93)); // Strafe to the third sample position.

        // --- Trajectory: sample_finish_3 ---
        // This trajectory finalizes the third sample collection by scoring it.
        TrajectoryActionBuilder sample_finish_3 = drive.actionBuilder(new Pose2d(new Vector2d(-63, -44), Math.toRadians(93)))
                .afterTime(0, scoring.gripper_grab()) // Grab the sample immediately.
                .afterTime(0.2, colection.griper_release()) // Release gripper after 0.2 seconds.
                .afterTime(0.3, slides.slide_sample()) // Slide action at 0.3 seconds.
                .afterTime(0.8, scoring.sample_score_auto()) // Score the sample automatically after 0.8 seconds.
                .strafeToLinearHeading(new Vector2d(-64, -48), Math.toRadians(87)); // Strafe to final position.

        // --- Trajectory: sample_4 ---
        // This trajectory moves the robot from the third sample area to a final position using a spline path.
        TrajectoryActionBuilder sample_4 = drive.actionBuilder(new Pose2d(new Vector2d(-64, -48), Math.toRadians(87)))
                .afterTime(0.4, slides.slide_init()) // moves the slides to the init position at 0.4 seconds.
                .afterTime(0.7, slides.slide_init()) // moves the slides to the init position at 0.7 seconds.
                .afterTime(0.5, colection.collecting_arm_default()) // Set collecting arm to default at 0.5 seconds.
                .splineToLinearHeading(new Pose2d(-20, -4, 0), Math.toRadians(0)); // Spline to final position with 0° heading.

        // -------------------------------------------------------------------
        // Pre-Start Configurations:
        // -------------------------------------------------------------------
        // Set initial positions for the grip transfer and gripper rotation before starting autonomous.
        scoring.grip_transfer.setPosition(scoring.gripper_hold);
        colection.gripper_rotation.setPosition(colection.gripper_rotation_score_sample);

        // -------------------------------------------------------------------
        // Wait for Autonomous Start:
        // -------------------------------------------------------------------
        // This method call halts execution until the start button is pressed.
        waitForStart();

        // After start, reset gripper rotation back to its default configuration.
        colection.gripper_rotation.setPosition(colection.gripper_rotation_default);

        // Set the slides to a high basket position, likely to prepare for scoring.
        slides.culisante(slides.slides_high_basket);

        // If a stop is requested during initialization, exit immediately.
        if (isStopRequested()) return;

        // -------------------------------------------------------------------
        // Autonomous Action Execution:
        // -------------------------------------------------------------------
        // Run the first action sequence that drives the robot from its starting
        // position to the scoring area using the previously defined trajectory.
        Actions.runBlocking(
                new SequentialAction(
                        start_to_score.build() // Build and execute the start_to_score trajectory.
                ));


        // ---------------------------------------------------------------------------
// This block handles sample scoring, sample finishing and initial arm/gripper actions.
// It extends the arm, releases the gripper, and then retracts the arm before executing sample_1 trajectory.
// ---------------------------------------------------------------------------
        extension.extend(extension.extension_extended); // Fully extend the mechanism (likely to reach the sample)
        sleep(100); // Wait 100ms for extension to complete
        scoring.gripper(scoring.gripper_release); // Release the gripper to let go of the sample
        sleep(200); // Wait 200ms to ensure the gripper action has completed
        scoring.scoring_arm_extension.setPosition(scoring.extension_retracted); // Retract the scoring arm
        Actions.runBlocking( // Execute the following trajectory action sequentially
                new SequentialAction(
                        sample_1.build() // Build and run the first sample collection trajectory
                ));

// ---------------------------------------------------------------------------
// This block reconfigures the collection system after finishing sample_1.
// It releases the gripper, adjusts the gripper height for collection, and sets the gripper to transfer mode.
// It also adjusts the gripper rotation and angle, then extends the arm slightly for transfer.
// Finally, it executes the "sample_finish" trajectory.
// ---------------------------------------------------------------------------
        colection.gripper_release(); // Ensure the gripper is released
        sleep(100); // Wait 100ms for the release action to settle
        colection.gripper_height.setPosition(colection.height_collecting); // Adjust gripper height to collection position
        sleep(200); // Wait 200ms for the height adjustment
        colection.gripper.setPosition(colection.gripper_transfer); // Set the gripper to transfer configuration
        sleep(200); // Wait 200ms
        colection.gripper_height.setPosition(colection.height_transfer); // Adjust gripper height for transfer action
        colection.gripper_rotation.setPosition(colection.gripper_rotation_default); // Reset gripper rotation to default
        colection.gripper_angle.setPosition(colection.gripper_angle_default); // Reset gripper angle to default
        extension.extend(extension.extension_transfer + 0.08); // Extend the arm slightly more than the transfer length (offset adjustment)
        sleep(400); // Wait 400ms for the extension
        extension.extend(extension.extension_transfer); // Ensure proper transfer extension is achieved
        Actions.runBlocking( // Execute the next sequential action block
                new SequentialAction(
                        sample_finish.build() // Build and execute the trajectory to finish sample 1
                ));

// ---------------------------------------------------------------------------
// This block configures the scoring mechanism after sample_finish.
// It extends the scoring arm to a maximum extended position, then retracts and releases the sample.
// Finally, it resets the arm and collection configuration.
// ---------------------------------------------------------------------------
        scoring.scoring_arm_extension.setPosition(scoring.extension_extended_max); // Set scoring arm extension to max for scoring
        extension.extend(extension.extension_extended); // Fully extend the mechanism (likely to deliver or score the sample)
        sleep(500); // Wait 500ms for the mechanism to extend
        scoring.gripper(scoring.gripper_release); // Release the sample from the gripper
        sleep(200); // Wait 200ms
        scoring.scoring_arm_extension.setPosition(scoring.extension_retracted); // Retract the scoring arm after scoring
        scoring.scoring_arm_default(); // Reset the scoring arm to its default position
        colection.default_config(); // Reset the collection system configuration
        colection.gripper_angle.setPosition(colection.gripper_angle_default); // Reset gripper angle

// ---------------------------------------------------------------------------
// Execute trajectory for sample_2 collection.
// ---------------------------------------------------------------------------
        Actions.runBlocking(
                new SequentialAction(
                        sample_2.build() // Build and run the sample_2 trajectory
                ));
        sleep(100); // Wait for 100ms

// ---------------------------------------------------------------------------
// Use Limelight for rotational detection before processing sample_2.
// The limelight function "rot_detect" adjusts gripper rotation based on vision data.
        ll.rot_detect(colection.gripper_rotation);
        colection.gripper_release(); // Ensure gripper is released
        sleep(100); // Wait 100ms
        colection.gripper_height.setPosition(colection.height_collecting); // Adjust gripper height for collection
        sleep(200); // Wait 200ms
        colection.gripper.setPosition(colection.gripper_transfer); // Set gripper to transfer position
        sleep(200); // Wait 200ms
        colection.gripper_height.setPosition(colection.height_transfer); // Adjust height to transfer configuration
        colection.gripper_rotation.setPosition(colection.gripper_rotation_default); // Reset gripper rotation
        colection.gripper_angle.setPosition(colection.gripper_angle_default); // Reset gripper angle

// ---------------------------------------------------------------------------
// Fine-tune extension for sample transfer for sample_2.
// ---------------------------------------------------------------------------
        extension.extend(extension.extension_transfer + 0.08); // Extend slightly beyond transfer position
        sleep(500); // Wait 500ms
        extension.extend(extension.extension_transfer); // Ensure correct transfer extension
        Actions.runBlocking(
                new SequentialAction(
                        sample_finish_2.build() // Execute the trajectory to finish sample 2
                ));

// ---------------------------------------------------------------------------
// After finishing sample_2, adjust extension and scoring arm for next operations.
// ---------------------------------------------------------------------------
        extension.extend(extension.extension_extended); // Fully extend the mechanism for scoring
        scoring.scoring_arm_extension.setPosition(scoring.extension_extended_max); // Set scoring arm to its max extended position
        sleep(500); // Wait 500ms
        scoring.gripper(scoring.gripper_release); // Release the sample from the gripper
        sleep(200); // Wait 200ms
        scoring.scoring_arm_default(); // Reset scoring arm to default

// ---------------------------------------------------------------------------
// Execute trajectory for sample_3 collection.
// ---------------------------------------------------------------------------
        scoring.scoring_arm_extension.setPosition(scoring.extension_retracted); // Retract scoring arm for safe movement
        colection.default_config(); // Reset collection configuration
        Actions.runBlocking(
                new SequentialAction(
                        sample_3.build() // Build and execute the sample_3 trajectory
                ));
        sleep(100); // Wait 100ms

// ---------------------------------------------------------------------------
// Use Limelight again to adjust rotation for sample_3 collection.
// ---------------------------------------------------------------------------
        ll.rot_detect(colection.gripper_rotation); // Adjust rotation based on Limelight detection
        colection.gripper_release(); // Release gripper
        colection.gripper_angle.setPosition(0.72); // Set gripper angle (specific value for sample_3)
        sleep(100); // Wait 100ms
        colection.gripper_height.setPosition(colection.height_collecting); // Set gripper height to collecting position
        sleep(300); // Wait 300ms
        colection.gripper.setPosition(colection.gripper_transfer); // Set gripper to transfer configuration
        sleep(200); // Wait 200ms
        colection.gripper_height.setPosition(colection.height_transfer); // Adjust height for transfer
        colection.gripper_rotation.setPosition(colection.gripper_rotation_default); // Reset gripper rotation
        colection.gripper_angle.setPosition(colection.gripper_angle_default); // Reset gripper angle

// ---------------------------------------------------------------------------
// Fine-tune extension for sample_3 transfer and finish sample_3.
// ---------------------------------------------------------------------------
        extension.extend(extension.extension_transfer + 0.08); // Extend slightly beyond transfer position
        sleep(500); // Wait 500ms
        extension.extend(extension.extension_transfer); // Ensure correct extension
        Actions.runBlocking(
                new SequentialAction(
                        sample_finish_3.build() // Execute trajectory to finish sample 3
                ));

// ---------------------------------------------------------------------------
// Post-sample_3: Retract extension and set scoring arm for next operation.
// ---------------------------------------------------------------------------
        extension.extend(extension.extension_retracted); // Retract extension fully
        scoring.scoring_arm_extension.setPosition(scoring.extension_extended_max); // Extend scoring arm to max (preparing for scoring)
        sleep(500); // Wait 500ms
        scoring.gripper(scoring.gripper_release); // Release sample from gripper
        sleep(200); // Wait 200ms
        scoring.scoring_arm_default(); // Reset scoring arm to default

// ---------------------------------------------------------------------------
// Prepare for final sample (sample_4): Retract arm, reset collection config.
// ---------------------------------------------------------------------------
        scoring.scoring_arm_extension.setPosition(scoring.extension_retracted);
        colection.default_config();
        colection.gripper_angle.setPosition(colection.gripper_angle_default);
        drive.updatePoseEstimate(); // Update robot's current pose estimate

// ---------------------------------------------------------------------------
// Execute trajectory for sample_4 collection.
// ---------------------------------------------------------------------------
        Actions.runBlocking(
                new SequentialAction(
                        sample_4.build() // Execute the sample_4 trajectory (likely a spline to a new area)
                ));

// ---------------------------------------------------------------------------
// Loop for sample detection using Limelight for sample_4.
// The robot continuously scans until a sample is detected by vision.
// ---------------------------------------------------------------------------
        while (opModeIsActive()) {
            sleep(200); // Wait 200ms between checks
            if (ll.is_detecting()) { // If Limelight detects a target/sample...
                ll.rot_detect(colection.gripper_rotation); // Adjust gripper rotation based on vision data
                ll.extend_detect(extension.left_extension, extension.right_extension); // Use vision to determine extension distances
                colection.gripper_release(); // Release gripper in preparation for collection
                colection.gripper_height.setPosition(colection.height_scanning); // Set height for scanning
                taim.reset(); // Reset a timer to control the next phase
                while (taim.seconds() < 0.4 && opModeIsActive()) { // For 0.4 seconds, process vision angle detection
                    telemetry.addData("samp4", samp4);
                    telemetry.update();
                    ll.angle_detect(colection.gripper_angle); // Adjust gripper angle based on vision
                }
                colection.gripper_height.setPosition(colection.height_collecting); // Reset height for collection
                sleep(200); // Wait 200ms
                colection.gripper.setPosition(colection.gripper_transfer); // Set gripper to transfer mode
                sleep(300); // Wait 300ms
                colection.gripper_height.setPosition(colection.height_default); // Reset gripper height to default
                sleep(300); // Wait 300ms
                // If the distance sensor reads a sample within collection range, extend the mechanism slightly and break out of loop
                if (colection.senzor.getDistance(DistanceUnit.CM) < colection.distance_to_collected_sample) {
                    extension.extend(extension.extension_transfer + 0.08);
                    break;
                }
                sleep(300); // Wait 300ms
                // Reset all configurations if sample not detected
                colection.gripper_height.setPosition(colection.height_default);
                colection.gripper_rotation.setPosition(colection.gripper_rotation_default);
                colection.gripper.setPosition(colection.gripper_release);
                extension.extend(extension.extension_retracted);
            } else {
                // If Limelight is not detecting, update pose and move slightly to scan a new area.
                drive.updatePoseEstimate();
                TrajectoryActionBuilder sample_collect1 = drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(new Vector2d(-20, x), Math.toRadians(0));
                Actions.runBlocking(
                        new SequentialAction(
                                sample_collect1.build()
                        ));
                x += 2.5; // Increment the offset for the next scan position
            }
            // Ensure the mechanisms are reset between iterations
            extension.extend(extension.extension_retracted);
            colection.gripper_height.setPosition(colection.height_default);
            colection.gripper_rotation.setPosition(colection.gripper_rotation_default);
            colection.gripper.setPosition(colection.gripper_release);
            sleep(100); // Brief pause between scans
        }

// ---------------------------------------------------------------------------
// Final adjustments after detection loop for sample_4
// ---------------------------------------------------------------------------
        colection.gripper_height.setPosition(colection.height_transfer);
        colection.gripper_rotation.setPosition(colection.gripper_rotation_default);
        colection.gripper_angle.setPosition(colection.gripper_angle_default);
        extension.extend(extension.extension_transfer + 0.08);
        sleep(200);
        extension.extend(extension.extension_transfer);

// ---------------------------------------------------------------------------
// Execute final finishing trajectory for sample_4
// ---------------------------------------------------------------------------
        TrajectoryActionBuilder sample_finish_4 = drive.actionBuilder(drive.pose)
                .afterTime(0.2, scoring.gripper_grab())
                .afterTime(0.4, colection.griper_release())
                .afterTime(0.5, slides.slide_sample())
                .afterTime(0.7, scoring.sample_score_retracted())
                .strafeToLinearHeading(new Vector2d(-60, -47), Math.toRadians(70));
        Actions.runBlocking(
                new SequentialAction(
                        sample_finish_4.build()
                ));
        extension.extend(extension.extension_retracted);
        sleep(300);
        scoring.gripper(scoring.gripper_release);
        sleep(200);
        scoring.scoring_arm_default();

// ---------------------------------------------------------------------------
// Reset configurations and update pose before reattempting sample_4 trajectory.
// ---------------------------------------------------------------------------
        scoring.scoring_arm_extension.setPosition(scoring.extension_retracted);
        colection.default_config();
        colection.gripper_angle.setPosition(colection.gripper_angle_default);
        drive.updatePoseEstimate();

        Actions.runBlocking(
                new SequentialAction(
                        sample_4.build()
                ));

// ---------------------------------------------------------------------------
// Second loop for sample_4 detection using Limelight, similar to the previous loop.
// ---------------------------------------------------------------------------
        while (opModeIsActive()) {
            sleep(200);
            if (ll.is_detecting()) {
                ll.rot_detect(colection.gripper_rotation);
                ll.extend_detect(extension.left_extension, extension.right_extension);
                colection.gripper_release();
                colection.gripper_height.setPosition(colection.height_scanning);
                taim.reset();
                while (taim.seconds() < 0.4 && opModeIsActive()) {
                    telemetry.addData("samp4", samp4);
                    telemetry.update();
                    ll.angle_detect(colection.gripper_angle);
                }
                colection.gripper_height.setPosition(colection.height_collecting);
                sleep(200);
                colection.gripper.setPosition(colection.gripper_transfer);
                sleep(300);
                colection.gripper_height.setPosition(colection.height_default);
                sleep(300);
                if (colection.senzor.getDistance(DistanceUnit.CM) < colection.distance_to_collected_sample) {
                    extension.extend(extension.extension_transfer + 0.08);
                    break;
                }
                sleep(300);
                colection.gripper_height.setPosition(colection.height_default);
                colection.gripper_rotation.setPosition(colection.gripper_rotation_default);
                colection.gripper.setPosition(colection.gripper_release);
                extension.extend(extension.extension_retracted);
            } else {
                drive.updatePoseEstimate();
                TrajectoryActionBuilder sample_collect1 = drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(new Vector2d(-20, x), Math.toRadians(0));
                Actions.runBlocking(
                        new SequentialAction(
                                sample_collect1.build()
                        ));
                x += 2.5;
            }
        }

// ---------------------------------------------------------------------------
// Final adjustments after second detection loop for sample_4.
// ---------------------------------------------------------------------------
        colection.gripper_height.setPosition(colection.height_transfer);
        colection.gripper_rotation.setPosition(colection.gripper_rotation_default);
        colection.gripper_angle.setPosition(colection.gripper_angle_default);
        extension.extend(extension.extension_transfer + 0.08);
        sleep(200);
        extension.extend(extension.extension_transfer);

// ---------------------------------------------------------------------------
// Execute final finishing trajectory for sample_4 (again) – sample_finish_5
// ---------------------------------------------------------------------------
        TrajectoryActionBuilder sample_finish_5 = drive.actionBuilder(drive.pose)
                .afterTime(0.2, scoring.gripper_grab())
                .afterTime(0.4, colection.griper_release())
                .afterTime(0.5, slides.slide_sample())
                .afterTime(0.7, scoring.sample_score_retracted())
                .strafeToLinearHeading(new Vector2d(-60, -47), Math.toRadians(70));
        Actions.runBlocking(
                new SequentialAction(
                        sample_finish_5.build()
                ));
        extension.extend(extension.extension_retracted);
        sleep(200);
        scoring.gripper(scoring.gripper_release);
        sleep(200);
        scoring.scoring_arm_default();

// ---------------------------------------------------------------------------
// Reset scoring arm and collection system before final trajectory.
// ---------------------------------------------------------------------------
        scoring.scoring_arm_extension.setPosition(scoring.extension_retracted);
        colection.default_config();
        colection.gripper_angle.setPosition(colection.gripper_angle_default);
        drive.updatePoseEstimate();

// ---------------------------------------------------------------------------
// Execute sample_4 trajectory again to complete the collection process.
// ---------------------------------------------------------------------------
        Actions.runBlocking(
                new SequentialAction(
                        sample_4.build()
                ));
        while (opModeIsActive()) {
            sleep(200);
            if (ll.is_detecting()) {
                ll.rot_detect(colection.gripper_rotation);
                ll.extend_detect(extension.left_extension, extension.right_extension);
                colection.gripper_release();
                colection.gripper_height.setPosition(colection.height_scanning);
                taim.reset();
                while (taim.seconds() < 0.4 && opModeIsActive()) {
                    telemetry.addData("samp4", samp4);
                    telemetry.update();
                    ll.angle_detect(colection.gripper_angle);
                }
                colection.gripper_height.setPosition(colection.height_collecting);
                sleep(200);
                colection.gripper.setPosition(colection.gripper_transfer);
                sleep(300);
                colection.gripper_height.setPosition(colection.height_default);
                sleep(300);
                if (colection.senzor.getDistance(DistanceUnit.CM) < colection.distance_to_collected_sample) {
                    extension.extend(extension.extension_transfer + 0.08);
                    break;
                }
                sleep(300);
                colection.gripper_height.setPosition(colection.height_default);
                colection.gripper_rotation.setPosition(colection.gripper_rotation_default);
                colection.gripper.setPosition(colection.gripper_release);
                extension.extend(extension.extension_retracted);
            } else {
                drive.updatePoseEstimate();
                TrajectoryActionBuilder sample_collect1 = drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(new Vector2d(-20, x), Math.toRadians(0));
                Actions.runBlocking(
                        new SequentialAction(
                                sample_collect1.build()
                        ));
                x += 2.5;
            }
        }

// ---------------------------------------------------------------------------
// Final adjustments and finishing trajectory for sample_4 (sample_finish_6).
// ---------------------------------------------------------------------------
        colection.gripper_height.setPosition(colection.height_transfer);
        colection.gripper_rotation.setPosition(colection.gripper_rotation_default);
        colection.gripper_angle.setPosition(colection.gripper_angle_default);
        extension.extend(extension.extension_transfer + 0.08);
        sleep(200);
        extension.extend(extension.extension_transfer);
        TrajectoryActionBuilder sample_finish_6 = drive.actionBuilder(drive.pose)
                .afterTime(0.2, scoring.gripper_grab())
                .afterTime(0.4, colection.griper_release())
                .afterTime(0.5, slides.slide_sample())
                .afterTime(0.7, scoring.sample_score_retracted())
                .strafeToLinearHeading(new Vector2d(-60, -47), Math.toRadians(70));
        Actions.runBlocking(
                new SequentialAction(
                        sample_finish_6.build()
                ));
        extension.extend(extension.extension_retracted);
        sleep(200);
        scoring.gripper(scoring.gripper_release);
        sleep(200);
        scoring.scoring_arm_default();

// ---------------------------------------------------------------------------
// Final reset and update before ending autonomous routine.
// ---------------------------------------------------------------------------
        scoring.scoring_arm_extension.setPosition(scoring.extension_retracted);
        colection.default_config();
        colection.gripper_angle.setPosition(colection.gripper_angle_default);
        sleep(200);
        telemetry.update();
    }
}
