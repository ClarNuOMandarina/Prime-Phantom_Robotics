package org.firstinspires.ftc.teamcode.user;

// Import necessary libraries for robot actions, trajectory building, and hardware components

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;

import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.PinpointDrive;
// Safe autonomous with no limelight, consistent and slow 5 sample
@Autonomous(name="auto_basket_5_sample") // Autonomous operation mode declaration
public class auto_5_sample_basket extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Initial robot pose (position and heading) set at (-40, -66) with 0 radians rotation
        Pose2d initialPose = new Pose2d(new Vector2d(-40, -66), Math.toRadians(0));


        // Initialize robot subsystems: drive, collection, extension, scoring, slides
        PinpointDrive drive = new PinpointDrive(hardwareMap, initialPose);
        colection colection = new colection(hardwareMap);
        extension extension = new extension(hardwareMap);
        scoring scoring = new scoring(hardwareMap);
        slides slides = new slides(hardwareMap);

        // Set initial configurations for various subsystems
        slides.slide_init(); // Initialize slide position
        colection.init_config(); // Initialize collection mechanism
        colection.gripper_rotation.setPosition(colection.gripper_rotation_default); // Set default gripper rotation
        extension.extend(extension.extension_retracted); // Retract extension
        scoring.scoring_arm_score_specimen_score(); // Set scoring arm to score specimen position
        slides.reset_encoder(); // Reset slide encoder
        scoring.gripper_grab(); // Grab with the gripper
        ElapsedTime timer = new ElapsedTime(); // Create timer for transfer sequence
        boolean transferz = false; // Flag for managing transfer actions

        // Define robot's trajectory for various actions and movements
        // Trajectory to move to scoring position and prepare for preload
        TrajectoryActionBuilder start_to_score_to_preload = drive.actionBuilder(new Pose2d(new Vector2d(-57, -59), Math.toRadians(46.5)))
                .afterTime(0.4, slides.slide_init()) // Initialize slide after 0.4 seconds
                .afterTime(0.7, slides.slide_init()) // Re-initialize slide after 0.7 seconds
                .afterTime(0.1, scoring.sample_collect()) // Collect sample after 0.1 seconds
                .strafeToLinearHeading(new Vector2d(31, -65), Math.toRadians(0)); // Strafe to the pre-load position

        // Trajectory to finish preload and move back to scoring position
        TrajectoryActionBuilder sample_finish_preload = drive.actionBuilder(new Pose2d(new Vector2d(31, -65), Math.toRadians(0)))
                .afterTime(0.6, scoring.gripper_grab()) // Grab the object after 0.6 seconds
                .afterTime(0.8, colection.griper_release()) // Release the gripper after 0.8 seconds
                .afterTime(1, slides.slide_sample()) // Move slides to sample position after 1 second
                .afterTime(1, scoring.sample_score()) // Score the sample after 1 second
                .strafeToLinearHeading(new Vector2d(-57, -59), Math.toRadians(46.5)); // Move back to the scoring position

        // Trajectory to start scoring at the initial position
        TrajectoryActionBuilder start_to_score = drive.actionBuilder(initialPose)
                .afterTime(0, slides.slide_sample()) // Move slides to sample position immediately
                .afterTime(0.4, slides.slide_sample()) // Additional slide movement after 0.4 seconds
                .afterTime(0.8, colection.collecting_arm_default()) // Reset collection arm after 0.8 seconds
                .strafeToLinearHeading(new Vector2d(-57, -59), Math.toRadians(46.5)); // Strafe to scoring position

        // Define more trajectories for sample collection and finishing the actions
        TrajectoryActionBuilder sample_1 = drive.actionBuilder(new Pose2d(new Vector2d(-57, -59), Math.toRadians(43)))
                .afterTime(0.4, slides.slide_init()) // Initialize slide
                .afterTime(0.7, slides.slide_init()) // Re-initialize slide
                .afterTime(0.1, scoring.sample_collect()) // Collect sample
                .strafeToLinearHeading(new Vector2d(-49, -47), Math.toRadians(90)); // Strafe to sample collection position

        TrajectoryActionBuilder sample_finish = drive.actionBuilder(new Pose2d(new Vector2d(-49, -47), Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(-58, -57), Math.toRadians(43)); // Strafe to finish collecting sample

        // Define additional trajectories for samples 2 and 3, similar to sample_1
        TrajectoryActionBuilder sample_2 = drive.actionBuilder(new Pose2d(new Vector2d(-57, -57), Math.toRadians(43)))
                .afterTime(0.4, slides.slide_init())
                .afterTime(0.7, slides.slide_init())
                .afterTime(0.5, colection.collecting_arm_default())
                .strafeToLinearHeading(new Vector2d(-60, -45.5), Math.toRadians(92));

        TrajectoryActionBuilder sample_finish_2 = drive.actionBuilder(new Pose2d(new Vector2d(-60, -45.5), Math.toRadians(92)))
                .strafeToLinearHeading(new Vector2d(-58, -57), Math.toRadians(43));

        TrajectoryActionBuilder sample_3 = drive.actionBuilder(new Pose2d(new Vector2d(-57, -57), Math.toRadians(43)))
                .afterTime(0.4, slides.slide_init())
                .afterTime(0.7, slides.slide_init())
                .afterTime(0.5, colection.collecting_arm_default())
                .strafeToLinearHeading(new Vector2d(-49, -26), Math.toRadians(-180));

        TrajectoryActionBuilder sample_3_end = drive.actionBuilder(new Pose2d(new Vector2d(-49, -26), Math.toRadians(-180)))
                .strafeTo(new Vector2d(-42, -26));

        TrajectoryActionBuilder sample_finish_3 = drive.actionBuilder(new Pose2d(new Vector2d(-42, -30), Math.toRadians(-180)))
                .strafeToLinearHeading(new Vector2d(-57.5, -57), Math.toRadians(30));

        // Define parking trajectories
        TrajectoryActionBuilder parking_pre = drive.actionBuilder(new Pose2d(new Vector2d(-57.5, -57), Math.toRadians(30)))
                .afterTime(1, slides.auto_park_basket()) // Auto-park basket after 1 second
                .afterTime(1, scoring.specimen_collect()) // Collect specimen after 1 second
                .strafeToLinearHeading(new Vector2d(-43, -13), Math.toRadians(0)); // Strafe to parking position

        TrajectoryActionBuilder parking = drive.actionBuilder(new Pose2d(new Vector2d(-43, -13), Math.toRadians(0)))
                .afterTime(0.2, slides.auto_park_basket()) // Auto-park basket after 0.2 seconds
                .afterTime(0.2, scoring.specimen_collect()) // Collect specimen after 0.2 seconds
                .strafeTo(new Vector2d(-25, -11)); // Strafe to final parking position

        scoring.grip_transfer.setPosition(scoring.gripper_hold); // Hold the grip transfer mechanism
        // Wait for the start signal to begin the autonomous routine
        waitForStart();
        if (isStopRequested()) return;
        // Begin scoring by positioning the arm
        scoring.scoring_arm_extension.setPosition(scoring.extension_retracted); // Retract arm
        scoring.scoring_arm_score_basket(); // Move arm to score basket position
        slides.culisante(slides.slides_high_basket); // Set slides to high basket position
        // Follow the mentioned trajectory with an Action based sistem
        Actions.runBlocking(
                new SequentialAction(
                        start_to_score.build()
                ));
        scoring.gripper(scoring.gripper_release); // Release the gripper to get ready for the next action
        sleep(400); // Pause for 400ms to ensure the action completes
        Actions.runBlocking(
                new SequentialAction(
                        start_to_score_to_preload.build()
                )); // Move to the pre-load position to prepare for collection

// Reset the scoring arm and reconfigure the robot's collection settings
        scoring.scoring_arm_default();
        scoring.scoring_arm_extension.setPosition(scoring.extension_retracted);

// Reposition the collection gripper for proper collection height
        colection.gripper_height.setPosition(colection.height_collecting_retracted);
        sleep(200); // Pause for 200ms to ensure height adjustment
        colection.gripper_grab(); // Grabbing the object
        sleep(300); // Pause for 300ms to allow the grab action to complete
        colection.scoring_config(); // Set the collection configuration for scoring
        sleep(200); // Pause for 200ms to finalize configuration

// Output some telemetry data for debugging/monitoring
        telemetry.addData("A", 0);
        telemetry.update();

// Extend the arm for transfer
        extension.extend(extension.extension_transfer);
        Actions.runBlocking(
                new SequentialAction(
                        sample_finish_preload.build()
                )); // Finish the sample transfer

        telemetry.addData("A", 1);
        telemetry.update(); // Update telemetry

// Release the grip transfer and update telemetry for next actions
        scoring.grip_transfer.setPosition(scoring.gripper_release);
        telemetry.addData("A", 2);
        telemetry.update();
        sleep(400); // Pause for 400ms to ensure the release is completed

// Reset the collection configuration
        colection.default_config();
        colection.gripper_angle.setPosition(colection.gripper_angle_default);
        sleep(200); // Pause for 200ms to allow the gripper angle to reset

// Begin the first sample collection and transfer
        Actions.runBlocking(
                new SequentialAction(
                        sample_1.build()
                ));

// Reset scoring arm and collection configuration before grabbing the next sample
        scoring.scoring_arm_default();
        scoring.scoring_arm_extension.setPosition(scoring.extension_retracted);
        colection.default_config();
        colection.gripper_height.setPosition(colection.height_collecting);
        sleep(200); // Pause for 200ms before grabbing the next sample
        colection.gripper_grab(); // Grab the next sample
        sleep(300); // Pause for 300ms to ensure the gripper action completes
        colection.scoring_config(); // Reconfigure the collection mechanism for scoring
        colection.gripper.setPosition(colection.gripper_transfer); // Position the gripper for transfer
        extension.extend(extension.extension_extended); // Extend the arm for transfer
        sleep(400); // Pause for 400ms to ensure arm is extended

// Initialize the timer for managing transfer timing
        timer.reset();
        transferz = true; // Set the transfer flag to true, starting the transfer process

// Execute the transfer process, ensuring correct timing for each action
        while (transferz) {
            extension.extend(extension.extension_transfer); // Continue extending the arm for transfer

            // If timer is between 0.4s and 0.5s, hold the grip transfer mechanism
            if (timer.seconds() > 0.4 && timer.seconds() < 0.5) {
                scoring.grip_transfer.setPosition(scoring.gripper_hold);
            }

            // If timer is between 0.5s and 0.6s, release the gripper
            if (timer.seconds() > 0.5 && timer.seconds() < 0.6) {
                colection.gripper.setPosition(colection.gripper_release);
            }

            // If timer is greater than 0.8s, reset configuration and move to high basket position
            if (timer.seconds() > 0.8) {
                colection.default_config(); // Reset collection configuration
                scoring.scoring_arm_extension.setPosition(scoring.extension_extended); // Extend arm for scoring
                slides.culisante(slides.slides_high_basket); // Position the slides for high basket scoring
                transferz = false; // Set transfer flag to false to end the loop
            }
        }

// Perform scoring at the high basket
        scoring.scoring_arm_score_basket();
        sleep(200); // Pause for 200ms to allow scoring action to complete

// Finish the first sample transfer
        Actions.runBlocking(
                new SequentialAction(
                        sample_finish.build()
                ));

// Release the gripper after scoring
        scoring.gripper(scoring.gripper_release);
        sleep(400); // Pause for 400ms to ensure the release is completed
        scoring.scoring_arm_extension.setPosition(scoring.extension_retracted); // Retract scoring arm

// Begin the second sample collection and transfer
        Actions.runBlocking(
                new SequentialAction(
                        sample_2.build()
                ));

// Reset and prepare for the second transfer
        sleep(200);
        scoring.scoring_arm_default();
        colection.gripper_height.setPosition(colection.height_collecting);
        sleep(200);
        colection.gripper_grab(); // Grab the second sample
        sleep(300); // Pause for 300ms
        colection.scoring_config(); // Set up for scoring
        colection.gripper.setPosition(colection.gripper_transfer); // Transfer the gripper
        extension.extend(extension.extension_extended); // Extend arm
        sleep(400); // Pause for 400ms before starting transfer

// Reset timer and begin transfer loop for the second sample
        timer.reset();
        transferz = true;

        while (transferz) {
            extension.extend(extension.extension_transfer); // Extend for transfer

            if (timer.seconds() > 0.4 && timer.seconds() < 0.5) {
                scoring.grip_transfer.setPosition(scoring.gripper_hold); // Hold the transfer
            }

            if (timer.seconds() > 0.5 && timer.seconds() < 0.6) {
                colection.gripper.setPosition(colection.gripper_release); // Release the gripper
            }

            if (timer.seconds() > 0.8) {
                colection.default_config(); // Reset collection configuration
                scoring.scoring_arm_extension.setPosition(scoring.extension_extended); // Extend arm for high basket
                slides.culisante(slides.slides_high_basket); // Position slides for high basket
                transferz = false; // End transfer loop
            }
        }

// Score at the high basket with second sample
        scoring.scoring_arm_score_basket();
        sleep(200); // Pause for 200ms

// Finish second sample transfer
        Actions.runBlocking(
                new SequentialAction(
                        sample_finish_2.build()
                ));

// Release the gripper after scoring the second sample
        scoring.gripper(scoring.gripper_release);
        sleep(400); // Pause for 400ms
        scoring.scoring_arm_extension.setPosition(scoring.extension_retracted); // Retract scoring arm

// Begin the third sample collection and transfer
        Actions.runBlocking(
                new SequentialAction(
                        sample_3.build()
                ));

        colection.gripper_angle.setPosition(colection.gripper_angle_vertical); // Position gripper vertically for collection
        sleep(300); // Pause for 300ms

// Reset scoring arm and prepare for the third collection
        scoring.scoring_arm_default();
        colection.gripper_height.setPosition(colection.height_collecting);
        sleep(200);
        colection.gripper_grab(); // Grab the third sample
        sleep(300); // Pause for 300ms
        colection.gripper_angle.setPosition(colection.gripper_angle_default); // Reset gripper angle

// Finish the third sample transfer
        Actions.runBlocking(
                new SequentialAction(
                        sample_3_end.build()
                ));

        scoring.scoring_arm_extension.setPosition(scoring.extension_retracted); // Retract arm for scoring
        colection.scoring_config(); // Reset collection config

// Reconfigure gripper for transfer
        colection.gripper.setPosition(colection.gripper_transfer);
        extension.extend(extension.extension_extended); // Extend arm for transfer
        sleep(400); // Pause for 400ms before starting transfer

// Reset timer for third transfer
        timer.reset();
        transferz = true; // Set transfer flag to true

        while (transferz) {
            extension.extend(extension.extension_transfer); // Continue extending arm

            if (timer.seconds() > 0.4 && timer.seconds() < 0.5) {
                scoring.grip_transfer.setPosition(scoring.gripper_hold); // Hold the grip transfer
            }

            if (timer.seconds() > 0.5 && timer.seconds() < 0.6) {
                colection.gripper.setPosition(colection.gripper_release); // Release gripper
            }

            if (timer.seconds() > 0.8) {
                colection.default_config(); // Reset collection configuration
                slides.culisante(slides.slides_high_basket); // Position slides for high basket scoring
                transferz = false; // End transfer loop
            }
        }

// Score at the high basket with third sample
        scoring.scoring_arm_score_basket();
        sleep(200); // Pause for 200ms

// Finish third sample transfer
        Actions.runBlocking(
                new SequentialAction(
                        sample_finish_3.build()
                ));

// Release the gripper after scoring
        scoring.gripper(scoring.gripper_release);
        sleep(300); // Pause for 300ms
        colection.gripper_rotation.setPosition(colection.gripper_rotation_default); // Reset gripper rotation
        colection.gripper_height.setPosition(colection.height_default); // Reset gripper height

// Perform final parking actions
        Actions.runBlocking(
                new SequentialAction(
                        parking_pre.build(),
                        parking.build()
                ));

// Score specimen and collect the specimen
        scoring.scoring_arm_score_specimen_collect();
        scoring.grip_transfer_release(); // Release grip transfer mechanism
        colection.init_config(); // Reset collection configuration

        telemetry.update(); // Final update to telemetry
    }
}