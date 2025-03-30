package org.firstinspires.ftc.teamcode.user;

// ---------------------------------------------------------------------------
// Import Statements:
// ---------------------------------------------------------------------------
// These imports include Road Runner classes for trajectory and pose handling,
// FTC SDK classes for opmode control, and others for sensor and telemetry functionality.
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.PinpointDrive;
// Autonomous sequence for scoring 6 specimens on the high bar using Limelight automated colection sistem

@Autonomous(name="auto spec experimental")
public class auto_specimen_experimental extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // -------------------------------------------------------------------
        // INITIALIZATION:
        // -------------------------------------------------------------------
        // Set the initial pose for the robot at coordinate (9, -60) with a heading of 90°.
        Pose2d initialPose = new Pose2d(new Vector2d(9, -60), Math.toRadians(90));

        // Initialize the Limelight for vision-based detection (e.g., detecting specimen samples)
        Limelight ll = new Limelight(hardwareMap);
        telemetry.setMsTransmissionInterval(11); // Fast telemetry updates (11ms interval)

        // Initialize subsystems for driving, collection, extension, scoring, and slides.
        PinpointDrive drive = new PinpointDrive(hardwareMap, initialPose);
        colection colection = new colection(hardwareMap);
        extension extension = new extension(hardwareMap);
        scoring scoring = new scoring(hardwareMap);
        slides slides = new slides(hardwareMap);

        // -------------------------------------------------------------------
        // HARDWARE SETUP:
        // -------------------------------------------------------------------
        // Move the slides to the initial position.
        slides.slide_init();
        // Initialize the collection system.
        colection.init_config();
        // Ensure the extension mechanism is retracted.
        extension.extend(extension.extension_retracted);
        // Initialize the scoring subsystem configuration.
        scoring.init_config();
        // Set the scoring mechanism's grip transfer to the grab state.
        scoring.grip_transfer_grab();
        // Reset the slides' encoder for accurate movement tracking.
        slides.reset_encoder();
        // Initialize variable x for later use in adjustments (e.g., scanning offsets).
        double x = 0;

        // -------------------------------------------------------------------
        // TRAJECTORY DEFINITIONS:
        // -------------------------------------------------------------------
        // These trajectory builders define the robot's path and timed actions.
        // They include movements (strafe, spline) and subsystem actions (slides, scoring, collection).

        // Trajectory: start_to_score
        // Drives from the initial pose to a scoring position while continuously triggering the specimen scoring mechanism.
        TrajectoryActionBuilder start_to_score = drive.actionBuilder(initialPose)
                .afterTime(0, slides.specimen_score_high())    // Immediately begin high specimen scoring
                .afterTime(0.7, slides.specimen_score_high())  // Reinforce scoring position after 0.7 sec
                .afterTime(0.2, slides.specimen_score_high())  // Additional scoring activation at 0.2 sec
                .afterTime(1, slides.specimen_score_high())    // Maintain scoring position after 1 sec
                .afterTime(1, scoring.gripper_grab_max())        // Grab with maximum grip after 1 sec
                .strafeTo(new Vector2d(6, -26));                 // Strafe to coordinate (6, -26)

        // Trajectory: transfer_sample_1
        // Drives from the scoring area to a transfer area for the first sample.
        TrajectoryActionBuilder transfer_sample_1 = drive.actionBuilder(new Pose2d(new Vector2d(6, -26), Math.toRadians(90)))
                .afterTime(0, scoring.specimen_score())          // Begin specimen scoring action immediately
                .afterTime(0.2, scoring.gripper_release())         // Release the gripper after 0.2 sec
                .afterTime(0.3, scoring.extension_retracted())     // Retract the extension after 0.3 sec
                .afterTime(0.4, scoring.specimen_collect())        // Trigger specimen collection after 0.4 sec
                .afterTime(0.4, slides.slide_init())               // moves the slides to the init position after 0.4 sec
                .strafeToLinearHeading(new Vector2d(20, -39), Math.toRadians(0))  // Move to (20, -39) with 0° heading
                .strafeToLinearHeading(new Vector2d(50, -8), Math.toRadians(75))  // Then to (50, -8) with 75° heading
                // .splineToConstantHeading(new Vector2d(50,-14), Math.PI/8)        // Optionally use spline (commented out)
                .strafeToLinearHeading(new Vector2d(53, -50), Math.toRadians(90)); // Finally, move to (53, -50) with 90° heading

        // Trajectory: transfer_sample_2
        // Drives from the previous position to collect a second sample.
        TrajectoryActionBuilder transfer_sample_2 = drive.actionBuilder(new Pose2d(new Vector2d(53, -50), Math.toRadians(90)))
                .afterTime(0, scoring.sample_collect())          // Immediately trigger sample collection
                .splineToConstantHeading(new Vector2d(60, -14), Math.PI / 8)  // Spline path to (60, -14)
                .strafeToLinearHeading(new Vector2d(61, -50), Math.toRadians(90)); // Then strafe to (61, -50) with 90° heading

        // Trajectory: transfer_sample_3
        // Drives to the position for the first specimen collection.
        TrajectoryActionBuilder transfer_sample_3 = drive.actionBuilder(new Pose2d(new Vector2d(61, -50), Math.toRadians(90)))
                .afterTime(0, scoring.specimen_first_collect())  // Trigger first specimen collection routine immediately
                .splineToConstantHeading(new Vector2d(68, -14), Math.PI / 8); // Spline path to (68, -14)

        // Trajectory: specimen_collect_pre
        // Moves from the current position to a pre-collection position for the specimen.
        TrajectoryActionBuilder specimen_collect_pre = drive.actionBuilder(new Pose2d(new Vector2d(68, -14), Math.toRadians(90)))
                .strafeToConstantHeading(new Vector2d(65, -54)); // Strafe to (65, -54) maintaining heading

        // Trajectory: scoring_poz_pre
        // Prepares the robot for final scoring by adjusting rotation, specimen preparation, and slides.
        TrajectoryActionBuilder scoring_poz_pre = drive.actionBuilder(new Pose2d(new Vector2d(65, -54), Math.toRadians(90)))
                .afterTime(1.5, colection.rotation_default())     // Reset rotation after 1.5 sec
                .afterTime(0, scoring.specimen_prepare())          // Begin specimen preparation immediately
                .afterTime(0, slides.specimen_score_high2())         // Activate high scoring configuration (version 2)
                .afterTime(0.7, slides.specimen_score_high2())       // Reinforce scoring after 0.7 sec
                .afterTime(0.2, slides.specimen_score_high2())       // Additional activation at 0.2 sec
                .afterTime(0.7, scoring.gripper_grab_max())          // Grab with maximum grip after 0.7 sec
                .afterTime(1, slides.specimen_score_high2())         // Maintain scoring state after 1 sec
                .strafeTo(new Vector2d(-6, -25));                    // Strafe to coordinate (-6, -25)

        // Trajectory: specimen_collect_pre_cicling
        // An alternate collection path for repeated specimen collection cycles.
        TrajectoryActionBuilder specimen_collect_pre_cicling = drive.actionBuilder(new Pose2d(new Vector2d(6, -25), Math.toRadians(90)))
                .afterTime(0, scoring.specimen_score())            // Begin scoring action
                .afterTime(0.2, scoring.gripper_release())           // Release gripper after 0.2 sec
                .afterTime(0.3, scoring.extension_retracted())       // Retract extension after 0.3 sec
                .afterTime(0.4, scoring.specimen_collect())          // Collect specimen after 0.4 sec
                .afterTime(0.4, slides.slide_init())                 // moves the slides to the init position after 0.4 sec
                .strafeToConstantHeading(new Vector2d(6, -30))       // Strafe to (6, -30)
                .strafeToConstantHeading(new Vector2d(38, -60))      // Then to (38, -60)
                .afterTime(0, colection.griper_release());           // Ensure collection gripper is released

        // Trajectory: scoring_poz
        // Final scoring position trajectory for specimen placement.
        TrajectoryActionBuilder scoring_poz = drive.actionBuilder(new Pose2d(new Vector2d(38, -60), Math.toRadians(90)))
                .afterTime(0, slides.specimen_score_high())          // Trigger high scoring position immediately
                .afterTime(0, scoring.specimen_prepare())            // Begin specimen preparation
                .afterTime(0.7, slides.specimen_score_high())        // Reinforce scoring state after 0.7 sec
                .afterTime(0.2, slides.specimen_score_high())        // Additional activation after 0.2 sec
                .afterTime(1, slides.specimen_score_high())          // Maintain state after 1 sec
                .afterTime(1, scoring.gripper_grab_max())            // Grab with maximum grip after 1 sec
                .strafeToConstantHeading(new Vector2d(4, -26));      // Strafe to (4, -26)

        // Trajectory: parking
        // Defines the final parking path. Executes actions like releasing the gripper,
        // ending autonomous mode, and reinitializing the slides before parking.
        TrajectoryActionBuilder parking = drive.actionBuilder(new Pose2d(new Vector2d(4, -27), Math.toRadians(90)))
                .afterTime(0.3, scoring.gripper_release())           // Release the gripper after 0.3 sec
                .afterTime(0.5, scoring.auto_End())                  // Trigger auto end sequence after 0.5 sec
                .afterTime(1, slides.slide_init())                   // Reset slides after 1 sec
                .strafeToLinearHeading(new Vector2d(45, -60), Math.toRadians(90)); // Strafe to parking coordinate (45, -60)

        // -------------------------------------------------------------------
        // PRE-ACTIVATION CONFIGURATION:
        // -------------------------------------------------------------------
        // Set the scoring gripper to hold and ensure the collection gripper is in the release state.
        scoring.gripper(scoring.gripper_hold);
        colection.gripper.setPosition(colection.gripper_release);

        // Wait for the autonomous period to start.
        waitForStart();

        // -------------------------------------------------------------------
        // ACTION SEQUENCE EXECUTION:
        // -------------------------------------------------------------------
        // The following sequence of Actions.runBlocking() calls executes the defined
        // trajectories and subsystem commands in order.
        // First, configure lights, slides, scoring arm, and gripper before running the trajectories.
        colection.light.setPosition(colection.detection_light); // Turn on the detection light for vision
        slides.culisante(slides.slides_specimen_high_score);      // Move slides to high score position for specimens
        scoring.scoring_arm_specimen_prepare();                    // Prepare the scoring arm for specimen handling
        scoring.grip_transfer.setPosition(scoring.gripper_hold);   // Set the grip transfer to hold position

        // Run a sequence of trajectories for specimen scoring and transfer
        Actions.runBlocking(
                new SequentialAction(
                        start_to_score.build(),      // Drive to the scoring position
                        transfer_sample_1.build(),   // Execute transfer for sample 1
                        colection.rotation_default(),// Reset collection rotation to default
                        transfer_sample_2.build(),   // Transfer sample 2
                        transfer_sample_3.build(),   // Transfer sample 3
                        specimen_collect_pre.build() // Move to pre-collection position for specimen
                ));

        // After transferring, set gripper states and apply timed delays.
        scoring.gripper(scoring.gripper_hold);
        sleep(200);
        scoring.gripper(scoring.gripper_semi_hold);

        // Execute the pre-scoring position trajectory
        Actions.runBlocking(
                new SequentialAction(
                        scoring_poz_pre.build()
                ));

        // Score the specimen by moving the scoring arm to the specimen score position.
        scoring.scoring_arm_score_specimen_score();

        // Create timers for controlling the detection loop
        ElapsedTime taim = new ElapsedTime();
        ElapsedTime taim2 = new ElapsedTime();
        taim2.reset();
        taim.reset();

        // -------------------------------------------------------------------
        // VISION-BASED SPECIMEN COLLECTION LOOP:
        // -------------------------------------------------------------------
        // While the opMode is active and within a 3-second window, use Limelight vision to detect the specimen.
        while (opModeIsActive() && taim2.seconds() < 3) {
            sleep(200);  // Wait 200ms between iterations
            // Continuously set the grip transfer to release.
            scoring.grip_transfer.setPosition(scoring.gripper_release);

            // If Limelight detects a target (i.e., the specimen)
            if (ll.is_detecting()) {
                ll.rot_detect(colection.gripper_rotation);                   // Adjust gripper rotation based on vision data
                ll.extend_detect(extension.left_extension, extension.right_extension); // Adjust extension based on detected distances
                colection.gripper_release();                                   // Ensure the gripper is released
                colection.gripper_height.setPosition(colection.height_scanning); // Set gripper height for scanning
                taim.reset();  // Reset timer to control inner loop

                // For 0.4 seconds, adjust gripper angle based on vision feedback
                while (taim.seconds() < 0.4 && opModeIsActive()) {
                    telemetry.update();
                    ll.angle_detect(colection.gripper_angle);
                }
                // After vision adjustments, set gripper height to collection height
                colection.gripper_height.setPosition(colection.height_collecting);
                sleep(200);
                colection.gripper.setPosition(colection.gripper_transfer);  // Prepare gripper for transfer
                sleep(300);

                // Check the distance sensor; if a specimen is close enough, retract the extension and adjust angles.
                if (colection.senzor.getDistance(DistanceUnit.CM) < colection.distance_to_collected_sample) {
                    extension.extend(extension.extension_retracted);
                    colection.gripper_height.setPosition(colection.height_default);
                    colection.gripper_rotation.setPosition(colection.gripper_rotation_score_sample);
                    colection.gripper_angle.setPosition(colection.gripper_angle_sample_observation);
                    break; // Break out of the loop if specimen is collected
                }
                // If specimen is not within range, reset configurations.
                colection.gripper_height.setPosition(colection.height_default);
                colection.gripper_rotation.setPosition(colection.gripper_rotation_default);
                colection.gripper.setPosition(colection.gripper_release);
                extension.extend(extension.extension_retracted);
            } else {
                // If no target is detected, incrementally adjust extension (e.g., scanning movement)
                extension.extend(extension.extension_retracted + x);
                x += 0.02;
            }
        }
        // After loop, reset mechanisms to default states.
        extension.extend(extension.extension_retracted);
        colection.gripper_height.setPosition(colection.height_default);
        colection.gripper_rotation.setPosition(colection.gripper_rotation_score_sample);
        colection.gripper_angle.setPosition(colection.gripper_angle_vertical);

        // -------------------------------------------------------------------
        // SPECIMEN COLLECT PRE-CYCLING:
        // -------------------------------------------------------------------
        // Build a trajectory for pre-cycling the specimen collection process.
        TrajectoryActionBuilder specimen_collect_pre_cicling_coelcting = drive.actionBuilder(drive.pose)
                .afterTime(0, scoring.specimen_score())
                .afterTime(0.2, scoring.gripper_release())
                .afterTime(0.3, scoring.extension_retracted())
                .afterTime(0.4, scoring.specimen_collect())
                .afterTime(0.4, slides.slide_init())
                .strafeToConstantHeading(new Vector2d(-6, -30))
                .strafeToConstantHeading(new Vector2d(38, -60))
                .afterTime(0, colection.griper_release());

        // Execute the pre-cycling trajectory and ensure the gripper is released.
        Actions.runBlocking(
                new SequentialAction(
                        specimen_collect_pre_cicling_coelcting.build(),
                        colection.griper_release()
                ));
        sleep(200);
        // Adjust gripper state with delays.
        scoring.gripper(scoring.gripper_hold);
        sleep(200);
        scoring.gripper(scoring.gripper_semi_hold);

        // -------------------------------------------------------------------
        // SCORING & SPECIMEN COLLECTION CYCLE:
        // -------------------------------------------------------------------
        // Execute a sequence of trajectories for scoring and collection.
        Actions.runBlocking(
                new SequentialAction(
                        scoring_poz.build(),
                        specimen_collect_pre_cicling.build()
                ));
        colection.gripper_release();
        scoring.gripper(scoring.gripper_hold);
        sleep(200);
        scoring.gripper(scoring.gripper_semi_hold);

        // Repeat the scoring and collection cycle multiple times
        Actions.runBlocking(
                new SequentialAction(
                        scoring_poz.build(),
                        specimen_collect_pre_cicling.build()
                ));
        scoring.gripper(scoring.gripper_hold);
        sleep(200);
        scoring.gripper(scoring.gripper_semi_hold);
        Actions.runBlocking(
                new SequentialAction(
                        scoring_poz.build(),
                        specimen_collect_pre_cicling.build()
                ));
        scoring.gripper(scoring.gripper_hold);
        sleep(200);
        scoring.gripper(scoring.gripper_semi_hold);
        // Final scoring and parking trajectory
        Actions.runBlocking(
                new SequentialAction(
                        scoring_poz.build(),
                        scoring.specimen_score(),
                        parking.build()
                ));

        // -------------------------------------------------------------------
        // FINAL UPDATE:
        // -------------------------------------------------------------------
        // Update the robot's pose estimate and send telemetry data for debugging.
        drive.updatePoseEstimate();
        telemetry.addData("pose", drive.pose);
        telemetry.update();
    }
}
