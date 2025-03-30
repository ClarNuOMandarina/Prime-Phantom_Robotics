package org.firstinspires.ftc.teamcode.user;

// ---------------------------------------------------------------------------
// Import Statements:
// ---------------------------------------------------------------------------
// These imports include Road Runner classes for robot pose, velocity, and trajectories,
// FTC SDK classes for teleop operation and timers, and external navigation units.
// The static imports allow for shorthand telemetry calls and math conversions.
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.PinpointDrive;
// Main teleop
@TeleOp(name="teleop test")
public class teleop_test extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // -------------------------------------------------------------------
        // INITIALIZATION: Setting Up Starting Pose and Subsystems
        // -------------------------------------------------------------------
        // Define the starting position and heading for the robot.
        // In this case, the robot starts at (50, -62) with a 90° orientation.
        Pose2d initpose = new Pose2d(new Vector2d(50, -62), Math.toRadians(90));

        // -------------------------------------------------------------------
        // Subsystem Initialization:
        // -------------------------------------------------------------------
        // Initialize the "colection" subsystem (handles sample collection mechanisms).
        // Calling init_config() sets default positions, speeds, and configurations.
        colection colection = new colection(hardwareMap);
        colection.init_config();

        // Initialize the scoring subsystem responsible for scoring samples.
        scoring scoring = new scoring(hardwareMap);

        // Initialize the slides subsystem which moves vertically (e.g., for reaching high goals).
        slides slides = new slides(hardwareMap);

        // Initialize the extension subsystem which controls a linear actuator or arm extension.
        extension extension = new extension(hardwareMap);

        // Initialize the drive system using PinpointDrive for precise movement control.
        PinpointDrive drive = new PinpointDrive(hardwareMap, initpose);

        // Initialize the hanging mechanism (used for endgame or lifting) and set it to a "stop" state.
        hanging hanging = new hanging(hardwareMap);
        hanging.hang(hanging.stop);

        // -------------------------------------------------------------------
        // BOOLEAN FLAGS AND TIMERS:
        // -------------------------------------------------------------------
        // These boolean flags manage various states and modes throughout the teleop period.
        boolean blockage = false; // True if the robot is transfering samples from one mechanism to the other
        boolean colectng = false; // Indicates if a collection process has been triggered.
        boolean colectng_intermediary = false; // Temporary flag for initiating collection routines.
        boolean is_extending = false; // True when the extension mechanism is actively extending.
        boolean extension_timer = false; // Used to debounce extension toggle actions.
        boolean manual = false; // True if the operator manually overrides automation.
        boolean is_collecting = false; // True when the robot is actively collecting a sample.
        boolean specimen_cycling = false; // Indicates cycling mode for specimen collection.
        boolean specimen_cycling_timer = false; // Debounce flag for toggling specimen cycling.
        boolean specimen_cycling_intermediary = false; // Temporary flag for specimen cycling state changes.
        boolean transfer_extend = false; // Indicates if the transfer process should extend the mechanism.
        boolean transfer_extend_counter = false; // Debounce flag for transfer extension.
        boolean transfer_retracted = false; // Indicates if the transfer process should retract the mechanism.
        boolean transfer_retracted_counter = false; // Debounce flag for transfer retraction.
        boolean colection_check = false; // True when a check is performed to see if a sample is within range.
        boolean basket_score = false; // True when the robot is in a scoring (basket) mode.
        boolean basket_reset = false; // True when the scoring mechanism should be reset.
        boolean to_score = false; // True when the robot is transitioning to a scoring state.
        boolean specimen_preparing = false; // True when the scoring arm is preparing the specimen for scoring.
        boolean sample_to_observation = false; // Indicates a transition to an observation state.
        boolean specimen_scoring = false; // True when the robot is scoring a specimen.
        boolean high_basket = true; // True if the scoring target is at a high level.
        boolean high_basket_intermediary = false; // Temporary flag for toggling between high and low basket states.
        boolean basket_scoring = false; // Indicates if the basket scoring routine is active.
        boolean is_collecting_check = false; // True if a collection check is active.
        boolean specimen_collected = false; // True when a specimen has been successfully collected.
        boolean tohang = false; // True if the robot should transition to hanging mode.
        boolean auto_hang = false; // True if automatic hanging is active.
        boolean specimen_automatization = true; // Enables automated specimen collection.
        boolean automatization_intermediary = false; // Temporary flag for automated routine state changes.
        boolean automatization_running = false; // True when the automation routine is currently running.
        boolean stop_code = false; // Flag to stop code execution (unused in this snippet).
        boolean automatization_finish = false; // True when automation has finished.

        // slidezz holds target positions for the slides (vertical mechanism).
        int slidezz = 0;

        // Ensure the extension mechanism is retracted at the start.
        extension.extend(extension.extension_retracted);

        // Initialize scoring subsystem configuration.
        scoring.init_config();
        boolean is_collected = false; // Flag indicating if a sample has been collected.



        // -------------------------------------------------------------------
        // TIMER INITIALIZATIONS:
        // -------------------------------------------------------------------
        // The following timers are used to debounce inputs and manage delays.
        // 'timer' is used for general delays (e.g., during blockage recovery).
        ElapsedTime timer = new ElapsedTime(0);
        // 'timer_colection' is used to time collection routines and sensor checks.
        ElapsedTime timer_colection = new ElapsedTime(0);
        // 'timer_extension' is used to debounce extension toggle commands.
        ElapsedTime timer_extension = new ElapsedTime(0);
        // 'timer2' is used for short delays during collection trigger sequences.
        ElapsedTime timer2 = new ElapsedTime(0);

        // Later in the code, additional timers for vision loops are declared:
        // (These are declared further down in the code where they are needed.)



        // -------------------------------------------------------------------
        // TRAJECTORY DEFINITION:
        // -------------------------------------------------------------------
        // Define a trajectory for the specimen scoring position.
        // Timed actions adjust slides and scoring arm before strafing to the target coordinate.
        TrajectoryActionBuilder scoring_poz = drive.actionBuilder(initpose)
                .afterTime(0, slides.specimen_score_high())    // Activate high scoring mode immediately
                .afterTime(0, scoring.specimen_prepare())        // Prepare the scoring arm for specimen scoring
                .afterTime(0.7, slides.specimen_score_high())      // Reinforce high scoring state after 0.7 sec
                .afterTime(0.2, slides.specimen_score_high())      // Additional activation after 0.2 sec
                .afterTime(1, slides.specimen_score_high())        // Maintain high scoring state after 1 sec
                .afterTime(2, scoring.gripper_grab_max())          // Grab sample with maximum grip after 2 sec
                .strafeToConstantHeading(new Vector2d(6, -25));    // Strafe to coordinate (6, -25) keeping heading constant

        // -------------------------------------------------------------------
        // PRE-START LOOP:
        // -------------------------------------------------------------------
        // Before the opMode is started, make the slide move to the position it was initialized in the last autonomous.
        while (!isStarted() && !isStopRequested()) {
            slides.culisante(0); // Set slides to initial (0) position.
            telemetry.addData("Status", "Waiting for start...");
            telemetry.update();
        }
        // Wait for the official start signal.
        waitForStart();

        // -------------------------------------------------------------------
        // POST-START INITIAL SETUP:
        // -------------------------------------------------------------------
        // After starting, reset the slide encoder and reinitialize scoring configuration.
        slides.reset_encoder();
        scoring.init_config();
        // Set the gripper rotation of the collection system to its default position.
        colection.gripper_rotation.setPosition(colection.gripper_rotation_default);

        // -------------------------------------------------------------------
        // MAIN TELEOP LOOP:
        // -------------------------------------------------------------------
        // This loop continuously runs while the opMode is active.
        while (opModeIsActive()) {
            // -------------------------------------------------------------------
            // MANUAL DRIVE CONTROL:
            // -------------------------------------------------------------------
            // When automation is not running, use gamepad inputs to control the drive system.
            if (!automatization_running) {
                // Use reduced power when the extension is partially extended and no sample is collected.
                if (extension.left_extension.getPosition() > extension.extension_retracted + 0.1 && !is_collected) {
                    drive.setDrivePowers(new PoseVelocity2d(
                            new Vector2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x * 0.7),
                            -gamepad1.right_stick_x * 0.5));
                } else {
                    drive.setDrivePowers(new PoseVelocity2d(
                            new Vector2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x),
                            -gamepad1.right_stick_x));
                }

                // -------------------------------------------------------------------
                // SLIDE, HANGER, AND GRIPPER ADJUSTMENTS:
                // -------------------------------------------------------------------
                // If the robot is not in hanging mode, update slides and check operator inputs.
                if (!tohang) {
                    slides.culisante(slidezz); // Set slides to the current target position in 'slidezz'

                    // Process operator inputs when the robot is not transfering samples
                    if (!blockage) {
                        // Trigger hanging specimen scoring if both options and share buttons are pressed.
                        if (gamepad1.options && gamepad1.share) {
                            timer.reset();
                            scoring.scoring_arm_score_specimen_collect();
                            tohang = true;
                        }
                        // Set the gripper angle based on left trigger input:
                        // - If left trigger is not pressed, use default angle.
                        // - If pressed, set to vertical.
                        if (!transfer_extend_counter && !transfer_retracted_counter && !transfer_extend && !transfer_retracted) {
                            if (gamepad1.left_trigger == 0) {
                                colection.gripper_angle.setPosition(colection.gripper_angle_default);
                            } else {
                                colection.gripper_angle.setPosition(colection.gripper_angle_vertical);
                            }
                        }
                        // Toggle high basket mode when dpad_left is pressed.
                        if (gamepad1.dpad_left && !high_basket_intermediary) {
                            if (basket_score && slides.right_slide.getCurrentPosition() < 800)
                                slidezz = slides.slides_high_basket;
                            else if (basket_score && slides.right_slide.getCurrentPosition() > 800)
                                slidezz = slides.slides_low_basket;
                            if (!basket_score) {
                                high_basket_intermediary = true;
                                high_basket = !high_basket;
                                timer_extension.reset();
                            } else {
                                high_basket_intermediary = true;
                                timer_extension.reset();
                            }
                        }
                        // Reset the intermediary flag after a short delay.
                        if (high_basket_intermediary) {
                            if (timer_extension.seconds() > 0.3) {
                                high_basket_intermediary = false;
                            }
                        }
                        // Toggle the extension mechanism using the right bumper.
                        if (gamepad1.right_bumper && !extension_timer) {
                            extension_timer = true;
                            is_extending = !is_extending;
                            timer_extension.reset();
                        }
                        if (extension_timer) {
                            if (timer_extension.seconds() > 0.3) {
                                extension_timer = false;
                            }
                        }
                        // Command the extension: extend if is_extending is true; otherwise retract.
                        if (!transfer_extend_counter && !transfer_retracted_counter && !transfer_extend && !transfer_retracted) {
                            if (is_extending)
                                extension.extend(extension.extension_extended);
                            else
                                extension.extend(extension.extension_retracted);
                        }
                        // Toggle specimen cycling using left bumper.
                        if (gamepad1.left_bumper && !specimen_cycling_timer) {
                            specimen_cycling_timer = true;
                            timer.reset();
                        }
                        if (specimen_cycling_timer) {
                            if (timer.seconds() > 0.5) {
                                specimen_cycling_timer = false;
                                specimen_cycling = !specimen_cycling;
                                specimen_cycling_intermediary = true;
                            }
                        }
                        // Based on specimen cycling state, adjust scoring arm and collection configuration.
                        if (specimen_cycling && specimen_cycling_intermediary) {
                            scoring.scoring_arm_score_specimen_collect();
                            scoring.scoring_arm_extension.setPosition(scoring.extension_retracted);
                            colection.default_config();
                            specimen_cycling_intermediary = false;
                            colection.gripper.setPosition(colection.gripper_release);
                            scoring.gripper(scoring.gripper_release);
                        } else if (!specimen_cycling && specimen_cycling_intermediary) {
                            scoring.scoring_arm_default();
                            scoring.scoring_arm_extension.setPosition(scoring.extension_retracted);
                            colection.default_config();
                            specimen_cycling_intermediary = false;
                            colection.gripper.setPosition(colection.gripper_release);
                            scoring.gripper(scoring.gripper_release);
                        }
                        // Manage collection based on right trigger:
                        // When pressed, set gripper height to collecting position; otherwise, reset.
                        if (!basket_scoring) {
                            if (!colection_check && !is_collected) {
                                if (gamepad1.right_trigger != 0) {
                                    colection.gripper_height.setPosition(colection.height_collecting);
                                    is_collecting = true;
                                }
                                if (gamepad1.right_trigger == 0) {
                                    colection.gripper_height.setPosition(colection.height_default);
                                    is_collecting = false;
                                }
                            }
                        } else {
                            if (gamepad1.right_trigger != 0) {
                                scoring.scoring_arm_extension.setPosition(scoring.extension_extended_specimen);
                            } else {
                                scoring.scoring_arm_extension.setPosition(scoring.extension_extended);
                            }
                        }
                        // Update the flag for checking if the robot is collecting.
                        is_collecting_check = is_collecting;

                        // -------------------------------------------------------------------
                        // SPECIMEN PREPARATION & COLLECTION:
                        // -------------------------------------------------------------------
                        // When specimen_preparing and specimen_cycling are active, adjust the scoring arm and collection.
                        if (specimen_preparing && specimen_cycling) {
                            if (!is_collected) {
                                if (timer_colection.seconds() > 0.2) {
                                    scoring.grip_transfer.setPosition(scoring.gripper_semi_hold);
                                    slidezz = slides.slides_specimen_high_score;
                                    scoring.score(scoring.scoring_arm_specimen_prepare);
                                    scoring.scoring_arm_extension.setPosition(scoring.extension_extended_specimen);
                                }
                                if (timer_colection.seconds() > 1.5) {
                                    scoring.grip_transfer.setPosition(scoring.gripper_hold);
                                    scoring.scoring_arm_extension.setPosition(scoring.extension_extended_specimen);
                                    specimen_preparing = false;
                                    specimen_collected = true;
                                }
                            }
                        }
                        // If the square button is pressed and collection is not in progress,
                        // trigger collection routines. Set intermediary flags and reset timers.
                        if (gamepad1.square && !colectng && !to_score) {
                            colectng = true;
                            colectng_intermediary = true;
                            timer2.reset();
                        }
                        if (colectng_intermediary) {
                            colectng_intermediary = false;
                            if (is_collecting && !is_collected) {
                                is_collecting = false;
                                colection.gripper.setPosition(colection.gripper_transfer);
                                colection_check = true;
                                timer_colection.reset();
                            } else if (!is_collecting_check && specimen_cycling) {
                                if (is_collected) {
                                    is_collected = false;
                                    colection.gripper.setPosition(colection.gripper_release);
                                    sample_to_observation = true;
                                    timer_colection.reset();
                                }
                                if (!manual) {
                                    if (scoring.senzor.getDistance(DistanceUnit.CM) < scoring.senzor_colected) {
                                        specimen_scoring = true;
                                        scoring.gripper(scoring.gripper_hold);
                                        specimen_preparing = true;
                                        timer_colection.reset();
                                        specimen_collected = true;
                                    }
                                } else {
                                    specimen_scoring = true;
                                    scoring.gripper(scoring.gripper_hold);
                                    specimen_preparing = true;
                                    timer_colection.reset();
                                    specimen_collected = true;
                                }
                            }
                        }
                        if (colectng && timer2.seconds() > 0.2) {
                            colectng = false;
                        }
                        // -------------------------------------------------------------------
                        // COLLECTION CHECK & TRANSFER DECISION:
                        // -------------------------------------------------------------------
                        // If colection_check is active, determine if a specimen is within collection range.
                        if (colection_check) {
                            if (specimen_cycling) {
                                if (manual) {
                                    if (timer_colection.seconds() > 0.1)
                                        colection.default_config();
                                    colection.default_config();
                                    colection.gripper_rotation.setPosition(colection.gripper_rotation_score_sample);
                                    is_collected = true;
                                    colection_check = false;
                                    is_extending = false;
                                } else {
                                    if (timer_colection.seconds() > 0.4) {
                                        if (colection.senzor.getDistance(DistanceUnit.CM) <= colection.distance_to_collected_sample) {
                                            if (automatization_intermediary) {
                                                automatization_running = true;
                                                automatization_intermediary = false;
                                            }
                                            colection.default_config();
                                            colection.gripper_rotation.setPosition(colection.gripper_rotation_score_sample);
                                            colection.gripper_angle.setPosition(colection.gripper_angle_vertical);
                                            is_collected = true;
                                            colection_check = false;
                                            is_extending = false;
                                        } else {
                                            colection.gripper_release();
                                            colection.gripper_height.setPosition(colection.height_default);
                                            colection_check = false;
                                        }
                                    }
                                }
                            } else {
                                if (manual) {
                                    if (timer_colection.seconds() > 0.4) {
                                        colection.scoring_config();
                                        extension.extend(extension.extension_transfer);
                                        if (!is_extending)
                                            transfer_retracted = true;
                                        else
                                            transfer_extend = true;
                                        is_collected = true;
                                        colection_check = false;
                                    }
                                } else {
                                    if (timer_colection.seconds() > 0.3) {
                                        if (colection.senzor.getDistance(DistanceUnit.CM) <= colection.distance_to_collected_sample) {
                                            colection.scoring_config();
                                            extension.extend(extension.extension_transfer);
                                            if (!is_extending)
                                                transfer_retracted = true;
                                            else
                                                transfer_extend = true;
                                            is_collected = true;
                                            colection_check = false;
                                        } else {
                                            colection.gripper_release();
                                            colection.gripper_height.setPosition(colection.height_default);
                                            colection_check = false;
                                        }
                                    }
                                }
                            }
                        }
                        // -------------------------------------------------------------------
                        // EXTENSION ADJUSTMENTS BASED ON TRANSFER FLAGS:
                        // -------------------------------------------------------------------
                        // If a specimen is collected and not cycling, extend or retract the mechanism based on transfer flags.
                        if (!specimen_cycling && is_collected) {
                            if (transfer_extend) {
                                colection.scoring_config();
                                extension.extend(extension.extension_transfer + 0.05);
                                timer.reset();
                                transfer_extend_counter = true;
                                transfer_extend = false;
                            }
                            if (transfer_retracted && is_collected) {
                                extension.extend(extension.extension_extended);
                                colection.scoring_config();
                                timer.reset();
                                transfer_retracted_counter = true;
                                transfer_retracted = false;
                            }
                        }
                        // After a short delay, reset transfer counters.
                        if ((transfer_retracted_counter && timer.seconds() > 0.4) ||
                                (transfer_extend_counter && timer.seconds() > 0.3)) {
                            transfer_extend_counter = false;
                            transfer_retracted_counter = false;
                            if (colection.senzor.getDistance(DistanceUnit.CM) < colection.distance_to_collected_sample) {
                                if (!blockage && slides.right_slide.getCurrentPosition() < 15) {
                                    colection.gripper_rotation.setPosition(colection.gripper_angle_default);
                                    colection.scoring_config();
                                    scoring.scoring_arm_default();
                                    extension.extend(extension.extension_transfer);
                                    blockage = true;
                                    timer.reset();
                                    basket_score = true;
                                }
                            } else {
                                is_collected = false;
                                is_extending = false;
                                scoring.score(scoring.scoring_arm_colect);
                                scoring.scoring_arm_extension.setPosition(scoring.extension_retracted);
                                colection.default_config();
                                colection.gripper_release();
                            }
                        }
                        // If a transition to observation is required, reset configuration after a delay.
                        if (sample_to_observation) {
                            if (timer_colection.seconds() > 0.3) {
                                colection.default_config();
                                sample_to_observation = false;
                            }
                        }
                        // -------------------------------------------------------------------
                        // BASKET SCORING & SPECIMEN SCORING TRIGGERS:
                        // -------------------------------------------------------------------
                        // When the circle button is pressed, trigger basket scoring routines.
                        if (gamepad1.circle) {
                            if (basket_score) {
                                scoring.gripper(scoring.gripper_release);
                                basket_score = false;
                                basket_reset = true;
                                timer.reset();
                                is_collected = false;
                                basket_scoring = false;
                            }
                            // If specimen scoring conditions are met, flag for scoring.
                            if (specimen_scoring && specimen_cycling && specimen_collected && !specimen_preparing) {
                                to_score = true;
                                timer_colection.reset();
                                scoring.scoring_arm_score_specimen_score();
                            }
                        }
                        // If basket reset is active, perform a reset after delays.
                        if (basket_reset) {
                            if (timer.seconds() > 1) {
                                scoring.score(scoring.scoring_arm_colect);
                                scoring.scoring_arm_extension.setPosition(scoring.extension_retracted);
                                slidezz = slides.slides_init;
                            }
                            if (timer.seconds() > 1.5) {
                                basket_reset = false;
                            }
                        }
                        // Transition to scoring state if flagged.
                        if (to_score) {
                            specimen_scoring = false;
                            if (timer_colection.seconds() > 0.4) {
                                scoring.gripper(scoring.gripper_release);
                            }
                            if (timer_colection.seconds() > 0.5) {
                                scoring.scoring_arm_extension.setPosition(scoring.extension_retracted);
                            }
                            if (timer_colection.seconds() > 0.6) {
                                slidezz = slides.slides_init;
                                scoring.scoring_arm_score_specimen_collect();
                                scoring.scoring_arm_extension.setPosition(scoring.extension_retracted);
                                to_score = false;
                            }
                        }
                    }
                    // -------------------------------------------------------------------
                    // HANDLING BLOCKAGE CONDITIONS:
                    // -------------------------------------------------------------------
                    // If blockage is detected, run a timed sequence to attempt recovery.
                    if (blockage) {
                        if (timer.seconds() > 0.1 && timer.seconds() < 0.2) {
                            scoring.grip_transfer.setPosition(scoring.gripper_hold);
                        }
                        if (timer.seconds() > 0.2 && timer.seconds() < 0.3) {
                            colection.gripper.setPosition(colection.gripper_release);
                        }
                        if (timer.seconds() > 0.4) {
                            colection.default_config();
                            extension.extend(extension.extension_retracted);
                            if (high_basket)
                                slidezz = slides.slides_high_basket;
                            else
                                slidezz = slides.slides_low_basket;
                        }
                        if (timer.seconds() > 0.8) {
                            scoring.scoring_arm_score_basket();
                            scoring.scoring_arm_extension.setPosition(scoring.extension_extended);
                            is_extending = false;
                            blockage = false;
                            basket_scoring = true;
                        }
                    }
                } else {
                    // -------------------------------------------------------------------
                    // HANDLING HANG MODE AND OTHER CONTROLS:
                    // -------------------------------------------------------------------
                    // When tohang is true, control the hanging mechanism and adjust slides accordingly.
                    if (timer.seconds() < 0.1) {
                        slidezz = 1000;
                        slides.culisante(slidezz);
                    }
                    if (timer.seconds() > 0.6 && timer.seconds() < 0.7) {
                        hanging.hang(hanging.hanged);
                    }
                    if (timer.seconds() > 0.75 && timer.seconds() < 0.8) {
                        hanging.hang(hanging.stop);
                        slidezz = 0;
                        slides.culisante(slidezz);
                    }
                    if (gamepad1.dpad_right)
                        tohang = false;
                    if (gamepad1.right_bumper) {
                        hanging.hang(hanging.hanged);
                        auto_hang = true;
                        timer_colection.reset();
                    }
                    if (timer_colection.seconds() < 1 && auto_hang) {
                        hanging.hang(hanging.hanged);
                    }
                    if (timer_colection.seconds() > 3.7 && auto_hang) {
                        hanging.hang(hanging.stop);
                        auto_hang = false;
                    }
                    if (!auto_hang && timer.seconds() > 1) {
                        hanging.hang(hanging.stop);
                    }
                }
                // -------------------------------------------------------------------
                // TELEMETRY UPDATE:
                // -------------------------------------------------------------------
                // Continuously display relevant sensor and state data for debugging.
                telemetry.addData("specimen_cyclng", specimen_cycling);
                telemetry.addData("culisante", slides.left_slide.getCurrentPosition());
                telemetry.addData("culisante2", slides.right_slide.getCurrentPosition());
                telemetry.addData("Using sensor?", !manual);
                telemetry.addData("senzor", colection.senzor.getDistance(DistanceUnit.CM));
                telemetry.addData("senzor2", scoring.senzor.getDistance(DistanceUnit.CM));
                telemetry.update();
            } else {
                // -------------------------------------------------------------------
                // AUTOMATIZATION OVERRIDE:
                // -------------------------------------------------------------------
                // If automation is running, the operator can override it using the right trigger.
                if (gamepad1.right_trigger != 0) {
                    automatization_running = false;
                    automatization_intermediary = false;
                }
                // If no specimen is collected, execute an automated scoring routine.
                if (!is_collected) {
                    if (automatization_running) {
                        Actions.runBlocking(
                                new SequentialAction(
                                        scoring_poz.build()
                                ));
                        colection.gripper_rotation.setPosition(colection.gripper_rotation_default);
                        scoring.scoring_arm_score_specimen_score();
                        extension.extend(extension.extension_extended);
                        sleep(300);
                        slidezz = 0;
                        slides.culisante(slidezz);
                        scoring.grip_transfer.setPosition(scoring.gripper_release);
                        specimen_collected = false;
                        colection.gripper_rotation.setPosition(colection.gripper_rotation_default);
                        is_extending = true;
                        automatization_running = false;
                        automatization_intermediary = true;
                        scoring.scoring_arm_extension.setPosition(scoring.extension_retracted);
                        sleep(100);
                        scoring.scoring_arm_score_specimen_collect();
                    }
                } else {
                    if (automatization_running) {
                        is_extending = false;
                        // Build a trajectory for pre-cycling specimen collection.
                        TrajectoryActionBuilder specimen_collect_pre_cicling = drive.actionBuilder(drive.pose)
                                .afterTime(0.2, scoring.gripper_release())
                                .afterTime(0.4, scoring.specimen_collect())
                                .afterTime(0.4, slides.slide_init())
                                .strafeToConstantHeading(new Vector2d(44, -61));
                        extension.extend(extension.extension_retracted);
                        Actions.runBlocking(
                                new SequentialAction(
                                        colection.rotation_observation(),
                                        specimen_collect_pre_cicling.build()
                                ));
                        colection.gripper_release();
                        scoring.gripper(scoring.gripper_semi_hold);
                        sleep(200);
                        is_collected = false;
                    }
                }
            }
            // Update the robot's pose estimate and telemetry at each loop iteration.
            drive.updatePoseEstimate();
            telemetry.update();
        }}}