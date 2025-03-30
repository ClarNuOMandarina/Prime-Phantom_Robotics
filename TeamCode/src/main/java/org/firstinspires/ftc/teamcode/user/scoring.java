package org.firstinspires.ftc.teamcode.user;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * The scoring class manages all mechanisms related to scoring and specimen collection.
 * It controls two scoring arms (left and right), an extension mechanism for the arms,
 * and a grip transfer servo used to grab and release game elements.
 * Additionally, a distance sensor is used for verifying collection.
 */
public class scoring {
    // -----------------------------------------------------------------------
    // Servo Declarations (Hardware Components):
    // -----------------------------------------------------------------------
    // These servos control the movement of the scoring arms and their extension.
    public Servo scoring_arm_left;       // Left scoring arm servo.
    public Servo scoring_arm_right;      // Right scoring arm servo.
    public Servo scoring_arm_extension;  // Servo controlling the extension of the scoring arms.
    public Servo grip_transfer;          // Servo controlling the grip transfer mechanism.

    // -----------------------------------------------------------------------
    // Sensor Declarations:
    // -----------------------------------------------------------------------
    // A distance sensor (Rev v3 color sensor ) is used to verify if a specimen is collected.
    public DistanceSensor senzor;

    // -----------------------------------------------------------------------
    // Constants for Mechanism Positions:
    // -----------------------------------------------------------------------
    // These double constants store target positions for various states of the mechanisms.
    // They are used as setpoints for the servos.
    public double senzor_colected = 3;                      // Distance (in centimeters) that indicates a specimen is collected.
    public double extension_retracted = 0.61;               // Position for the extension mechanism when retracted.
    public double extension_specimen_collect = 0.47;        // Position for the extension during specimen collection.
    public double extension_extended = 0.25;                // Position for the extension when extended for basket scoring.
    public double extension_extended_specimen = 0.15;       // Extension position for specimen collection (extended state).
    public double extension_extended_max = 0.12;            // Maximum extended position for the extension during specimen scoring.

    public double gripper_hold = 0.57;                      // Servo position to hold an object.
    public double gripper_semi_hold = 0.53;                 // Servo position for a semi-hold state.
    public double gripper_release = 0.35;                   // Servo position to release an object.
    // Position values for the scoring arms (applied to both left and right servos).
    public double scoring_arm_colect = 0.21;                // Position for collecting a specimen.
    public double scoring_arm_default = 0.21;               // Default position for the scoring arms.
    public double scoring_arm_basket = 0.71;                // Position for basket scoring.
    public double scoring_arm_specimen_collect_first_cycle = 0.88; // Position for specimen collection in the first cycle.
    public double scoring_arm_specimen_score = 0.54;        // Position for scoring a specimen.
    public double scoring_arm_specimen_prepare = 0.44;      // Position to prepare the specimen before collection.

    public double scoring_arm_auto_end_init = 0.87;         // Position for auto end initialization of the scoring arm.
    public double scoring_arm_specimen_collect = 0.89;      // Position for collecting a specimen.
    public double scoring_arm_auto_init = 0;                // Initial auto position for scoring arms.
    public double scoring_arm_auto_park_basket = 0;         // Position for parking the basket during auto mode.

    // -----------------------------------------------------------------------
    // Constructor: Initialize Hardware Components
    // -----------------------------------------------------------------------
    public scoring(HardwareMap hardwareMap) {
        // Retrieve hardware from the hardwareMap based on the configured names.
        scoring_arm_left = hardwareMap.get(Servo.class, "scoring_arm_left");
        scoring_arm_right = hardwareMap.get(Servo.class, "scoring_arm_right");
        grip_transfer = hardwareMap.get(Servo.class, "grip_transfer");
        scoring_arm_extension = hardwareMap.get(Servo.class, "arm_extend");
        senzor = hardwareMap.get(DistanceSensor.class, "senzorsc");
    }

    // -----------------------------------------------------------------------
    // Basic Mechanism Control Methods:
    // -----------------------------------------------------------------------

    /**
     * Sets both scoring arms to the given position.
     *
     * @param x The target position (setpoint) for the scoring arms.
     */
    public void score(double x) {
        scoring_arm_left.setPosition(x);
        scoring_arm_right.setPosition(x);
    }

    /**
     * Sets both scoring arms to the collection position.
     */
    public void scoring_arm_colect() {
        scoring_arm_left.setPosition(scoring_arm_colect);
        scoring_arm_right.setPosition(scoring_arm_colect);
    }

    /**
     * Sets both scoring arms to the auto end initialization position.
     */
    public void scoring_arm_auto_init_end() {
        scoring_arm_left.setPosition(scoring_arm_auto_end_init);
        scoring_arm_right.setPosition(scoring_arm_auto_end_init);
    }

    /**
     * Prepares the scoring arms for specimen collection.
     * Also extends the scoring arm extension to the specimen collection extended position.
     */
    public void scoring_arm_specimen_prepare() {
        scoring_arm_left.setPosition(scoring_arm_specimen_prepare);
        scoring_arm_right.setPosition(scoring_arm_specimen_prepare);
        scoring_arm_extension.setPosition(extension_extended_specimen);
    }

    /**
     * Sets both scoring arms to the default position.
     */
    public void scoring_arm_default() {
        scoring_arm_left.setPosition(scoring_arm_default);
        scoring_arm_right.setPosition(scoring_arm_default);
    }

    /**
     * Moves both scoring arms to the basket scoring position.
     */
    public void scoring_arm_score_basket() {
        scoring_arm_left.setPosition(scoring_arm_basket);
        scoring_arm_right.setPosition(scoring_arm_basket);
    }

    /**
     * Moves both scoring arms to the specimen scoring position.
     */
    public void scoring_arm_score_specimen_score() {
        scoring_arm_left.setPosition(scoring_arm_specimen_score);
        scoring_arm_right.setPosition(scoring_arm_specimen_score);
    }

    /**
     * Sets the scoring arms to the specimen collection position and adjusts the extension.
     */
    public void scoring_arm_score_specimen_collect() {
        scoring_arm_left.setPosition(scoring_arm_specimen_collect);
        scoring_arm_right.setPosition(scoring_arm_specimen_collect);
        scoring_arm_extension.setPosition(extension_specimen_collect);
    }

    /**
     * Sets the grip transfer servo to a given position.
     *
     * @param x The target position for the grip transfer.
     */
    public void gripper(double x) {
        grip_transfer.setPosition(x);
    }

    /**
     * Sets the grip transfer servo to the "grab" position.
     */
    public void grip_transfer_grab() {
        grip_transfer.setPosition(gripper_hold);
    }

    /**
     * Sets the grip transfer servo to the "release" position.
     */
    public void grip_transfer_release() {
        grip_transfer.setPosition(gripper_release);
    }

    /**
     * Initializes the scoring subsystem to its default configuration.
     * This includes retracting the extension and setting arms and grip to collection positions.
     */
    public void init_config() {
        scoring_arm_extension.setPosition(extension_retracted);
        scoring_arm_left.setPosition(scoring_arm_colect);
        scoring_arm_right.setPosition(scoring_arm_colect);
        grip_transfer.setPosition(gripper_release);
    }

    // -----------------------------------------------------------------------
    // Inner Classes Implementing the Action Interface:
    // These classes allow the scoring subsystem to be used as actions within a
    // sequential or parallel action framework (e.g., Road Runner Actions).
    // Each inner class overrides the run() method and returns false when done,
    // indicating that the action does not need to be repeated.
    // -----------------------------------------------------------------------

    /**
     * Action to execute the gripper grab command.
     */
    public class Gripper_grab implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            grip_transfer_grab();
            return false; // Return false to indicate that the action is complete.
        }
    }

    public Action gripper_grab() {
        return new Gripper_grab();
    }

    /**
     * Alternative action for grabbing with maximum grip.
     */
    public class Gripper_grab2 implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            // This is similar to gripper_grab() but directly sets the servo position.
            grip_transfer.setPosition(gripper_hold);
            return false;
        }
    }

    public Action gripper_grab_max() {
        return new Gripper_grab2();
    }

    /**
     * Action to execute the gripper release command.
     */
    public class Gripper_release implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            grip_transfer.setPosition(gripper_release);
            return false;
        }
    }

    public Action gripper_release() {
        return new Gripper_release();
    }

    /**
     * Action to set the scoring arm extension for specimen collection.
     * Despite the name "Extension_retracted", this action sets the extension to the specimen collection position.
     */
    public class Extension_retracted implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            scoring_arm_extension.setPosition(extension_specimen_collect);
            return false;
        }
    }

    public Action extension_retracted() {
        return new Extension_retracted();
    }

    /**
     * Action to execute specimen collection by setting the arms to collection positions.
     */
    public class Specimen_collect implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            scoring_arm_score_specimen_collect();
            return false;
        }
    }

    public Action specimen_collect() {
        return new Specimen_collect();
    }

    /**
     * Action to prepare the scoring arm for specimen collection.
     */
    public class Specimen_prepare implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            scoring_arm_specimen_prepare();
            return false;
        }
    }

    public Action specimen_prepare() {
        return new Specimen_prepare();
    }

    /**
     * Action to reset the scoring mechanisms at the end of autonomous.
     * This action resets the scoring arms to auto-init end position, releases the grip, and retracts the extension.
     */
    public class AUTO_RESET implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            scoring_arm_auto_init_end();
            grip_transfer_release();
            scoring_arm_extension.setPosition(extension_retracted);
            return false;
        }
    }

    public Action auto_End() {
        return new scoring.AUTO_RESET();
    }

    /**
     * Action to score a specimen by moving the arms to the specimen scoring position.
     */
    public class Specimen_score implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            scoring_arm_score_specimen_score();
            return false;
        }
    }

    public Action specimen_score() {
        return new Specimen_score();
    }

    /**
     * Action to set the scoring arms to the collection position and retract the extension.
     */
    public class Sample_collect implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            scoring_arm_colect();
            scoring_arm_extension.setPosition(extension_retracted);
            return false;
        }
    }

    public Action sample_collect() {
        return new Sample_collect();
    }

    /**
     * Action to score a sample by moving the scoring arms to the basket scoring position
     * and extending the mechanism.
     */
    public class Sample_score implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            scoring_arm_score_basket();
            scoring_arm_extension.setPosition(extension_extended);
            return false;
        }
    }

    public Action sample_score() {
        return new Sample_score();
    }

    /**
     * Action to automatically score a sample using a maximum extension setting.
     */
    public class Sample_score_auto implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            scoring_arm_score_basket();
            scoring_arm_extension.setPosition(extension_extended_max);
            return false;
        }
    }

    public Action sample_score_auto() {
        return new Sample_score_auto();
    }

    /**
     * Action to score a sample while retracting the extension (useful for safe movement).
     */
    public class Sample_score_retracted implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            scoring_arm_score_basket();
            scoring_arm_extension.setPosition(extension_retracted);
            return false;
        }
    }

    public Action sample_score_retracted() {
        return new Sample_score_retracted();
    }

    /**
     * Action for the first cycle of specimen collection.
     * Sets the arms to specimen collection position and fully extends the mechanism to maximum.
     */
    public class speci_FIRST_collect implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            // Use the specimen_collect_first_cycle constant as the setpoint.
            score(scoring_arm_specimen_collect_first_cycle);
            scoring_arm_extension.setPosition(extension_extended_max);
            return false;
        }
    }

    public Action specimen_first_collect() {
        return new speci_FIRST_collect();
    }
}
