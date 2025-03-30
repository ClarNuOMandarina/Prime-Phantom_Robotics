package org.firstinspires.ftc.teamcode.user;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * The 'colection' class manages the collection mechanism for the robot, which includes controlling the gripper,
 * its rotation, height, angle, and the light sensor for detecting collection events.
 * It also handles various configurations for collecting, transferring, and scoring mechanisms.
 */
public class colection {

    // Mechanisms and components of the collection system
    public DistanceSensor senzor;  // The distance sensor for detecting the proximity of a sample.
    public Servo gripper;  // The servo controlling the gripper (used to grab/release samples).
    public Servo light;  // Led used for better Limelight detection
    public Servo gripper_rotation;  // The servo controlling the rotation of the gripper.
    public Servo gripper_angle;  // The servo controlling the angle of the gripper.
    public Servo gripper_height;  // The servo controlling the vertical height of the gripper.

    // Constants for different gripper states and collection positions
    public double detection_light = 1;  // Light sensor , this value gives off white light.
    public double height_collecting = 0.94;  // Height for when the gripper is collecting.
    public double height_collecting_retracted = 0.925;  // Retracted height for collection.
    public double height_scanning = 0.8;  // Height for scanning operation with Limelight.
    public double height_default = 0.735;  // Default height for the gripper when not actively collecting.
    public double height_transfer = 0.385;  // Height for transferring collected samples.
    public double distance_to_collected_sample = 3;  // Distance to the sample when it is detected for collection.

    // Gripper positions for various actions
    public double gripper_hold = 0.88;  // Position for gripping (holding) an item.
    public double gripper_release = 0.65;  // Position for releasing the item.
    public double gripper_transfer = 0.86;  // Position for transferring an item.
    public double gripper_rotation_score = 0.42;  // Position for gripper rotation when scoring.
    public double gripper_rotation_default = 0.42;  // Default position for gripper rotation.
    public double gripper_rotation_collect = 0.42;  // Position for gripper rotation when collecting.
    public double gripper_rotation_score_sample = 0.8;  // Rotation position for scoring a sample.
    public double gripper_rotation_sample3 = 0.28;  // Rotation position for a specific sample type.

    // Gripper angle positions for different states
    public double gripper_angle_default = 0.52;  // Default angle of the gripper.
    double gripper_angle_tranfer = 0.52;  // Angle during the transfer process.
    public double gripper_angle_vertical = 0.25;  // Angle when the gripper is vertical.
    public double gripper_angle_sample_observation = 0.15;  // Angle for sample observation.

    /**
     * Constructor for initializing the hardware components from the hardwareMap.
     * @param hardwareMap The hardware map containing all robot hardware.
     */
    public colection(HardwareMap hardwareMap) {
        gripper = hardwareMap.get(Servo.class, "gripper");  // Initialize the gripper servo.
        gripper_height = hardwareMap.get(Servo.class, "gripper_height");  // Initialize the gripper height servo.
        gripper_angle = hardwareMap.get(Servo.class, "gripper_angle");  // Initialize the gripper angle servo.
        gripper_rotation = hardwareMap.get(Servo.class, "gripper_rotation");  // Initialize the gripper rotation servo.
        light = hardwareMap.get(Servo.class, "light");  // Initialize the Led.
        senzor = hardwareMap.get(DistanceSensor.class, "senzor");  // Initialize the distance sensor.
    }

    /**
     * Method to control the light detection mechanism, setting the light to the specified detection value.
     */
    public void detectio_light() {
        light.setPosition(detection_light);  // Set the led with maximum intensity.
    }

    /**
     * Method to set the gripper to the "grab" position to hold an object.
     */
    public void gripper_grab() {
        gripper.setPosition(gripper_hold);  // Move the gripper to the 'hold' position to grab an item.
    }

    /**
     * Method to set the gripper to the "release" position to release an object.
     */
    public void gripper_release() {
        gripper.setPosition(gripper_release);  // Move the gripper to the 'release' position.
    }

    /**
     * Method to configure the gripper for scoring action by setting its height, rotation, and angle.
     */
    public void scoring_config() {
        gripper_height.setPosition(height_transfer);  // Set the gripper height to transfer position.
        gripper_rotation.setPosition(gripper_rotation_score);  // Set the gripper rotation for scoring.
        gripper_angle.setPosition(gripper_angle_tranfer);  // Set the gripper angle for transferring.
    }

    /**
     * Method to configure the gripper for collecting action by setting its height, rotation, and angle.
     */
    public void collecting_config() {
        gripper_height.setPosition(height_collecting);  // Set the gripper height to collecting position.
        gripper_rotation.setPosition(gripper_rotation_collect);  // Set the gripper rotation for collecting.
        gripper_angle.setPosition(gripper_angle_default);  // Set the gripper angle to default for collecting.
    }

    /**
     * Method to initialize the gripper configuration to default values.
     */
    public void init_config() {
        gripper_height.setPosition(height_default);  // Set the gripper height to default.
        gripper.setPosition(gripper_release);  // Release the gripper.
        gripper_rotation.setPosition(gripper_rotation_score_sample);  // Set the gripper rotation for scoring sample.
        gripper_angle.setPosition(gripper_angle_tranfer);  // Set the gripper angle for transfer.
    }

    /**
     * Method to set the gripper to its default configuration.
     */
    public void default_config() {
        gripper_height.setPosition(height_default);  // Set height to default.
        gripper_rotation.setPosition(gripper_rotation_collect);  // Set rotation to collection position.
        gripper_angle.setPosition(gripper_angle_default);  // Set angle to default.
    }

    // Action classes that represent different behaviors for the gripper:

    /**
     * Action that releases the gripper to its 'release' position.
     */
    public class Gripper_release_max implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            gripper.setPosition(gripper_release);  // Release the gripper.
            return false;  // Action is complete.
        }
    }

    /**
     * Method to return an action that releases the gripper.
     */
    public Action griper_release() {
        return new Gripper_release_max();
    }

    /**
     * Action for handling the third sample collection by adjusting the gripper's angle and rotation.
     */
    public class third_Sample implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            gripper_angle.setPosition(0.72);  // Set the gripper angle for third sample.
            gripper_rotation.setPosition(gripper_rotation_sample3);  // Set the gripper rotation for third sample.
            return false;  // Action complete.
        }
    }

    /**
     * Method to return an action for collecting the third sample.
     */
    public Action third_sample() {
        return new third_Sample();
    }

    /**
     * Action to set the gripper rotation to its default position.
     */
    public class Rotation_default implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            gripper_rotation.setPosition(gripper_rotation_default);  // Set the rotation to default.
            return false;  // Action complete.
        }
    }

    /**
     * Method to return an action that sets the gripper rotation to its default position.
     */
    public Action rotation_default() {
        return new Rotation_default();
    }

    /**
     * Action for the gripper to enter observation mode by adjusting its angle and rotation.
     */
    public class Rotation_observation implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            gripper_angle.setPosition(gripper_angle_sample_observation);  // Set angle for observation.
            gripper_rotation.setPosition(gripper_rotation_score_sample);  // Set rotation for observation.
            return false;  // Action complete.
        }
    }

    /**
     * Method to return an action that sets the gripper to observation mode.
     */
    public Action rotation_observation() {
        return new Rotation_observation();
    }

    /**
     * Action to reset the gripper and return it to the default configuration.
     */
    public class Collecting_arm_default implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            gripper.setPosition(gripper_release);  // Release the gripper.
            gripper_angle.setPosition(gripper_angle_default);  // Set the angle to default.
            gripper_rotation.setPosition(gripper_rotation_collect);  // Set rotation to collecting position.
            gripper_height.setPosition(height_default);  // Set height to default.
            return false;  // Action complete.
        }
    }

    /**
     * Method to return an action that resets the collecting arm to default configuration.
     */
    public Action collecting_arm_default() {
        return new Collecting_arm_default();
    }
}
