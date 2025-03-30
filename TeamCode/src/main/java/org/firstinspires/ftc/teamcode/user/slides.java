package org.firstinspires.ftc.teamcode.user;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

// The slides class handles the control of the robot's slide mechanisms
public class slides {

    // Motor declarations for controlling the slide mechanism
    public DcMotorEx left_slide, right_slide;

    // Positions for various slide actions
    public int slides_init = 0; // Initial position of the slides
    public int slides_low_basket = 525; // Position for low basket (where the robot places items)
    public int slides_high_basket = 1050; // Position for high basket (higher placing position)
    public int slides_specimen_high = 60; // Position for scoring a high specimen
    public int slides_specimen_high_score = 60; // Another position for specimen scoring
    public int slide_auto_parking_new = 0; // Position for auto-parking action (to place robot after task)

    // Constructor: Initialize the hardware components (motors)
    public slides(HardwareMap hardwareMap) {
        // Declare motors and set their directions
        left_slide = hardwareMap.get(DcMotorEx.class, "left_slide");
        right_slide = hardwareMap.get(DcMotorEx.class, "right_slide");

        // Set the direction of the right slide motor to reverse (opposite direction of left slide)
        right_slide.setDirection(DcMotorSimple.Direction.REVERSE);
        left_slide.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set the motors to run using the encoder (for precise movement control)
        left_slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // Method to reset the encoders of both slide motors
    public void reset_encoder() {
        left_slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the left slide motor's encoder
        right_slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the right slide motor's encoder
    }

    // General function for controlling the slide motors to move to a target position
    public void culisante(int x) {
        // Set target position for both motors
        left_slide.setTargetPosition(x);
        right_slide.setTargetPosition(x);

        // Set the motors to run to the target position
        left_slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set the motors to full power for moving the slides
        left_slide.setPower(1);
        right_slide.setPower(1);
    }

    // Action for setting the slides to their initial position (used in autonomous)
    public class Slide_init implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            // Call the general function to set slides to initial position
            culisante(slides_init);
            return false; // Return false to indicate the action is complete
        }
    }

    // Returns a new Slide_init action object
    public Action slide_init() {
        return new Slide_init();
    }

    // Action to move the slides to a high specimen score position
    public class Slide_specimen_score implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            // Move the slides to the high specimen scoring position
            culisante(slides_specimen_high);
            return false; // Action is complete
        }
    }

    // Returns a new Slide_specimen_score action object
    public Action specimen_score_high() {
        return new Slide_specimen_score();
    }

    // Another action to move slides to the high specimen scoring position (similar to the previous one)
    public class Slide_specimen_score2 implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            // Move the slides to the high specimen scoring position
            culisante(slides_specimen_high);
            return false; // Action is complete
        }
    }

    // Returns a new Slide_specimen_score2 action object
    public Action specimen_score_high2() {
        return new Slide_specimen_score2();
    }

    // Action to move slides to the position for sample placement in the high basket
    public class Slide_sample implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            // Move slides to the high basket placement position
            culisante(slides_high_basket);
            return false; // Action is complete
        }
    }

    // Returns a new Slide_sample action object
    public Action slide_sample() {
        return new Slide_sample();
    }

    // Action to move the slides to the auto-parking position for the basket
    public class Auto_park_basket implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            // Move slides to the auto parking position (new)
            culisante(slide_auto_parking_new);
            return false; // Action is complete
        }
    }

    // Returns a new Auto_park_basket action object
    public Action auto_park_basket() {
        return new Auto_park_basket();
    }
}
