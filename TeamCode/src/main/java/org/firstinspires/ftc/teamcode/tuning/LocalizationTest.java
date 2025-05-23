package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.SparkFunOTOSDrive;
import org.firstinspires.ftc.teamcode.TankDrive;
import org.firstinspires.ftc.teamcode.user.colection;
import org.firstinspires.ftc.teamcode.user.extension;
import org.firstinspires.ftc.teamcode.user.scoring;
import org.firstinspires.ftc.teamcode.user.slides;
@Config
@TeleOp(name="localizer")
public class LocalizationTest extends LinearOpMode {
    public static int slidez=0;
    public static double extendz=0.75;
    public static double scoring_left_arm=0.2;
    public static double scoring_right_arm=0.1;
    public static double colecting_arms=0.62;
    public static double gripz=0.8;
    public static double transfer_gripz=0.62;
    public static double gripz_rotation=0.74;
    public static double gripz_angle=0.93;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        slides slides=new slides(hardwareMap);
        colection colection = new colection(hardwareMap);
        scoring scoring = new scoring(hardwareMap);
        extension extension = new extension(hardwareMap);
        boolean blockage=false;
        ElapsedTime timer =new ElapsedTime(0);
        boolean extend=false;
        PinpointDrive drive = new PinpointDrive(hardwareMap, new Pose2d(new Vector2d(-40,-58), Math.toRadians(90)));

        waitForStart();

        while (opModeIsActive()) {
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ),
                    -gamepad1.right_stick_x
            ));
            if(gamepad1.dpad_up)
            {
                slides.culisante(slidez);
            }
            if(gamepad1.dpad_left){
                extension.extend(extendz);
            }
            if(gamepad1.dpad_right){
//                scoring.score(scoring_left_arm,scoring_right_arm);
            }
            if(gamepad1.dpad_down){
//                colection.colection_arm(colecting_arms);

            }
            if(gamepad2.dpad_right)
            {
                colection.gripper.setPosition(gripz);
            }
            if(gamepad2.dpad_left)scoring.grip_transfer.setPosition(transfer_gripz);
            if(gamepad2.dpad_up)colection.gripper_rotation.setPosition(gripz_rotation);
            if(gamepad2.dpad_down)colection.gripper_angle.setPosition(gripz_angle);
            if(gamepad1.left_bumper)colection.collecting_config();
            if(gamepad1.right_bumper)colection.scoring_config();
            if(gamepad1.right_bumper)colection.scoring_config();
            if(gamepad1.right_trigger!=0) scoring.scoring_arm_colect();
            if(gamepad1.touchpad) {
                if ( blockage == false
                        && slides.right_slide.getCurrentPosition() < 15) {
                    colection.scoring_config();
                    scoring.scoring_arm_default();
                    blockage = true;
                    timer.reset();
                }
            }
            if(blockage==true) {
                if (timer.seconds() > 1 && timer.seconds() < 1.2) {
                    colection.gripper.setPosition(colection.gripper_transfer);
                    scoring.scoring_arm_colect();
                }

                if (timer.seconds() > 1.4 && timer.seconds() < 1.6) {
                    scoring.grip_transfer_grab();

                } if (timer.seconds() > 1.6 && timer.seconds() < 1.8) {
                    colection.gripper.setPosition(colection.gripper_release);
//                    colection.colection_arm(colection.colection_extended);

                }
                if (timer.seconds() > 2 && timer.seconds() < 2.2) {
                    colection.collecting_config();
                    blockage = false;
                }
            }
            if(gamepad1.triangle){
                scoring.init_config();
                colection.init_config();
            }

            if(gamepad1.share)colection.gripper_grab();

//            extension.extend_forced_cond(extend);
            telemetry.addData("CULI",slides.left_slide.getCurrentPosition());
            telemetry.addData("CULI2",slides.right_slide.getCurrentPosition());
            telemetry.addData("extend",extension.left_extension.getPosition());
            drive.updatePoseEstimate();

            telemetry.addData("x", drive.pose.position.x);
            telemetry.addData("y", drive.pose.position.y);
            telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), drive.pose);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
    }
}
