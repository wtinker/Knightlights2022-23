package org.firstinspires.ftc.teamcode.drive.code;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name = "Exhibit Teleop - Bot")
public class ExhibitTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        double speed = 0.75;
        boolean turbomode = true;

        waitForStart();

        while(opModeIsActive()) {
            drive.update();

            Pose2d myPose = drive.getPoseEstimate();

            Vector2d input = new Vector2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x);


            if(turbomode){
                drive.setWeightedDrivePower(new Pose2d(
                        input.getX(),
                        input.getY(),
                        -gamepad1.right_stick_x));
            } else{
                drive.setWeightedDrivePower(new Pose2d(
                        input.getX() * speed,
                        input.getY() * speed,
                        -gamepad1.right_stick_x));
            }

            if(gamepad1.a){
                turbomode = false;
            }
            if(gamepad1.b){
                turbomode = true;
            }
            telemetry.addData("turbomode:",turbomode);
            telemetry.update();
        }
    }

}
