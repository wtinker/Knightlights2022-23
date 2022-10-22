package org.firstinspires.ftc.teamcode.drive.code;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name = "RRTestingTeleop")
//@Disabled
public class RRTestingTeleop extends LinearOpMode {

    DcMotor Lift;

    Servo claw;

    double power = 0.75;
    boolean fieldcentric = true;

    double leftTriggerStartTime = 0;
    double leftBumperStartTime = 0;
    double rightBumperStartTime = 0;
    double rightTriggerStartTime = 0;
    double bStartTime = 0;
    double yStartTime = 0;
    double aStartTime = 0;
    double xStartTime = 0;
    double upStartTime = 0;
    double downStartTime = 0;
    double leftStartTime = 0;
    double rightStartTime = 0;
    //Values for button cooldowns

    int LiftEncoderValue = 0;
    int ground = 0;
    int low = 333;
    int mid = 533;

    double open = 0.001;
    double closed = 0.25;

    boolean clawisclosed = true;

    BNO055IMU imu;

    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Lift = hardwareMap.dcMotor.get("Lift");

        claw = hardwareMap.servo.get("claw");

        Lift.setDirection(DcMotorSimple.Direction.REVERSE);

        Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        imu.initialize(parameters);

        claw.setPosition(closed);

        telemetry.addData("Ready to start", null);
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            drive.update();
            Pose2d myPose = drive.getPoseEstimate();
            if(fieldcentric){
                Vector2d input = new Vector2d(
                        -gamepad1.left_stick_y * power,
                        -gamepad1.left_stick_x * power
                ).rotated(-myPose.getHeading());
                drive.setWeightedDrivePower(new Pose2d(
                        input.getX(),
                        input.getY(),
                        -gamepad1.right_stick_x * power * 0.5));
            } else {
                Vector2d input = new Vector2d(
                        -gamepad1.left_stick_y * power,
                        -gamepad1.left_stick_x * power
                );
                drive.setWeightedDrivePower(new Pose2d(
                        input.getX(),
                        input.getY(),
                        -gamepad1.right_stick_x));
            }

            Lift.setTargetPosition(LiftEncoderValue);
            Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Lift.setPower(1);

            defaultMode();
            composeTelemetry();
        }

    }

    public void defaultMode() {
        //Other Controller Inputs
        if (gamepad1.right_bumper && rightBumperCooldown()) {
            clawisclosed = !clawisclosed;
            if(clawisclosed){claw.setPosition(closed);}
            if(!clawisclosed){claw.setPosition(open);}
        }

        if (gamepad1.left_bumper && leftBumperCooldown()) {
            fieldcentric = !fieldcentric;
        }

        if (gamepad1.left_trigger > 0.5 && leftTriggerCooldown()) {

        }

        if (gamepad1.right_trigger > 0.5 && rightTriggerCooldown()) {

        }

        if (gamepad1.b && bCooldown()) {

        }

        if (gamepad1.y && yCooldown()) {
            LiftEncoderValue = mid;
        }

        if (gamepad1.a && aCooldown()) {
            LiftEncoderValue = ground;
        }

        if(gamepad1.x && xCooldown()) {
            LiftEncoderValue = low;
        }

        if(gamepad1.dpad_up && upCooldown()) {
            LiftEncoderValue = LiftEncoderValue + 10;
        }

        if(gamepad1.dpad_down && downCooldown()) {
            LiftEncoderValue = LiftEncoderValue - 10;
        }

        if(gamepad1.dpad_left && leftCooldown()) {

        }

        if(gamepad1.dpad_right && rightCooldown()) {

        }

    }

    public void composeTelemetry(){
        telemetry.addData("field centric:", fieldcentric);
        telemetry.addData("lift target:", Lift.getTargetPosition());
        telemetry.addData("lift position:", Lift.getCurrentPosition());
        telemetry.update();
    }

    //Input Cooldowns
    public boolean leftTriggerCooldown() {
        if(getRuntime() - leftTriggerStartTime > 0.25) { //Must wait 250 milliseconds before input can be used again
            leftTriggerStartTime = getRuntime();
            return true;
        }
        return false;
    }

    public boolean leftBumperCooldown() {
        if(getRuntime() - leftBumperStartTime > 0.25) { //Must wait 250 milliseconds before input can be used again
            leftBumperStartTime = getRuntime();
            return true;
        }
        return false;
    }

    public boolean rightTriggerCooldown() {
        if(getRuntime() - rightTriggerStartTime > 0.25) { //Must wait 250 milliseconds before input can be used again
            rightTriggerStartTime = getRuntime();
            return true;
        }
        return false;
    }

    public boolean rightBumperCooldown() {
        if(getRuntime() - rightBumperStartTime > 0.25) { //Must wait 250 milliseconds before input can be used again
            rightBumperStartTime = getRuntime();
            return true;
        }
        return false;
    }

    public boolean xCooldown() {
        if(getRuntime() - xStartTime > .25) { //Must wait 250 milliseconds before input can be used again
            xStartTime = getRuntime();
            return true;
        }
        return false;
    }

    public boolean bCooldown() {
        if(getRuntime() - bStartTime > .25) { //Must wait 250 milliseconds before input can be used again
            bStartTime = getRuntime();
            return true;
        }
        return false;
    }

    public boolean yCooldown() {
        if(getRuntime() - yStartTime > .25) { //Must wait 250 milliseconds before input can be used again
            yStartTime = getRuntime();
            return true;
        }
        return false;
    }

    public boolean aCooldown() {
        if(getRuntime() - aStartTime > .25) { //Must wait 250 milliseconds before input can be used again
            aStartTime = getRuntime();
            return true;
        }
        return false;
    }

    public boolean upCooldown() {
        if(getRuntime() - upStartTime > .25) { //Must wait 250 milliseconds before input can be used again
            upStartTime = getRuntime();
            return true;
        }
        return false;
    }

    public boolean downCooldown() {
        if(getRuntime() - downStartTime > .25) { //Must wait 250 milliseconds before input can be used again
            downStartTime = getRuntime();
            return true;
        }
        return false;
    }

    public boolean leftCooldown() {
        if(getRuntime() - leftStartTime > .25) { //Must wait 250 milliseconds before input can be used again
            leftStartTime = getRuntime();
            return true;
        }
        return false;
    }

    public boolean rightCooldown() {
        if(getRuntime() - rightStartTime > .25) { //Must wait 250 milliseconds before input can be used again
            rightStartTime = getRuntime();
            return true;
        }
        return false;
    }

}
