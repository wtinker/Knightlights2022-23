package org.firstinspires.ftc.teamcode.drive.code.tournament;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name = "Demo TeleOp", group = "comp")
public class Teleop3 extends LinearOpMode {

    DcMotor Lift;

    Servo claw;

    double power = 0.75;
    boolean fieldcentric = true;
    boolean driving = false;

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
    double dropStartTime = 0;
    //Values for button cooldowns

    int LiftEncoderValue = 0;
    int ground = 0;
    int low = 333;
    int mid = 533;

    double open = 0.001;
    double closed = 0.25;

    boolean clawisclosed = true;
    boolean lowering = false;

    BNO055IMU imu;

    DistanceSensor juncdist;

    int junctiondistance;

    RevBlinkinLedDriver LED;

    RevBlinkinLedDriver.BlinkinPattern red;
    RevBlinkinLedDriver.BlinkinPattern blue;

    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(PoseStorage.transferpose);

        Lift = hardwareMap.dcMotor.get("Lift");

        claw = hardwareMap.servo.get("claw");

        juncdist = hardwareMap.get(DistanceSensor.class, "dist");

        LED = hardwareMap.get(RevBlinkinLedDriver.class, "LED");

        red = RevBlinkinLedDriver.BlinkinPattern.RED;
        blue = RevBlinkinLedDriver.BlinkinPattern.BLUE;

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
            if(driving) {
                drive.update();
                Pose2d myPose = drive.getPoseEstimate();
                if (fieldcentric) {
                    Vector2d input = new Vector2d(
                            -gamepad1.left_stick_y * power,
                            -gamepad1.left_stick_x * power
                    ).rotated(-myPose.getHeading() - Math.toRadians(90));
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
                            -gamepad1.right_stick_x * power * 0.5));
                }
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
            driving = !driving;
        }

        if (gamepad1.y && yCooldown()) {
            LiftEncoderValue = mid;
            lowering = false;
        }

        if (gamepad1.a && aCooldown()) {
            lowering = true;
            dropStartTime = getRuntime();
        }

        if(gamepad1.x && xCooldown()) {
            LiftEncoderValue = low;
            lowering = false;
        }

        if(gamepad1.dpad_up && upCooldown()) {
            LiftEncoderValue = LiftEncoderValue + 10;
        }

        if(gamepad1.dpad_down && downCooldown()) {
            LiftEncoderValue = LiftEncoderValue - 10;
        }

        if(gamepad1.dpad_left && leftCooldown()) {
            LED.setPattern(blue);
        }

        if(gamepad1.dpad_right && rightCooldown()) {
            LED.setPattern(red);
        }

        if(lowering){
            if(getRuntime() - dropStartTime > 0.1 && LiftEncoderValue > ground){LiftEncoderValue = LiftEncoderValue - 20;}
            if(Math.abs(LiftEncoderValue - ground) < 20){LiftEncoderValue = ground; lowering = false;}
        }

    }

    public void composeTelemetry(){
        telemetry.addData("driving:", driving);
        telemetry.addData("field centric:", fieldcentric);
        telemetry.addData("lift target:", Lift.getTargetPosition());
        telemetry.addData("lift position:", Lift.getCurrentPosition());
        telemetry.addData("Junction Distance:", juncdist.getDistance(DistanceUnit.INCH));
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
