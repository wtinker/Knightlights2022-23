package org.firstinspires.ftc.teamcode.drive.code.tournament;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name = "Tournament TeleOp - Red", group = "comp")
public class Teleop2 extends LinearOpMode {

    DcMotor Lift;

    Servo claw;

    double power = 0.5;
    boolean fieldcentric = true;

    Gamepad prevGamepad = new Gamepad();

    double yStartTime = 0;
    double aStartTime = 0;
    double dropStartTime = 0;
    double timer;
    //Values for button cooldowns

    int LiftEncoderValue = 0;
    int ground = 0;
    int low = 273;
    int mid = 533;

    double open = 0.001;
    double closed = 0.25;

    boolean clawisclosed = true;
    boolean lowering = false;
    boolean raising = false;

    int adjust;
    int tick;

    BNO055IMU imu;

    DistanceSensor juncdist;

    int midDist = 9;
    int lowDist = 15;
    int targDist;

    boolean scoreable;

    RevBlinkinLedDriver LED;

    RevBlinkinLedDriver.BlinkinPattern red;
    RevBlinkinLedDriver.BlinkinPattern blue;
    RevBlinkinLedDriver.BlinkinPattern green;
    RevBlinkinLedDriver.BlinkinPattern blueflash;
    RevBlinkinLedDriver.BlinkinPattern redflash;
    RevBlinkinLedDriver.BlinkinPattern end;
    RevBlinkinLedDriver.BlinkinPattern old;
    RevBlinkinLedDriver.BlinkinPattern current;

    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(PoseStorage.transferpose);

        Lift = hardwareMap.dcMotor.get("Lift");

        claw = hardwareMap.servo.get("claw");

        juncdist = hardwareMap.get(DistanceSensor.class, "dist");

        LED = hardwareMap.get(RevBlinkinLedDriver.class, "LED");

        red = RevBlinkinLedDriver.BlinkinPattern.RED;
        blue = RevBlinkinLedDriver.BlinkinPattern.BLUE;
        green = RevBlinkinLedDriver.BlinkinPattern.GREEN;
        blueflash = RevBlinkinLedDriver.BlinkinPattern.WHITE;
        redflash = RevBlinkinLedDriver.BlinkinPattern.WHITE;
        end = RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_WHITE;

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

        timer = getRuntime();

        while (opModeIsActive()) {
            drive.update();
            Pose2d myPose = drive.getPoseEstimate();
            if(fieldcentric){
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

            if(tick > 0 && dropStartTime - getRuntime() > 0.1){
                LiftEncoderValue = LiftEncoderValue + adjust;
                dropStartTime = getRuntime();
                tick = tick - 1;
            }

            Lift.setTargetPosition(LiftEncoderValue);
            Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Lift.setPower(0.5);

            if(Math.abs(juncdist.getDistance(DistanceUnit.INCH) - targDist) < 1){scoreable = true;}
            else{scoreable = false;}

            runColors();
            defaultMode();
            composeTelemetry();
            try {
                prevGamepad.copy(gamepad1);
            } catch (RobotCoreException e) {

            }
        }

    }

    public void defaultMode() {
        //Other Controller Inputs
        if (gamepad1.right_bumper && !prevGamepad.right_bumper) {
            clawisclosed = !clawisclosed;
            if(clawisclosed){claw.setPosition(closed);}
            if(!clawisclosed){claw.setPosition(open);}
        }

        if (gamepad1.left_bumper && !prevGamepad.left_bumper) {
            fieldcentric = !fieldcentric;
        }

        if (gamepad1.left_trigger > 0.5) {

        }

        if (gamepad1.right_trigger > 0.5) {

        }

        if (gamepad1.b && !prevGamepad.b) {

        }

        if (gamepad1.y && yCooldown()) {
            moveArm(Lift.getCurrentPosition(), mid, 5);
            targDist = midDist;
        }

        if (gamepad1.a && aCooldown()) {
            moveArm(Lift.getCurrentPosition(), ground, 5);
        }

        if(gamepad1.x && !prevGamepad.x) {
            moveArm(Lift.getCurrentPosition(), low, 5);
            targDist = lowDist;
        }

        if(gamepad1.dpad_up && !prevGamepad.dpad_up) {
            LiftEncoderValue = LiftEncoderValue + 25;
        }

        if(gamepad1.dpad_down && !prevGamepad.dpad_down) {
            LiftEncoderValue = LiftEncoderValue - 25;
        }

        if(gamepad1.dpad_left && !prevGamepad.dpad_left) {

        }

        if(gamepad1.dpad_right && !prevGamepad.dpad_right) {

        }

    }

    public void moveArm(int current, int target, int change){
        adjust = (int) ((current - target) / change);
        dropStartTime = getRuntime();
        tick = change;
    }

    public void composeTelemetry(){
        telemetry.addData("field centric:", fieldcentric);
        telemetry.addData("lift target:", Lift.getTargetPosition());
        telemetry.addData("lift position:", Lift.getCurrentPosition());
        telemetry.addData("Junction Distance:", juncdist.getDistance(DistanceUnit.INCH));
        telemetry.update();
    }

    public void runColors(){
        old = current;
        if(scoreable){current = green;}
        else{
            if(getRuntime() - timer > 115){current = end;}
            else{
                if(getRuntime() - timer > 90){current = redflash;}
                else{
                    current = red;}}
        }
        LED.setPattern(current);
    }

    //Input Cooldowns
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

}
