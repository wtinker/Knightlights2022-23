package org.firstinspires.ftc.teamcode.drive.code;

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

@TeleOp(name = "TestingTeleop")
//@Disabled
public class TestingTeleop extends LinearOpMode {

    DcMotor FrontLeft;
    DcMotor FrontRight;
    DcMotor BackLeft;
    DcMotor BackRight;
    DcMotor Lift;

    Servo claw;

    double power = 0.75;
    boolean turboMode = false;
    boolean mecanumdrive = true;
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
        FrontLeft = hardwareMap.dcMotor.get("Front Left");
        FrontRight = hardwareMap.dcMotor.get("Front Right");
        BackLeft = hardwareMap.dcMotor.get("Back Left");
        BackRight = hardwareMap.dcMotor.get("Back Right");
        Lift = hardwareMap.dcMotor.get("Lift");

        claw = hardwareMap.servo.get("claw");

        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        BackRight.setDirection(DcMotorSimple.Direction.REVERSE);
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
            if(mecanumdrive) {mecanumDrive();}
            else {tankDrive();}
        }

    }

    public void defaultMode() {
        Lift.setTargetPosition(LiftEncoderValue);
        Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Lift.setPower(1);

        telemetry.addData("field centric:", fieldcentric);
        telemetry.addData("mecanum mode:", mecanumdrive);
        telemetry.addData("lift target:", Lift.getTargetPosition());
        telemetry.addData("lift position:", Lift.getCurrentPosition());
        telemetry.update();

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
            mecanumdrive = !mecanumdrive;
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

    public void mecanumDrive() {
        if(fieldcentric){
            if(turboMode){
                double botHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS).secondAngle;
                double r = Math.hypot(-gamepad1.left_stick_x, gamepad1.left_stick_y);
                double θ = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4 + botHeading;
                double rot = -gamepad1.right_stick_x * 0.5;

                final double v1 = r * Math.cos(θ) + rot;
                final double v2 = r * Math.sin(θ) - rot;
                final double v3 = r * Math.sin(θ) + rot;
                final double v4 = r * Math.cos(θ) - rot;

                FrontLeft.setPower(v1);
                FrontRight.setPower(v2);
                BackLeft.setPower(v3);
                BackRight.setPower(v4);
            } else {
                double r = Math.hypot(-gamepad1.left_stick_x * power, gamepad1.left_stick_y * power);
                double θ = Math.atan2(gamepad1.left_stick_y * power, -gamepad1.left_stick_x * power) - Math.PI / 4;
                double rot = -gamepad1.right_stick_x * power * 0.5;

                final double v1 = r * Math.cos(θ) + rot;
                final double v2 = r * Math.sin(θ) - rot;
                final double v3 = r * Math.sin(θ) + rot;
                final double v4 = r * Math.cos(θ) - rot;

                FrontLeft.setPower(v1);
                FrontRight.setPower(v2);
                BackLeft.setPower(v3);
                BackRight.setPower(v4);
            }
        } else {
            if(turboMode){
                double r = Math.hypot(-gamepad1.left_stick_x, gamepad1.left_stick_y);
                double θ = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4;
                double rot = -gamepad1.right_stick_x * 0.5;

                final double v1 = r * Math.cos(θ) + rot;
                final double v2 = r * Math.sin(θ) - rot;
                final double v3 = r * Math.sin(θ) + rot;
                final double v4 = r * Math.cos(θ) - rot;

                FrontLeft.setPower(v1);
                FrontRight.setPower(v2);
                BackLeft.setPower(v3);
                BackRight.setPower(v4);
            } else {
                double r = Math.hypot(-gamepad1.left_stick_x * power, gamepad1.left_stick_y * power);
                double θ = Math.atan2(gamepad1.left_stick_y * power, -gamepad1.left_stick_x * power) - Math.PI / 4;
                double rot = -gamepad1.right_stick_x * power * 0.5;

                final double v1 = r * Math.cos(θ) + rot;
                final double v2 = r * Math.sin(θ) - rot;
                final double v3 = r * Math.sin(θ) + rot;
                final double v4 = r * Math.cos(θ) - rot;

                FrontLeft.setPower(v1);
                FrontRight.setPower(v2);
                BackLeft.setPower(v3);
                BackRight.setPower(v4);
            }
        }
        defaultMode();
    }

    public void tankDrive() {
        if (turboMode) {
            FrontLeft.setPower(gamepad1.left_stick_y);
            FrontRight.setPower(gamepad1.right_stick_y);
            BackLeft.setPower(gamepad1.left_stick_y);
            BackRight.setPower(gamepad1.right_stick_y);
        } else {
            FrontLeft.setPower(gamepad1.left_stick_y * power);
            FrontRight.setPower(gamepad1.right_stick_y * power);
            BackLeft.setPower(gamepad1.left_stick_y * power);
            BackRight.setPower(gamepad1.right_stick_y * power);
        }
        defaultMode();
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
