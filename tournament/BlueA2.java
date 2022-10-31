package org.firstinspires.ftc.teamcode.drive.code.tournament;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

@Autonomous(name = "Blue A2", group = "comp")
public class BlueA2 extends LinearOpMode{

    Pose2d startpose = new Pose2d(-31.5, 68, Math.toRadians(270));
    boolean zone1 = false;
    boolean zone2 = false;
    boolean zone3 = false;

    int bb = 237, br = 107,bg = 201;
    int rb = 175, rr = 177, rg = 218;
    int gb = 212, gr = 130, gg = 272;
    int error = 50;

    int ground = 0;
    int low = 333;
    int mid = 518;

    double open = 0.001;
    double closed = 0.25;

    DcMotor Lift;

    Servo claw;

    ColorSensor color;

    @Override
    public void runOpMode() {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        color = hardwareMap.colorSensor.get("color");

        Lift = hardwareMap.dcMotor.get("Lift");

        claw = hardwareMap.servo.get("claw");

        Lift.setDirection(DcMotorSimple.Direction.REVERSE);

        Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        claw.setPosition(closed);

        drive.setPoseEstimate(startpose);

        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(startpose)
                .setVelConstraint(new TranslationalVelocityConstraint(24))
                .setTurnConstraint(Math.toRadians(90), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-33, 41), Math.toRadians(270))
                .waitSeconds(1)
                .addDisplacementMarker(() -> {
                    //scan code here
                    telemetry.addData("Blue:", color.blue());
                    telemetry.addData("Red:", color.red());
                    telemetry.addData("Green:", color.green());
                    if(Math.abs(br - color.red()) < error && Math.abs(bg - color.green()) < error && Math.abs(bb - color.blue()) < error){telemetry.addData("Detecting blue", null); zone1 = true;}
                    if(Math.abs(rr - color.red()) < error && Math.abs(rg - color.green()) < error && Math.abs(rb - color.blue()) < error){telemetry.addData("Detecting red", null); zone2 = true;}
                    if(Math.abs(gr - color.red()) < error && Math.abs(gg - color.green()) < error && Math.abs(gb - color.blue()) < error){telemetry.addData("Detecting green", null); zone3 = true;}
                    telemetry.update();
                })
                .splineToConstantHeading(new Vector2d(-36, 36), Math.toRadians(270))
                .turn(Math.toRadians(45))
                .forward(11)
                .turn(Math.toRadians(15))
                .addDisplacementMarker(() -> {
                    //cone scoring
                    claw.setPosition(open);
                    sleep(500);
                })
                .turn(Math.toRadians(-15))
                .back(11)
                .build();

        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(traj1.end())
                .turn(Math.toRadians(45))
                .forward(24)
                .turn(Math.toRadians(90))
                .build();

        TrajectorySequence traj3 = drive.trajectorySequenceBuilder(traj1.end())
                .turn(Math.toRadians(135))
                .build();

        TrajectorySequence traj4 = drive.trajectorySequenceBuilder(traj1.end())
                .turn(Math.toRadians(-135))
                .forward(24)
                .turn(Math.toRadians(-90))
                .build();

        telemetry.addData("Ready to start", null);
        telemetry.update();

        waitForStart();

        Lift.setTargetPosition(mid);
        Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Lift.setPower(0.75);

        sleep(1000);

        drive.followTrajectorySequence(traj1);

        sleep (500);

        if(zone1){drive.followTrajectorySequence(traj2);}
        else if(zone2){drive.followTrajectorySequence(traj3);}
        else if(zone3){drive.followTrajectorySequence(traj4);}

        Lift.setTargetPosition(ground);
        Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(500);

        PoseStorage.transferpose = drive.getPoseEstimate();

    }

}
