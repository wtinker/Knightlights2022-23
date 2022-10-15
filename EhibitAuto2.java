package org.firstinspires.ftc.teamcode.drive.code;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "Exhibit Auto 2")
public class EhibitAuto2 extends LinearOpMode {

    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d()).lineToConstantHeading(new Vector2d(18, -18)).build();
        Trajectory traj2 = drive.trajectoryBuilder(traj1.end()).splineTo(new Vector2d(36, 0), Math.toRadians(90)).build();
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end()).lineToSplineHeading(new Pose2d(18, 18, Math.toRadians(180) + 1e-6)).build();
        Trajectory traj4 = drive.trajectoryBuilder(traj3.end()).splineToConstantHeading(new Vector2d(0, 0), 0).build();
        TrajectorySequence turn = drive.trajectorySequenceBuilder(traj4.end()).turn(Math.toRadians(180) - 1e-6).build();

        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectory(traj1);
        drive.followTrajectory(traj2);
        drive.followTrajectory(traj3);
        drive.followTrajectory(traj4);
        drive.followTrajectorySequence(turn);
    }

}
