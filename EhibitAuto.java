package org.firstinspires.ftc.teamcode.drive.code;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "Exhibit Auto")
public class EhibitAuto extends LinearOpMode {

    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d()).forward(36).build();
        Trajectory traj2 = drive.trajectoryBuilder(traj1.end()).strafeLeft(36).build();
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end()).back(36).build();
        Trajectory traj4 = drive.trajectoryBuilder(traj3.end()).strafeRight(36).build();

        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectory(traj1);
        drive.followTrajectory(traj2);
        drive.followTrajectory(traj3);
        drive.followTrajectory(traj4);
    }

}
