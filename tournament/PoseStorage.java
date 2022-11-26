package org.firstinspires.ftc.teamcode.drive.code.tournament;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;

public class PoseStorage {
    public static Pose2d transferpose = new Pose2d();
    public static int bb = 153, br = 73,bg = 137;
    public static int rb = 118, rr = 154, rg = 168;
    public static int gb = 160, gr = 109, gg = 232;
    public static int error = 40;
    public static TranslationalVelocityConstraint speed = new TranslationalVelocityConstraint(10);

}
