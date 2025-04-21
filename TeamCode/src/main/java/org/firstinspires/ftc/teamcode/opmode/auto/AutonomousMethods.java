package org.firstinspires.ftc.teamcode.opmode.auto;

import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;

public class AutonomousMethods {

    public static Path buildPath(Pose pose1, Pose pose2, int time) {
        Point point1 = new Point(pose1.getX(), pose1.getY());
        Point point2 = new Point(pose2.getX(), pose2.getY());

        Path path = new Path(new BezierLine(point1, point2));

        path.setLinearHeadingInterpolation(pose1.getHeading(), pose2.getHeading(), time);

        return path;
    }

    public static Path buildPath(Pose pose1, Pose pose2) {
        return buildPath(pose1, pose2, 1);
    }

    public static Path buildCurve(Pose pose1, Pose pose2, Point control1, double time) {
        Point point1 = new Point(pose1.getX(), pose1.getY());
        Point point2 = new Point(pose2.getX(), pose2.getY());

        Path path = new Path(new BezierCurve(point1, control1, point2));

        path.setLinearHeadingInterpolation(pose1.getHeading(), pose2.getHeading(), time);

        return path;
    }

    public static Path buildCurve(Pose pose1, Pose pose2, Point control1) {
        return buildCurve(pose1, pose2, control1, 1);
    }

    public static Path buildCurve(Pose pose1, Pose pose2, Point control1, Point control2, double time) {
        Point point1 = new Point(pose1.getX(), pose1.getY());
        Point point2 = new Point(pose2.getX(), pose2.getY());

        Path path = new Path(new BezierCurve(point1, control1, control2, point2));

        path.setLinearHeadingInterpolation(pose1.getHeading(), pose2.getHeading(), time);

        return path;
    }

    public static Path buildCurve(Pose pose1, Pose pose2, Point control1, Point control2) {
        return buildCurve(pose1, pose2, control1, control2, 1);
    }

}
