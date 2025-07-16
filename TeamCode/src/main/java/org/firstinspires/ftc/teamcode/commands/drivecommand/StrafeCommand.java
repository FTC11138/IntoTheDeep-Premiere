package org.firstinspires.ftc.teamcode.commands.drivecommand;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.opmode.auto.AutonomousMethods;

public class StrafeCommand extends CommandBase {

    private final double inches;

    private final Robot robot = Robot.getInstance();

    private final double speed;

    public StrafeCommand(double inches) {




        this.inches = inches;
        this.speed = 1;
    }

    public StrafeCommand(double inches, double speed) {
        this.inches = inches;
        this.speed = speed;
    }

    @Override
    public void initialize() {

        Pose current = robot.follower.getPose();
        Pose target = robot.follower.getPose();
        target.setX(target.getX() + inches);

        PathChain path = new PathChain(AutonomousMethods.buildPath(current, target));

        robot.follower.setMaxPower(speed);
        robot.follower.followPath(path, true);
    }

    @Override
    public void execute() {
        robot.follower.update();
    }

    @Override
    public boolean isFinished() {
//        double xError = Math.abs(robot.getPose().getX() - path.getPath(0).getLastControlPoint().getX());
//        double yError = Math.abs(robot.getPose().getY() - path.getPath(0).getLastControlPoint().getY());
//        double headingError = Math.abs(robot.getPose().getHeading() - path.getPath(0).getHeadingGoal(1));

//        return xError < Constants.pathEndXTolerance && yError < Constants.pathEndYTolerance && headingError < Constants.pathEndHeadingTolerance;
        return !robot.follower.isBusy();
    }

}
