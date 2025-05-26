package org.firstinspires.ftc.teamcode.commands.advancedcommand;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.util.CustomPIDFCoefficients;
import com.pedropathing.util.PIDFController;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.SensorSubsystem;
import org.firstinspires.ftc.teamcode.util.Constants;


public class NewSampleAlignCommand extends CommandBase {

    private final Robot robot = Robot.getInstance();
    private PIDFController anglePID;

    private double angle;

    @Override
    public void initialize() {
        anglePID = new PIDFController(new CustomPIDFCoefficients(Constants.kPAngle, Constants.kIAngle, Constants.kDAngle, Constants.kFAngle));
        robot.follower.startTeleopDrive();
        robot.sensorSubsystem.updateCameraState(SensorSubsystem.CameraState.ON);
    }

    @Override
    public void execute() {
        angle = robot.sensorSubsystem.getCameraAngleSample();

        robot.follower.update();

        anglePID.updatePosition(0);
        anglePID.setTargetPosition(angle);

        robot.follower.setTeleOpMovementVectors(0, 0, -anglePID.runPIDF(), true);
    }

    @Override
    public boolean isFinished() {
        return angle < Constants.sampleAlignAngleTolerance;
    }

    @Override
    public void end(boolean interrupted) {
        robot.follower.setTeleOpMovementVectors(0, 0, 0, true);
        robot.sensorSubsystem.updateCameraState(SensorSubsystem.CameraState.OFF);
    }

}
