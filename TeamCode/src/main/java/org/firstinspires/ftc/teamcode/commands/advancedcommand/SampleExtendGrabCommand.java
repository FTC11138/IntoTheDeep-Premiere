package org.firstinspires.ftc.teamcode.commands.advancedcommand;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.util.PIDFController;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.util.Constants;


public class SampleExtendGrabCommand extends CommandBase {

    private final Robot robot = Robot.getInstance();
    private PIDFController anglePID;

    @Override
    public void initialize() {
        assert robot.intakeSubsystem.armState == IntakeSubsystem.ArmState.INTAKE;
        robot.intakeSubsystem.setExtensionPower(1);
    }

    @Override
    public boolean isFinished() {
        return robot.sensorSubsystem.getIntakeSpeed() < Constants.samplePickupTurnSpeedTolerance || robot.intakeSubsystem.getExtensionPosition() >= Constants.extMax;
    }

    @Override
    public void end(boolean interrupted) {
        robot.intakeSubsystem.setExtensionPower(0);
    }

}
