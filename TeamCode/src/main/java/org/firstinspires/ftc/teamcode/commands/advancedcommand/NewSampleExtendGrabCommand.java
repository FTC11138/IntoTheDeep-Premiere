/*package org.firstinspires.ftc.teamcode.commands.advancedcommand;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.util.PIDFController;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystems.NewIntakeSubsystem;
import org.firstinspires.ftc.teamcode.util.Constants;


public class NewSampleExtendGrabCommand extends CommandBase {

    private final Robot robot = Robot.getInstance();
    private PIDFController anglePID;

    @Override
    public void initialize() {
        assert robot.newIntakeSubsystem.armState == NewIntakeSubsystem.ArmState.INTAKE;
        robot.newIntakeSubsystem.setExtensionPower(1);
    }

    @Override
    public boolean isFinished() {
        return robot.sensorSubsystem.getIntakeSpeed() < Constants.samplePickupTurnSpeedTolerance || robot.newIntakeSubsystem.getExtensionPosition() >= Constants.extMax;
    }

    @Override
    public void end(boolean interrupted) {
        robot.newIntakeSubsystem.setExtensionPower(0);
    }

}*/
