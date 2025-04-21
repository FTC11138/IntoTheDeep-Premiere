package org.firstinspires.ftc.teamcode.commands.advancedcommand;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.subsystem.ArmStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.ExtensionPositionCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.IntakePushStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.IntakeStateCommand;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.util.Constants;

public class IntakePullBackCommand extends SequentialCommandGroup {
    public IntakePullBackCommand() {
        super(
                new InstantCommand(Robot.getInstance().data::stopIntaking),
                new ArmStateCommand(IntakeSubsystem.ArmState.TRANSFER),
                new IntakePushStateCommand(IntakeSubsystem.IntakePushState.UP),
                new ExtensionPositionCommand(Constants.extMin),
                new WaitCommand((int) (Robot.getInstance().intakeSubsystem.getExtensionPosition() * 0.6)),
                new IntakeStateCommand(IntakeSubsystem.IntakeState.STOP)
        );
    }
}
