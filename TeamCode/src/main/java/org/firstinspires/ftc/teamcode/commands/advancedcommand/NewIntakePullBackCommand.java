package org.firstinspires.ftc.teamcode.commands.advancedcommand;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.subsystem.ArmStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.NewArmStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.ExtensionPositionCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.IntakePushStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.IntakeStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.NewIntakePushStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.RotateStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.WristStateCommand;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystems.NewIntakeSubsystem;
import org.firstinspires.ftc.teamcode.util.Constants;

public class NewIntakePullBackCommand extends SequentialCommandGroup {
    public NewIntakePullBackCommand() {
        super(
                new InstantCommand(Robot.getInstance().data::stopIntaking),
                new NewArmStateCommand(NewIntakeSubsystem.ArmState.TRANSFER),
                new WristStateCommand(NewIntakeSubsystem.WristState.TRANSFER),
                new RotateStateCommand(NewIntakeSubsystem.RotateState.VERTICAL),
                new ExtensionPositionCommand(Constants.extMin),
                new WaitCommand((int) (Robot.getInstance().intakeSubsystem.getExtensionPosition() * 0.6)),
                new RotateStateCommand(NewIntakeSubsystem.RotateState.HORIZONTAL)
        );
    }
}
