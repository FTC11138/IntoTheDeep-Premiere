package org.firstinspires.ftc.teamcode.commands.advancedcommand;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.subsystem.ArmStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.ClawStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.ExtensionPositionCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.IntakePushStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.IntakeStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.NewArmStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.NewIntakePushStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.RotateStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.WristStateCommand;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystems.NewIntakeSubsystem;

public class NewIntakePushOutCommand extends SequentialCommandGroup {
    public NewIntakePushOutCommand(int ext) {
        super(
                new ExtensionPositionCommand(ext),
                new ClawStateCommand(NewIntakeSubsystem.ClawState.OPEN),
//                new WaitCommand(5000),
                new RotateStateCommand(NewIntakeSubsystem.RotateState.HORIZONTAL),
                new NewArmStateCommand(NewIntakeSubsystem.ArmState.FLAT),
                new WristStateCommand(NewIntakeSubsystem.WristState.GRAB),
//                new WaitCommand(1000),
//                new ClawStateCommand(NewIntakeSubsystem.ClawState.CLOSE),
//                new WaitCommand(500),

                new InstantCommand(Robot.getInstance().data::startIntaking)
        );
    }
}
