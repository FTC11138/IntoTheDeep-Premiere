package org.firstinspires.ftc.teamcode.commands.advancedcommand;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.subsystem.ArmStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.ClawStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.IntakePushStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.NewArmStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.NewIntakePushStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.WristStateCommand;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystems.NewIntakeSubsystem;

public class NewSampleTransferCommand extends ConditionalCommand {

    public NewSampleTransferCommand() {
        super(
                new SequentialCommandGroup(

                        new ConditionalCommand(
                                new SequentialCommandGroup(
                                        new NewArmStateCommand(NewIntakeSubsystem.ArmState.TRANSFER),
                                        new NewIntakePushStateCommand(NewIntakeSubsystem.IntakePushState.UP),
                                        new WaitCommand(50),
                                        new WristStateCommand(NewIntakeSubsystem.WristState.TRANSFER),
                                        new WaitCommand(50)
                                ),
                                new InstantCommand(),
                                () -> Robot.getInstance().newIntakeSubsystem.armState != NewIntakeSubsystem.ArmState.TRANSFER
                        ),
                        new InstantCommand(Robot.getInstance().data::setSampleLoaded),
                        new ClawStateCommand(NewIntakeSubsystem.ClawState.OPEN),
                        new WaitCommand(600),
                        new NewArmStateCommand(NewIntakeSubsystem.ArmState.UP),
                        new NewIntakePushStateCommand(NewIntakeSubsystem.IntakePushState.STORE),
                        new ClawStateCommand(NewIntakeSubsystem.ClawState.CLOSE)
                ),
                new InstantCommand(),
                () -> (
                        !Robot.getInstance().data.intaking && !Robot.getInstance().data.scoring
                )
        );
    }
}
