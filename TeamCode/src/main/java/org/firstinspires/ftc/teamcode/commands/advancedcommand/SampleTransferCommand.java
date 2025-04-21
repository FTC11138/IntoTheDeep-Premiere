package org.firstinspires.ftc.teamcode.commands.advancedcommand;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.subsystem.ArmStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.IntakePushStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.IntakeStateCommand;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.IntakeSubsystem;

public class SampleTransferCommand extends ConditionalCommand {

    public SampleTransferCommand() {
        super(
                new SequentialCommandGroup(

                        new ConditionalCommand(
                                new SequentialCommandGroup(
                                        new ArmStateCommand(IntakeSubsystem.ArmState.TRANSFER),
                                        new IntakePushStateCommand(IntakeSubsystem.IntakePushState.UP),
                                        new WaitCommand(50)
                                ),
                                new InstantCommand(),
                                () -> Robot.getInstance().intakeSubsystem.armState != IntakeSubsystem.ArmState.TRANSFER
                        ),
                        new InstantCommand(Robot.getInstance().data::setSampleLoaded),
                        new IntakeStateCommand(IntakeSubsystem.IntakeState.OUT),
                        new WaitCommand(600),
                        new ArmStateCommand(IntakeSubsystem.ArmState.UP),
                        new IntakePushStateCommand(IntakeSubsystem.IntakePushState.STORE),
                        new IntakeStateCommand(IntakeSubsystem.IntakeState.STOP)
                ),
                new InstantCommand(),
                () -> (
                        !Robot.getInstance().data.intaking && !Robot.getInstance().data.scoring
                )
        );
    }
}
