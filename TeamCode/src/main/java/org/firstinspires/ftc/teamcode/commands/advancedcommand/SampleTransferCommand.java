package org.firstinspires.ftc.teamcode.commands.advancedcommand;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.subsystem.ArmStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.ClawStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.IntakePushStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.IntakeStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.RotateStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.WristStateCommand;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystems.NewIntakeSubsystem;

public class SampleTransferCommand extends ConditionalCommand {

    public SampleTransferCommand() {
        super(
                new SequentialCommandGroup(

                        new ConditionalCommand(
                                new SequentialCommandGroup(
                                        new ArmStateCommand(NewIntakeSubsystem.ArmState.TRANSFER),
//                                        new IntakePushStateCommand(IntakeSubsystem.IntakePushState.UP),
                                        new WaitCommand(50)
                                ),
                                new InstantCommand(),
                                () -> Robot.getInstance().newIntakeSubsystem.armState != NewIntakeSubsystem.ArmState.TRANSFER
                        ),
                        new InstantCommand(Robot.getInstance().data::setSampleLoaded),
                        new RotateStateCommand(NewIntakeSubsystem.RotateState.VERTICAL),
                        new WaitCommand(600),
                        new WristStateCommand(NewIntakeSubsystem.WristState.TRANSFER),
                        new ArmStateCommand(NewIntakeSubsystem.ArmState.TRANSFER),
                        new WristStateCommand(NewIntakeSubsystem.WristState.STORE),
                        new WaitCommand(600),
                        new ClawStateCommand(NewIntakeSubsystem.ClawState.OPEN)


//                        new IntakeStateCommand(IntakeSubsystem.IntakeState.STOP)
                ),
                new InstantCommand(),
                () -> (
                        !Robot.getInstance().data.intaking && !Robot.getInstance().data.scoring
                )
        );
    }
}
