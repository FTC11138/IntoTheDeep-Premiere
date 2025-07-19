package org.firstinspires.ftc.teamcode.commands.advancedcommand;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.subsystem.ClawStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.NewArmStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.RotateStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.WristStateCommand;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.NewIntakeSubsystem;

public class NewSampleTransferCommand extends ConditionalCommand {

    public NewSampleTransferCommand() {
        super(
                new SequentialCommandGroup(
//                        new NewIntakePullBackCommand(),
//                        new NewArmStateCommand(NewIntakeSubsystem.ArmState.FLAT),
                        new ConditionalCommand(
                                new SequentialCommandGroup(
                                        new WristStateCommand(NewIntakeSubsystem.WristState.TRANSFER),
                                        new WaitCommand(100),
                                        new RotateStateCommand(NewIntakeSubsystem.RotateState.VERTICAL),
                                        new WaitCommand(300),
                                        new NewArmStateCommand(NewIntakeSubsystem.ArmState.TRANSFER)

                                ),
                                new InstantCommand(),
                                () -> Robot.getInstance().newIntakeSubsystem.armState != NewIntakeSubsystem.ArmState.TRANSFER
                                    || Robot.getInstance().newIntakeSubsystem.wristState != NewIntakeSubsystem.WristState.TRANSFER
                        ),
                        new InstantCommand(Robot.getInstance().data::setSampleLoaded),
                        new ClawStateCommand(NewIntakeSubsystem.ClawState.OPEN),
                        new WaitCommand(200),
                        new NewArmStateCommand(NewIntakeSubsystem.ArmState.UP),
                        new WaitCommand(250),
                        new ClawStateCommand(NewIntakeSubsystem.ClawState.CLOSE),
                        new WristStateCommand(NewIntakeSubsystem.WristState.STORE),
                        new RotateStateCommand(NewIntakeSubsystem.RotateState.HORIZONTAL)
                ),
                new InstantCommand(),
                () -> (
                        !Robot.getInstance().data.intaking && !Robot.getInstance().data.scoring
                )
        );
    }
}
