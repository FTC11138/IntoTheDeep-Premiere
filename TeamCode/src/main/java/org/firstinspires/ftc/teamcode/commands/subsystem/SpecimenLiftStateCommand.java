package org.firstinspires.ftc.teamcode.commands.subsystem;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.SpecimenSubsystem;

public class SpecimenLiftStateCommand extends ConditionalCommand {
    public SpecimenLiftStateCommand(SpecimenSubsystem.SpecimenLiftState state) {
        super(
                new SequentialCommandGroup(
                        new SpecimenClawStateCommand(SpecimenSubsystem.SpecimenClawState.CLOSED),
                        new InstantCommand(() -> Robot.getInstance().specimenSubsystem.updateSpecimenLiftState(state)),
                        new WaitCommand(250),
                        new SpecimenClawStateCommand(SpecimenSubsystem.SpecimenClawState.OPEN)
                ),
                new InstantCommand(() -> {
                    Robot.getInstance().specimenSubsystem.updateSpecimenLiftState(state);
                }),
                () -> state == SpecimenSubsystem.SpecimenLiftState.DOWN || state == SpecimenSubsystem.SpecimenLiftState.GRAB
        );
    }
}
