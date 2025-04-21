package org.firstinspires.ftc.teamcode.commands.advancedcommand;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.subsystem.SpecimenClawStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.SpecimenLiftStateCommand;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.SpecimenSubsystem;

public class SpecimenGrabCommand extends ConditionalCommand {
    public SpecimenGrabCommand() {
        super(
                new SequentialCommandGroup(
                    new SpecimenClawStateCommand(SpecimenSubsystem.SpecimenClawState.CLOSED),
                    new WaitCommand(200),
                    new SpecimenLiftStateCommand(SpecimenSubsystem.SpecimenLiftState.GRABBED)
                ),
                new InstantCommand(),
                () -> Robot.getInstance().specimenSubsystem.getSpecimenLiftState() == SpecimenSubsystem.SpecimenLiftState.GRAB
        );
    }
}
