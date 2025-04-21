package org.firstinspires.ftc.teamcode.commands.subsystem;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.SpecimenSubsystem;

public class SpecimenClawStateCommand extends InstantCommand {
    public SpecimenClawStateCommand(SpecimenSubsystem.SpecimenClawState state) {
        super(
                () -> Robot.getInstance().specimenSubsystem.updateSpecimenClawState(state)
        );
    }
}
