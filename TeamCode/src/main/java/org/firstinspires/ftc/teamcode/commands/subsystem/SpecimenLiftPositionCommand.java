package org.firstinspires.ftc.teamcode.commands.subsystem;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.hardware.Robot;

public class SpecimenLiftPositionCommand extends InstantCommand {
    public SpecimenLiftPositionCommand(int target) {
        super(
                () -> Robot.getInstance().specimenSubsystem.setTargetSpecimenLiftPosition(target)
        );
    }
}
