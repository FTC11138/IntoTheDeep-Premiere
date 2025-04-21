package org.firstinspires.ftc.teamcode.commands.subsystem;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.hardware.Robot;

public class SpecimenLiftPowerCommand extends InstantCommand {
    public SpecimenLiftPowerCommand(double power) {
        super(
                () -> Robot.getInstance().specimenSubsystem.setSpecimenLiftPower(power)
        );
    }
}
