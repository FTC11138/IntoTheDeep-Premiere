package org.firstinspires.ftc.teamcode.commands.subsystem;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.hardware.Robot;

public class SpecLiftResetCommand extends InstantCommand {
    public SpecLiftResetCommand() {
        super(
                () -> Robot.getInstance().specimenSubsystem.resetLift()
        );
    }
}
