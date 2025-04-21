package org.firstinspires.ftc.teamcode.commands.subsystem;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.hardware.Robot;

public class LiftResetCommand extends InstantCommand {
    public LiftResetCommand() {
        super(
                () -> Robot.getInstance().depositSubsystem.resetLift()
        );
    }
}
