package org.firstinspires.ftc.teamcode.commands.subsystem;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.hardware.Robot;

public class WristPositionCommand extends InstantCommand {
    public WristPositionCommand(double pos) {
        super(
                () -> Robot.getInstance().newIntakeSubsystem.setWristPosition(pos)
        );
    }
}
