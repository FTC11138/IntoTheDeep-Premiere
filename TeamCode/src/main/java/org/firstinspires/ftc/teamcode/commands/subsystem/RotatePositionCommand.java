package org.firstinspires.ftc.teamcode.commands.subsystem;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.hardware.Robot;

public class RotatePositionCommand extends InstantCommand {
    public RotatePositionCommand(double pos) {
        super(
                () -> Robot.getInstance().newIntakeSubsystem.setRotatePosition(pos)
        );
    }
}
