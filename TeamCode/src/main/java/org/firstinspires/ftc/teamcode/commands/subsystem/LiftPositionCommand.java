package org.firstinspires.ftc.teamcode.commands.subsystem;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.hardware.Robot;

public class LiftPositionCommand extends InstantCommand {
    public LiftPositionCommand(int target, double power) {
        super(
                () -> Robot.getInstance().depositSubsystem.setTargetLiftPosition(target, power)
        );
    }
}
