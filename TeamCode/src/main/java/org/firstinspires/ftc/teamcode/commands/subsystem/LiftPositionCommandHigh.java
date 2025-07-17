package org.firstinspires.ftc.teamcode.commands.subsystem;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.hardware.Robot;

public class LiftPositionCommandHigh extends InstantCommand {
    public LiftPositionCommandHigh(int target, double power) {
        super(
                () -> Robot.getInstance().depositSubsystem.setTargetLiftPositionHigh(target, power)
        );
    }
}