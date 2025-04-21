package org.firstinspires.ftc.teamcode.commands.subsystem;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import org.firstinspires.ftc.teamcode.hardware.Robot;

public class LedSetCommand extends InstantCommand {
    public LedSetCommand(RevBlinkinLedDriver.BlinkinPattern pattern) {
        super(
                () -> Robot.getInstance().intakeSubsystem.setLeds(pattern)
        );
    }
}
