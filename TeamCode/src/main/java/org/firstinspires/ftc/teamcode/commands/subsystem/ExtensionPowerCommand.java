package org.firstinspires.ftc.teamcode.commands.subsystem;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.hardware.Robot;

public class ExtensionPowerCommand extends InstantCommand {
    public ExtensionPowerCommand(double power) {
        super(
                () -> Robot.getInstance().intakeSubsystem.setExtensionPower(power)
        );
    }
}
