package org.firstinspires.ftc.teamcode.commands.subsystem;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.hardware.Robot;

public class ExtensionPositionCommand extends InstantCommand {
    public ExtensionPositionCommand(int target) {
        super(
                () -> Robot.getInstance().intakeSubsystem.setTargetExtensionPosition(target)
        );
    }
}
