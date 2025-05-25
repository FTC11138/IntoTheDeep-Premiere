package org.firstinspires.ftc.teamcode.commands.subsystem;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.hardware.Robot;

public class NewExtensionPositionCommand extends InstantCommand {
    public NewExtensionPositionCommand(int target) {
        super(
                () -> Robot.getInstance().newIntakeSubsystem.setTargetExtensionPosition(target)
        );
    }
}
