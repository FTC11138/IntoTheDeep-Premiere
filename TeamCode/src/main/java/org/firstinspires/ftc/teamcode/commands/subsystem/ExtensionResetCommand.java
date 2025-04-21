package org.firstinspires.ftc.teamcode.commands.subsystem;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.hardware.Robot;

public class ExtensionResetCommand extends InstantCommand {
    public ExtensionResetCommand() {
        super(
                () -> Robot.getInstance().intakeSubsystem.resetExtension()
        );
    }
}
