package org.firstinspires.ftc.teamcode.commands.subsystem;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.hardware.Robot;

public class IntakeSensLightToggleCommand extends InstantCommand {
    public IntakeSensLightToggleCommand() {
        super(
                () -> Robot.getInstance().intakeSubsystem.toggleLight()
        );
    }
}
