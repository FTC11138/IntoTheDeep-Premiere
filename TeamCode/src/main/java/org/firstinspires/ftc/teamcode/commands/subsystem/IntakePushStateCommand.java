package org.firstinspires.ftc.teamcode.commands.subsystem;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.IntakeSubsystem;

public class IntakePushStateCommand extends InstantCommand {
    public IntakePushStateCommand(IntakeSubsystem.IntakePushState state) {
        super(
                () -> Robot.getInstance().intakeSubsystem.updateIntakePushState(state)
        );
    }
}
