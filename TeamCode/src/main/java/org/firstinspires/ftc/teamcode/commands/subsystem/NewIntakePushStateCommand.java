package org.firstinspires.ftc.teamcode.commands.subsystem;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystems.NewIntakeSubsystem;

public class NewIntakePushStateCommand extends InstantCommand {
    public NewIntakePushStateCommand(NewIntakeSubsystem.IntakePushState state) {
        super(
                () -> {}
        );
    }
}
