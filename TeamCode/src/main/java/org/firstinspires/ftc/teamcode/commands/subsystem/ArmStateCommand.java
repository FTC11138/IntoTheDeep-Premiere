package org.firstinspires.ftc.teamcode.commands.subsystem;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.IntakeSubsystem;

public class ArmStateCommand extends InstantCommand {
    public ArmStateCommand(IntakeSubsystem.ArmState state) {
        super(
                () -> Robot.getInstance().intakeSubsystem.updateArmState(state)
        );
    }
}
