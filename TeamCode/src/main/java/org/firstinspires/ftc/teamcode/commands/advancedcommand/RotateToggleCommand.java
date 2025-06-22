package org.firstinspires.ftc.teamcode.commands.advancedcommand;

import com.arcrobotics.ftclib.command.ConditionalCommand;

import org.firstinspires.ftc.teamcode.commands.subsystem.RotateStateCommand;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.NewIntakeSubsystem;

public class RotateToggleCommand extends ConditionalCommand {
    public RotateToggleCommand() {
        super(
                new RotateStateCommand(NewIntakeSubsystem.RotateState.VERTICAL),
                new RotateStateCommand(NewIntakeSubsystem.RotateState.HORIZONTAL),
                () -> Robot.getInstance().newIntakeSubsystem.rotateState == NewIntakeSubsystem.RotateState.HORIZONTAL
        );
    }
}
