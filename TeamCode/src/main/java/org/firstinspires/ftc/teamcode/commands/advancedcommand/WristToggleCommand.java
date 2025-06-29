package org.firstinspires.ftc.teamcode.commands.advancedcommand;

import com.arcrobotics.ftclib.command.ConditionalCommand;

import org.firstinspires.ftc.teamcode.commands.subsystem.RotateStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.WristStateCommand;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.NewIntakeSubsystem;

public class WristToggleCommand extends ConditionalCommand {
    public WristToggleCommand() {
        super(
                new WristStateCommand(NewIntakeSubsystem.WristState.PREGRABBACK),
                new WristStateCommand(NewIntakeSubsystem.WristState.PREGRABFORWARD),
                () -> Robot.getInstance().newIntakeSubsystem.wristState == NewIntakeSubsystem.WristState.PREGRABFORWARD
        );
    }
}
