package org.firstinspires.ftc.teamcode.commands.advancedcommand;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.subsystem.ClawStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.IntakeStateCommand;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystems.NewIntakeSubsystem;
import org.firstinspires.ftc.teamcode.util.Constants;

public class NewSampleEjectCommand extends SequentialCommandGroup {
    public NewSampleEjectCommand() {
        super(
                new ClawStateCommand(NewIntakeSubsystem.ClawState.OPEN),
                new WaitCommand(5000),
                new ClawStateCommand(NewIntakeSubsystem.ClawState.CLOSE)
        );
    }
}