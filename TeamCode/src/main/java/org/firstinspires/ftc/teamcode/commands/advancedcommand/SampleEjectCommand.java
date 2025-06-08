package org.firstinspires.ftc.teamcode.commands.advancedcommand;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.subsystem.IntakeStateCommand;
import org.firstinspires.ftc.teamcode.hardware.subsystems.IntakeSubsystem;

public class SampleEjectCommand extends SequentialCommandGroup {
    public SampleEjectCommand() {
        super(

//                new IntakeStateCommand(IntakeSubsystem.IntakeState.OUT),
//                new WaitCommand(500),
//                new IntakeStateCommand(IntakeSubsystem.IntakeState.IN)
        );
    }
}
