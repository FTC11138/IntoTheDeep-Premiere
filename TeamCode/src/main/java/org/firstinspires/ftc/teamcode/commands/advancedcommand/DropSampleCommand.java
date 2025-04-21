package org.firstinspires.ftc.teamcode.commands.advancedcommand;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.subsystem.BucketStateCommand;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.DepositSubsystem;

public class DropSampleCommand extends SequentialCommandGroup {
    public DropSampleCommand() {
        super(
                new BucketStateCommand(DepositSubsystem.BucketState.DROP),
                new InstantCommand(Robot.getInstance().data::setSampleUnloaded),
                new WaitCommand(750),
                new BucketStateCommand(DepositSubsystem.BucketState.INTAKE)
        );
    }
}
