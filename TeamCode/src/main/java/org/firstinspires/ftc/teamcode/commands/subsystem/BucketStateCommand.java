package org.firstinspires.ftc.teamcode.commands.subsystem;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.DepositSubsystem;

public class BucketStateCommand extends InstantCommand {
    public BucketStateCommand(DepositSubsystem.BucketState state) {
        super(
                () -> Robot.getInstance().depositSubsystem.updateBucketState(state)
        );
    }
}
