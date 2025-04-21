package org.firstinspires.ftc.teamcode.commands.advancedcommand;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.subsystem.BucketStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.LiftPositionCommand;
import org.firstinspires.ftc.teamcode.hardware.subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.util.Constants;

public class Ascent1Command extends SequentialCommandGroup {
    public Ascent1Command() {
        super(
                new LiftPositionCommand(Constants.liftAscent, 1),
                new WaitCommand(500),
                new BucketStateCommand(DepositSubsystem.BucketState.ASCENT)
        );
    }
}
