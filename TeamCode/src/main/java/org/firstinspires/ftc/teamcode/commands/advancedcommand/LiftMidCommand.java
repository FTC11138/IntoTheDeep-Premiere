package org.firstinspires.ftc.teamcode.commands.advancedcommand;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.subsystem.LiftPositionCommand;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.util.Constants;

public class LiftMidCommand extends SequentialCommandGroup {
    public LiftMidCommand() {
        super(
                new LiftPositionCommand(Constants.liftMid, 1),
                new InstantCommand(Robot.getInstance().data::startScoring)
        );
    }
}
