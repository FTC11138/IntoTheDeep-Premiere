package org.firstinspires.ftc.teamcode.commands.subsystem;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.hardware.Robot;

public class NewArmPositionCommand extends InstantCommand {
    public NewArmPositionCommand(double pos) {
        super(
                () -> Robot.getInstance().newIntakeSubsystem.setArmPosition(pos)
        );
    }
}
