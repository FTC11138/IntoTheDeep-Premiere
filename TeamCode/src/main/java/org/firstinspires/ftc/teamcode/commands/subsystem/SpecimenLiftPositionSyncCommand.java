package org.firstinspires.ftc.teamcode.commands.subsystem;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.hardware.Robot;

public class SpecimenLiftPositionSyncCommand extends CommandBase {

    private final int target, error;
    private int currentPosition;

    public SpecimenLiftPositionSyncCommand(int target, int error) {
        this.target = target;
        this.error = error;
        this.currentPosition = Robot.getInstance().specimenSubsystem.getSpecimenLiftPosition();
    }

    @Override
    public void initialize() {
        Robot.getInstance().specimenSubsystem.setTargetSpecimenLiftPosition(target);
    }

    @Override
    public void execute() {
        this.currentPosition = Robot.getInstance().specimenSubsystem.getSpecimenLiftPosition();
    }

    @Override
    public boolean isFinished() {
        return (Math.abs(this.target - this.currentPosition) <= this.error);
    }
}
