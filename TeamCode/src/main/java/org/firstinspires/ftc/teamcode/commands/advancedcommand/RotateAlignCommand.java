package org.firstinspires.ftc.teamcode.commands.advancedcommand;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.NewIntakeSubsystem;

public class RotateAlignCommand extends InstantCommand {

    public RotateAlignCommand() {
        super(
                () -> {
                    int angle = (int) Robot.getInstance().cameraSubsystem.getCameraAngleSample();
                    NewIntakeSubsystem.RotateState state;

                    if (angle < 22) {
                        state = NewIntakeSubsystem.RotateState.ANGLE_0;
                    } else if (angle < 77) {
                        state = NewIntakeSubsystem.RotateState.ANGLE_45;
                    } else if (angle < 112) {
                        return;
                    } else if (angle < 157) {
                        state = NewIntakeSubsystem.RotateState.ANGLE_135;
                    } else {
                        state = NewIntakeSubsystem.RotateState.ANGLE_180;
                    }

                    Robot.getInstance().newIntakeSubsystem.updateRotateState(state);
                }
        );
    }
}
