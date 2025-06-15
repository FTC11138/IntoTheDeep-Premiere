package org.firstinspires.ftc.teamcode.commands.advancedcommand;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.subsystem.ClawStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.NewArmStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.WristStateCommand;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.NewIntakeSubsystem;

public class SampleGrabCommand extends SequentialCommandGroup {
    public SampleGrabCommand() {
        super(
                new InstantCommand(Robot.getInstance().data::startIntaking),
                new NewArmStateCommand(NewIntakeSubsystem.ArmState.DOWN),
                new WristStateCommand(NewIntakeSubsystem.WristState.GRAB),
                new WaitCommand(150),
                //Either make a seperate command to incorporate the camera or include here!
                new ClawStateCommand(NewIntakeSubsystem.ClawState.CLOSE),
                new WaitCommand(150)
        );
    }
}
