package org.firstinspires.ftc.teamcode.commands.advancedcommand;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.subsystem.ClawStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.NewArmStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.ExtensionPositionCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.RotateStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.WristStateCommand;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.NewIntakeSubsystem;
import org.firstinspires.ftc.teamcode.util.Constants;

public class NewIntakePullBackCommand extends SequentialCommandGroup {
    public NewIntakePullBackCommand() {
        super(
                new InstantCommand(Robot.getInstance().data::startIntaking),
                new NewArmStateCommand(NewIntakeSubsystem.ArmState.DOWN),
                new WristStateCommand(NewIntakeSubsystem.WristState.GRAB),
                new WaitCommand(200),
                //Either make a seperate command to incorporate the camera or include here!
                new ClawStateCommand(NewIntakeSubsystem.ClawState.CLOSE),
                new WaitCommand(200),
//                //lower wait after testing!
                new InstantCommand(Robot.getInstance().data::stopIntaking),
                new RotateStateCommand(NewIntakeSubsystem.RotateState.HORIZONTAL),
                new NewArmStateCommand(NewIntakeSubsystem.ArmState.UP),
                new WristStateCommand(NewIntakeSubsystem.WristState.STORE),
                new ExtensionPositionCommand(Constants.extMin),
                new WaitCommand((int) (Robot.getInstance().newIntakeSubsystem.getExtensionPosition() * 0.6))
        );
    }
}
