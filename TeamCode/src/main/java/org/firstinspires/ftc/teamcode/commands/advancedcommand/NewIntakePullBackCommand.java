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

import java.util.function.IntSupplier;

public class NewIntakePullBackCommand extends SequentialCommandGroup {

    IntSupplier test = () -> {return (int) (Robot.getInstance().newIntakeSubsystem.getExtensionPosition() * 0.6);};

    public NewIntakePullBackCommand() {
        super(
                new InstantCommand(Robot.getInstance().data::stopIntaking),
                new RotateStateCommand(NewIntakeSubsystem.RotateState.HORIZONTAL),
                new NewArmStateCommand(NewIntakeSubsystem.ArmState.UP),
                new ExtensionPositionCommand(Constants.extMin),
                new WristStateCommand(NewIntakeSubsystem.WristState.STORE),
                new WaitCommand((int) (Robot.getInstance().newIntakeSubsystem.getExtensionPosition() * 0.5))
        );
    }
}
