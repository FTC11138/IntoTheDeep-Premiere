package org.firstinspires.ftc.teamcode.commands.advancedcommand;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.subsystem.ArmStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.ClawStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.NewArmStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.WristStateCommand;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.NewIntakeSubsystem;

public class NewSamplePrePickupCommand extends SequentialCommandGroup {
    public NewSamplePrePickupCommand() {
        super(
                new RotateAlignCommand(),
                new ExtensionJumpCommand(1, (int) Robot.getInstance().cameraSubsystem.getExtDistanceSample()),
                new WaitCommand(300),
                new WristStateCommand(NewIntakeSubsystem.WristState.GRAB),
                new WaitCommand(100),
                new ArmStateCommand(NewIntakeSubsystem.ArmState.FLAT),
                new ClawStateCommand(NewIntakeSubsystem.ClawState.OPEN_WIDE)
        );
    }
}
