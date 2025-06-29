package org.firstinspires.ftc.teamcode.commands.advancedcommand;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.subsystem.ArmStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.ClawStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.NewArmStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.WristStateCommand;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.NewIntakeSubsystem;
import org.firstinspires.ftc.teamcode.util.Constants;

public class NewSamplePickupCommand extends SequentialCommandGroup {
    public NewSamplePickupCommand() {
        super(
//                new ConditionalCommand(
//                        new SequentialCommandGroup(
//                                new NewIntakePushOutCommand(300),
//                                new WaitCommand(500)
//                        ),
//                        new InstantCommand(),
//                        () -> Robot.getInstance().data.intaking
//                ),
//                new NewSampleAlignCommand(), will need later, we have no camera currently -----------------------------------------------------------------
//                new ExtensionJumpCommand(-1, Constants.extGrabJump + (int) Robot.getInstance().data.sampleDy),
                new WaitCommand(100),
                new NewArmStateCommand(NewIntakeSubsystem.ArmState.DOWN),
                new WristStateCommand(NewIntakeSubsystem.WristState.GRAB),
                new WaitCommand(100),
                new ClawStateCommand(NewIntakeSubsystem.ClawState.CLOSE),
                new WaitCommand(200),
                new ArmStateCommand(NewIntakeSubsystem.ArmState.FLAT)
//                new SampleExtendGrabCommand(),
        );
    }
}
