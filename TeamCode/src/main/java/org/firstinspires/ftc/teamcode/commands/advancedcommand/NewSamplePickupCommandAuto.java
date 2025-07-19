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
import org.firstinspires.ftc.teamcode.util.Globals;

public class NewSamplePickupCommandAuto extends SequentialCommandGroup {
    public
    NewSamplePickupCommandAuto() {
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

                new ClawStateCommand(NewIntakeSubsystem.ClawState.OPEN_WIDE),
                new SequentialCommandGroup(
                        new RotateAlignCommand(),
                        new ExtensionJumpCommand(1, () -> (int) Robot.getInstance().cameraSubsystem.getExtDistanceSample()),
                        new WaitCommand(300)
                ),
//                new ConditionalCommand(
//                        new SequentialCommandGroup(
//                                new RotateAlignCommand(),
//                                new ExtensionJumpCommand(1, (int) Robot.getInstance().cameraSubsystem.getExtDistanceSample()),
//                                new WaitCommand(300)
//                        ),
//                        new ExtensionJumpCommand(-1, Constants.extGrabJump),
//                        () -> !Globals.IS_AUTO
//
//                ),

                new NewArmStateCommand(NewIntakeSubsystem.ArmState.DOWN),
                new WristStateCommand(NewIntakeSubsystem.WristState.GRAB),
                new WaitCommand(400),
                new ClawStateCommand(NewIntakeSubsystem.ClawState.CLOSE),
                new WaitCommand(200),
                new ArmStateCommand(NewIntakeSubsystem.ArmState.FLAT)
//                new SampleExtendGrabCommand(),
        );
    }
}
