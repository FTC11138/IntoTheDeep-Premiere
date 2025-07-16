package org.firstinspires.ftc.teamcode.commands.advancedcommand;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;

import org.firstinspires.ftc.teamcode.commands.drivecommand.PathCommand;
import org.firstinspires.ftc.teamcode.commands.drivecommand.StrafeCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.ArmStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.ClawStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.NewArmStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.WristStateCommand;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.NewIntakeSubsystem;

public class NewSamplePickupCommandSub extends SequentialCommandGroup {
    public NewSamplePickupCommandSub() {
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
                        new ExtensionJumpCommand(1, (int) Robot.getInstance().cameraSubsystem.getExtDistanceSample()),
                        new StrafeCommand(Robot.getInstance().cameraSubsystem.getRobotDistanceSample()),
                        new WaitCommand(300) // TUNE THIS VALUE, ITS JUST SET HIGH TO DEBUG
                ),

                new ClawStateCommand(NewIntakeSubsystem.ClawState.OPEN_WIDE),
                
//                new WaitCommand(300),

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
