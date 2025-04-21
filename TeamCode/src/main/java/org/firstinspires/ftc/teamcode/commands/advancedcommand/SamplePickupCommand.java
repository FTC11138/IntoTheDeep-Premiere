package org.firstinspires.ftc.teamcode.commands.advancedcommand;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.hardware.Robot;

public class SamplePickupCommand extends SequentialCommandGroup {
    public SamplePickupCommand() {
        super(
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                new IntakePushOutCommand(300),
                                new WaitCommand(500)
                        ),
                        new InstantCommand(),
                        () -> Robot.getInstance().data.intaking
                ),
                new SampleAlignCommand(),
                new SampleExtendGrabCommand(),
                new IntakePullBackCommand()
        );
    }
}
