package org.firstinspires.ftc.teamcode.commands.advancedcommand;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import org.firstinspires.ftc.teamcode.util.Constants;

import org.firstinspires.ftc.teamcode.commands.subsystem.BucketStateCommand;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.DepositSubsystem;

public class SubmersibleConstantCommand extends SequentialCommandGroup {
    public SubmersibleConstantCommand() {

            for (int i = 0; i < 9; i++) {
            new NewIntakePushOutCommand(400);
            new NewSamplePickupCommandSub();
            new NewIntakePullBackCommand();
            new NewSampleTransferCommand();
            new DropSampleCommand();

        }
    }
}
