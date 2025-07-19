package org.firstinspires.ftc.teamcode.commands.advancedcommand;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.util.Constants;

import java.util.function.IntSupplier;

public class ExtensionJumpCommand extends InstantCommand {
    public ExtensionJumpCommand(int dir) {
        super(
                () -> {
                    Robot.getInstance().newIntakeSubsystem.setTargetExtensionPosition(
                            Robot.getInstance().data.extensionPosition + Constants.extJump * dir
                    );
                }
        );
    }

    public ExtensionJumpCommand(int dir, int dist) {
        super(
                () -> {
                    Robot.getInstance().newIntakeSubsystem.setTargetExtensionPosition(
                            Robot.getInstance().data.extensionPosition + (dist * dir)
                    );
                }
        );
    }

    public ExtensionJumpCommand(int dir, IntSupplier dist) {
        super(
                () -> {
                    Robot.getInstance().newIntakeSubsystem.setTargetExtensionPosition(
                            Robot.getInstance().data.extensionPosition + (dist.getAsInt() * dir)
                    );
                }
        );
    }
}
