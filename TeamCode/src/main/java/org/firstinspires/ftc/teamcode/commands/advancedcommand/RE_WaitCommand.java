package org.firstinspires.ftc.teamcode.commands.advancedcommand;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.WaitCommand;

import java.util.function.IntSupplier;

public class RE_WaitCommand extends InstantCommand {
    public RE_WaitCommand(IntSupplier supplier) {
        super(
                () -> CommandScheduler.getInstance().schedule(new WaitCommand(supplier.getAsInt()))
        );
    }
}
