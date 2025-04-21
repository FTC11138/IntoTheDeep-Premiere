package org.firstinspires.ftc.teamcode.commands.advancedcommand;

import static org.firstinspires.ftc.teamcode.hardware.subsystems.SpecimenSubsystem.SpecimenClawState.OPEN;
import static org.firstinspires.ftc.teamcode.hardware.subsystems.SpecimenSubsystem.SpecimenLiftState.GRAB;
import static org.firstinspires.ftc.teamcode.hardware.subsystems.SpecimenSubsystem.SpecimenLiftState.HIGH;
import static org.firstinspires.ftc.teamcode.hardware.subsystems.SpecimenSubsystem.SpecimenLiftState.LOW;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.subsystem.SpecimenClawStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.SpecimenLiftPowerCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.SpecimenLiftStateCommand;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.util.Constants;

public class SpecimenDepositCommand extends ConditionalCommand {
    public SpecimenDepositCommand() {
        super(
                new SequentialCommandGroup(
//                        new ConditionalCommand(
//                                new SpecimenLiftPositionSyncCommand(Constants.specimenLiftHigh - Constants.specimenLiftHangOffset, 10),
//                                new SpecimenLiftPositionSyncCommand(Constants.specimenLiftLow - Constants.specimenLiftHangOffset, 10),
//                                () -> Robot.getInstance().specimenSubsystem.getSpecimenLiftState() == HIGH
//                        ),
                        new SpecimenLiftPowerCommand(-Constants.specimenLiftHangPower),
                        new WaitCommand(Constants.specimenLiftHangTime),
                        new SpecimenLiftPowerCommand(0),
                        new SpecimenClawStateCommand(OPEN)
                ),
                new SpecimenLiftStateCommand(GRAB),
                () -> Robot.getInstance().specimenSubsystem.getSpecimenLiftState() == HIGH ||
                        Robot.getInstance().specimenSubsystem.getSpecimenLiftState() == LOW
        );
    }
}
