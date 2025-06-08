package org.firstinspires.ftc.teamcode.opmode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.advancedcommand.LiftDownCommand;
import org.firstinspires.ftc.teamcode.commands.advancedcommand.LiftUpCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.ArmStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.BucketStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.IntakePushStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.IntakeStateCommand;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystems.NewIntakeSubsystem;
import org.firstinspires.ftc.teamcode.util.Globals;

@TeleOp (name = "Tune")
public class TeleOp_Tune extends CommandOpMode {

    private final Robot robot = Robot.getInstance();
    private final CommandScheduler cs = CommandScheduler.getInstance();
    private GamepadEx g1;

    double fieldCentricOffset;


    @Override
    public void initialize() {

        g1 = new GamepadEx(gamepad1);

        Globals.IS_AUTO = false;

        robot.initialize(hardwareMap, telemetry);
        robot.follower.setPose(robot.data.currentPose);

        Robot.getInstance().data.stopIntaking();
        Robot.getInstance().data.stopScoring();

        switch (Globals.ALLIANCE) {
            case BLUE:
                fieldCentricOffset = Math.toRadians(-90);
                break;
            case RED:
                fieldCentricOffset = Math.toRadians(90);
                break;
        }

        g1.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(() -> cs.schedule(new LiftUpCommand()));

        g1.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(() -> cs.schedule(new LiftDownCommand()));

        g1.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(() -> cs.schedule(new ArmStateCommand(NewIntakeSubsystem.ArmState.INTAKE)));

        g1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(() -> cs.schedule(new ArmStateCommand(NewIntakeSubsystem.ArmState.TRANSFER)));

        g1.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)
                .whenPressed(() -> cs.schedule(new ArmStateCommand(NewIntakeSubsystem.ArmState.UP)));

        g1.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(() -> cs.schedule(new BucketStateCommand(DepositSubsystem.BucketState.INTAKE)));

        g1.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(() -> cs.schedule(new BucketStateCommand(DepositSubsystem.BucketState.DROP)));

        g1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(() -> cs.schedule(new IntakePushStateCommand(IntakeSubsystem.IntakePushState.PUSH)));

        g1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(() -> cs.schedule(new IntakePushStateCommand(IntakeSubsystem.IntakePushState.UP)));

        g1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(() -> cs.schedule(new IntakePushStateCommand(IntakeSubsystem.IntakePushState.STORE)));

        g1.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
                .whenPressed(() -> cs.schedule(new IntakeStateCommand(IntakeSubsystem.IntakeState.STOP)));

    }

    @Override
    public void run() {

        cs.run();
        robot.periodic();

        robot.depositSubsystem.setLiftPower(-gamepad1.right_stick_y);
        robot.intakeSubsystem.setExtensionPower(-gamepad1.left_stick_y);
        robot.specimenSubsystem.setSpecimenLiftPower(gamepad1.right_stick_x);

        robot.updateData();
        robot.write();

    }
}
