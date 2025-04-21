package org.firstinspires.ftc.teamcode.opmode;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.advancedcommand.DropSampleCommand;
import org.firstinspires.ftc.teamcode.commands.advancedcommand.ExtensionJumpCommand;
import org.firstinspires.ftc.teamcode.commands.advancedcommand.IntakePullBackCommand;
import org.firstinspires.ftc.teamcode.commands.advancedcommand.IntakePushOutCommand;
import org.firstinspires.ftc.teamcode.commands.advancedcommand.LiftDownCommand;
import org.firstinspires.ftc.teamcode.commands.advancedcommand.LiftMidCommand;
import org.firstinspires.ftc.teamcode.commands.advancedcommand.LiftUpCommand;
import org.firstinspires.ftc.teamcode.commands.advancedcommand.SampleTransferCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.IntakeStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.LiftPowerCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.LiftResetCommand;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.RobotData;
import org.firstinspires.ftc.teamcode.hardware.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.Globals;

@Disabled
@TeleOp (name = "Duo")
public class TeleOp_Duo extends CommandOpMode {

    private final Robot robot = Robot.getInstance();
    private final RobotData data = Robot.getInstance().data;
    private final CommandScheduler cs = CommandScheduler.getInstance();
    private GamepadEx g1;
    private GamepadEx g2;

    Pose currentPose;
    double heading;
    double fieldCentricOffset;

    boolean lastLeftTrigger;
    boolean lastRightTrigger;

    boolean lastA;
    boolean lastB;
    boolean lastX;
    boolean lastY;

    boolean lastLeftBumper;
    boolean lastRightBumper;

    boolean lastDpadUp;
    boolean lastDpadDown;
    boolean lastDpadLeft;
    boolean lastDpadRight;

    boolean lastLiftChangeJoystickUp;
    boolean lastLiftChangeJoystickDown;


    @Override
    public void initialize() {

        g1 = new GamepadEx(gamepad1);
        g1 = new GamepadEx(gamepad2);

        Globals.IS_AUTO = false;

        robot.initialize(hardwareMap, telemetry);
        robot.follower.setPose(robot.data.currentPose);

        data.stopIntaking();
        data.stopScoring();
        data.setSampleUnloaded();

        switch (Globals.ALLIANCE) {
            case RED:
                fieldCentricOffset = Math.toRadians(90);
                break;
            case BLUE:
                fieldCentricOffset = Math.toRadians(-90);
                break;
        }

//        g1.getGamepadButton(GamepadKeys.Button.B)
//                .whenPressed(() -> cs.schedule(new LiftDownCommand()));
//
//        g1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
//                .whenPressed(() -> cs.schedule(new LiftUpCommand()));
//
//        g1.getGamepadButton(GamepadKeys.Button.A)
//                .whenPressed(() -> cs.schedule(new DropSampleCommand().andThen(new LiftDownCommand())));
//
//        g1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
//                .whenPressed(() -> cs.schedule(new SampleTransferCommand()));
//
//        g1.getGamepadButton(GamepadKeys.Button.DPAD_UP)
//                .whenPressed(() -> cs.schedule(new ExtensionJumpCommand(1)));
//
//        g1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
//                .whenPressed(() -> cs.schedule(new ExtensionJumpCommand(-1)));

    }

    @Override
    public void run() {

        cs.run();
        robot.periodic();


        robot.follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, false);

        boolean liftChangeJoystickUp = gamepad1.right_stick_y < -0.8;
        boolean liftChangeJoystickDown = gamepad1.right_stick_y > 0.8;

        if (robot.data.intaking) {
            if (liftChangeJoystickUp && !lastLiftChangeJoystickUp) {
                cs.schedule(new ExtensionJumpCommand(1));
            } else if (liftChangeJoystickDown && !lastLiftChangeJoystickDown) {
                cs.schedule(new ExtensionJumpCommand(-1));
            }
        }

        lastLiftChangeJoystickUp = liftChangeJoystickUp;
        lastLiftChangeJoystickDown = liftChangeJoystickDown;

        boolean a = g1.getButton(GamepadKeys.Button.A);
        boolean b = g1.getButton(GamepadKeys.Button.B);
        boolean x = g1.getButton(GamepadKeys.Button.X);
        boolean y = g1.getButton(GamepadKeys.Button.Y);
        boolean leftBumper = g1.getButton(GamepadKeys.Button.LEFT_BUMPER);
        boolean rightBumper = g1.getButton(GamepadKeys.Button.RIGHT_BUMPER);
        boolean dpadUp = g1.getButton(GamepadKeys.Button.DPAD_UP);
        boolean dpadDown = g1.getButton(GamepadKeys.Button.DPAD_DOWN);
        boolean dpadLeft = g1.getButton(GamepadKeys.Button.DPAD_LEFT);
        boolean dpadRight = g1.getButton(GamepadKeys.Button.DPAD_RIGHT);

        scheduleCommand(lastA, a, new SequentialCommandGroup(
                new DropSampleCommand(),
                new WaitCommand(400),
                new LiftDownCommand()
        ));
        scheduleCommand(lastB, b, new LiftDownCommand());
        scheduleCommand(lastX, x, new IntakeStateCommand(IntakeSubsystem.IntakeState.OUT));
        scheduleCommand(lastY, y, new LiftMidCommand());

        scheduleCommand(lastLeftBumper, leftBumper, new SampleTransferCommand());
        scheduleCommand(lastRightBumper, rightBumper, new LiftUpCommand());

        scheduleCommand(lastDpadDown, dpadDown, new ExtensionJumpCommand(-1));
        scheduleCommand(lastDpadUp, dpadUp, new ExtensionJumpCommand(1));

        if (dpadLeft) {
            Globals.LIMITS = false;
            cs.schedule(new LiftPowerCommand(-0.5));
        } else {
            Globals.LIMITS = true;
            cs.schedule(new LiftPowerCommand(0));
        }

        scheduleCommand(lastDpadRight, dpadRight, new LiftResetCommand());

        lastA = a;
        lastB = b;
        lastX = x;
        lastY = y;
        lastLeftBumper = leftBumper;
        lastRightBumper = rightBumper;
        lastDpadUp = dpadUp;
        lastDpadDown = dpadDown;
        lastDpadLeft = dpadLeft;
        lastDpadRight = dpadRight;


        boolean leftTrigger = gamepad1.left_trigger > .5;
        boolean rightTrigger = gamepad1.right_trigger > .5;

        if (leftTrigger && !lastLeftTrigger) {
            cs.schedule(new IntakePushOutCommand(Constants.extIntake));
        }

        if (rightTrigger && !lastRightTrigger) {
            cs.schedule(new IntakePullBackCommand().andThen(new SampleTransferCommand()));
        }

        lastLeftTrigger = leftTrigger;
        lastRightTrigger = rightTrigger;

        robot.updateData();
        robot.write();

    }

    private void scheduleCommand(boolean lastPress, boolean currPress, Command command) {
        if (currPress && !lastPress) {
            cs.schedule(command);
        }
    }
}
