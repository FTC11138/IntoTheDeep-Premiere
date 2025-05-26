package org.firstinspires.ftc.teamcode.opmode;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ScheduleCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.commands.advancedcommand.DropSampleCommand;
import org.firstinspires.ftc.teamcode.commands.advancedcommand.IntakePullBackCommand;
import org.firstinspires.ftc.teamcode.commands.advancedcommand.IntakePushOutCommand;
import org.firstinspires.ftc.teamcode.commands.advancedcommand.LiftDownCommand;
import org.firstinspires.ftc.teamcode.commands.advancedcommand.LiftMidCommand;
import org.firstinspires.ftc.teamcode.commands.advancedcommand.LiftUpCommand;
import org.firstinspires.ftc.teamcode.commands.advancedcommand.NewIntakePullBackCommand;
import org.firstinspires.ftc.teamcode.commands.advancedcommand.NewIntakePushOutCommand;
import org.firstinspires.ftc.teamcode.commands.advancedcommand.NewSampleEjectCommand;
import org.firstinspires.ftc.teamcode.commands.advancedcommand.NewSamplePickupCommand;
import org.firstinspires.ftc.teamcode.commands.advancedcommand.NewSampleTransferCommand;
import org.firstinspires.ftc.teamcode.commands.advancedcommand.SampleEjectCommand;
import org.firstinspires.ftc.teamcode.commands.advancedcommand.SampleTransferCommand;
import org.firstinspires.ftc.teamcode.commands.advancedcommand.SpecimenDepositCommand;
import org.firstinspires.ftc.teamcode.commands.advancedcommand.SpecimenGrabCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.ExtensionResetCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.LiftPowerCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.LiftResetCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.SpecLiftResetCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.SpecimenLiftStateCommand;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.RobotData;
import org.firstinspires.ftc.teamcode.hardware.subsystems.SpecimenSubsystem;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.Globals;

@TeleOp (name = "Solo")
public class TeleOp_Solo extends CommandOpMode {

    private final Robot robot = Robot.getInstance();
    private final RobotData data = Robot.getInstance().data;
    private GamepadEx g1;


    boolean teleOpEnabled = false;

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

    boolean lastRightStickButton;
    boolean lastLeftStickbutton;

    boolean lastLiftChangeJoystickUp;
    boolean lastLiftChangeJoystickDown;

    boolean lastPS;
    boolean lastStart;
    boolean lastBack;


    double lastIntakeSpeed;
    double lastIntakeDistance;


    @Override
    public void initialize() {

        g1 = new GamepadEx(gamepad1);

        Globals.IS_AUTO = false;

        robot.initialize(hardwareMap, telemetry);
        robot.follower.setPose(robot.data.currentPose);

        CommandScheduler.getInstance().reset();
        CommandScheduler.getInstance().cancelAll();

        data.stopIntaking();
        data.stopScoring();
        data.setSampleUnloaded();

        robot.follower.startTeleopDrive();

    }

    @Override
    public void run() {

        if (teleOpEnabled) {

            CommandScheduler.getInstance().run();

            robot.periodic();

            robot.updateData();
            robot.write();

            robot.follower.setTeleOpMovementVectors(
                    -gamepad1.left_stick_y * (data.scoring || data.intaking ? 0.5 : 1),
                    -gamepad1.left_stick_x * (data.scoring || data.intaking ? 0.5 : 1),
                    -gamepad1.right_stick_x * (data.scoring || data.intaking ? 0.7 : 1),
                    Constants.robotCentric
            );

        }

        boolean liftChangeJoystickUp = gamepad1.right_stick_y < -0.8;
        boolean liftChangeJoystickDown = gamepad1.right_stick_y > 0.8;

        if (robot.data.intaking) {
            robot.newIntakeSubsystem.setExtensionPower(-gamepad1.right_stick_y);
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
        boolean rightStickButton = g1.getButton(GamepadKeys.Button.RIGHT_STICK_BUTTON);
        boolean leftStickButton = g1.getButton(GamepadKeys.Button.LEFT_STICK_BUTTON);
        boolean ps = gamepad1.ps;
        boolean start = g1.getButton(GamepadKeys.Button.START);
        boolean back = g1.getButton(GamepadKeys.Button.BACK);

//        boolean sampleIn = robot.sensorSubsystem.getIntakeDistance() < Constants.samplePickupTolerance;

        scheduleCommand(lastPS, ps, new SpecLiftResetCommand());

        scheduleCommand(lastA, a, new SequentialCommandGroup(
                new DropSampleCommand(),
                new WaitCommand(400),
                new ConditionalCommand(
                        new LiftDownCommand(),
                        new InstantCommand(),
                        () -> robot.data.scoring
                )
        ));
        scheduleCommand(lastB, b, new LiftDownCommand());
        scheduleCommand(lastY, y, new LiftMidCommand());

        scheduleCommand(lastLeftBumper, leftBumper, new NewSampleEjectCommand());
        scheduleCommand(lastRightBumper, rightBumper, new LiftUpCommand());

        scheduleCommand(lastDpadDown, dpadDown, new SpecimenLiftStateCommand(SpecimenSubsystem.SpecimenLiftState.GRAB));
        scheduleCommand(lastDpadUp, dpadUp, new ConditionalCommand(
                new SpecimenLiftStateCommand(SpecimenSubsystem.SpecimenLiftState.LOW),
                new SpecimenLiftStateCommand(SpecimenSubsystem.SpecimenLiftState.HIGH),
                () -> robot.specimenSubsystem.getSpecimenLiftState() == SpecimenSubsystem.SpecimenLiftState.HIGH
        ));

        scheduleCommand(lastDpadLeft, dpadLeft, new SpecimenGrabCommand()
                .andThen(new SpecimenLiftStateCommand(SpecimenSubsystem.SpecimenLiftState.HIGH)));
        scheduleCommand(lastDpadRight, dpadRight, new SpecimenDepositCommand().alongWith(new InstantCommand(() -> gamepad1.rumble(500)))
                .andThen(new WaitCommand(200).andThen(new SpecimenLiftStateCommand(SpecimenSubsystem.SpecimenLiftState.GRAB))));

        if (leftStickButton) {
            Globals.LIMITS = false;
            CommandScheduler.getInstance().schedule(new LiftPowerCommand(-0.5));
        } else if (lastLeftStickbutton) { // if just released set power back to 0
            Globals.LIMITS = true;
            CommandScheduler.getInstance().schedule(new LiftPowerCommand(0));
        }

        scheduleCommand(lastRightStickButton, rightStickButton, new LiftResetCommand().alongWith(new InstantCommand(() -> gamepad1.rumble(500))));
        scheduleCommand(lastBack, back, new ExtensionResetCommand().alongWith(new InstantCommand(() -> gamepad1.rumble(500))));



        if (!lastX && x) {
            Constants.robotCentric = !Constants.robotCentric;
            gamepad1.rumble(500);
            if (Constants.robotCentric) gamepad1.setLedColor(1, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
            else gamepad1.setLedColor(0, 0, 1, Gamepad.LED_DURATION_CONTINUOUS);
        }

        if (!lastStart && start) {
            teleOpEnabled = true;
            gamepad1.rumble(2000);
        }





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
        lastRightStickButton = rightStickButton;
        lastLeftStickbutton = leftStickButton;
        lastPS = ps;
        lastStart = start;


        double intakeSpeed = robot.sensorSubsystem.getIntakeSpeed();
        double intakeDistance = robot.sensorSubsystem.getIntakeDistance();
        boolean leftTrigger = gamepad1.left_trigger > .5;
        boolean rightTrigger = gamepad1.right_trigger > .5;

        if (rightTrigger && !lastRightTrigger ||
                (
                        robot.data.intaking
                        && (intakeDistance < Constants.samplePickupTolerance)
                )) {
            gamepad1.rumble(500);
            CommandScheduler.getInstance().schedule(new NewSamplePickupCommand());
//            CommandScheduler.getInstance().schedule(new WaitCommand(5000));
//            CommandScheduler.getInstance().schedule(new NewSampleTransferCommand());
//                    new ConditionalCommand(
//                            new NewIntakePullBackCommand().andThen(new NewSampleTransferCommand()),
//                            new NewSampleTransferCommand())
//                            () -> robot.data.intaking
//                    )
//            );
        }

        if (leftTrigger && !lastLeftTrigger) {
            CommandScheduler.getInstance().schedule(new NewIntakePushOutCommand(Constants.extIntake));
        }

        lastLeftTrigger = leftTrigger;
        lastRightTrigger = rightTrigger;
        lastIntakeDistance = intakeDistance;

        if (gamepad1.touchpad) {
            robot.follower.setPose(new Pose());
            gamepad1.rumble(500);
            gamepad1.setLedColor(0, 1, 0, 1000);
//            CommandScheduler.getInstance().schedule(new NewSampleEjectCommand());
        }


    }

    private void scheduleCommand(boolean lastPress, boolean currPress, Command command) {
        if (currPress && !lastPress) {
            CommandScheduler.getInstance().schedule(command);
        }
    }
}
