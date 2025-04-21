package org.firstinspires.ftc.teamcode.opmode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
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
import org.firstinspires.ftc.teamcode.commands.advancedcommand.SampleEjectCommand;
import org.firstinspires.ftc.teamcode.commands.advancedcommand.SampleTransferCommand;
import org.firstinspires.ftc.teamcode.commands.advancedcommand.SpecimenDepositCommand;
import org.firstinspires.ftc.teamcode.commands.advancedcommand.SpecimenGrabCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.ExtensionResetCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.LiftPowerCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.SpecimenLiftStateCommand;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.RobotData;
import org.firstinspires.ftc.teamcode.hardware.subsystems.SpecimenSubsystem;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.Globals;

@TeleOp (name = "Solo New")
public class TeleOp_Solo_New extends CommandOpMode {

    private final Robot robot = Robot.getInstance();
    private final RobotData data = Robot.getInstance().data;
    private GamepadEx g1;

    boolean teleOpEnabled = false;

    boolean lastLeftTrigger;
    boolean lastRightTrigger;

    boolean lastSampleIn;
    boolean lastSampleInTurnSpeed;


    @Override
    public void initialize() {

        g1 = new GamepadEx(gamepad1);

        Globals.IS_AUTO = false;

        robot.initialize(hardwareMap, telemetry);
        robot.follower.setPose(robot.data.currentPose);

        CommandScheduler.getInstance().reset();

        data.stopIntaking();
        data.stopScoring();

        robot.follower.startTeleopDrive();

        g1.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new SequentialCommandGroup(
                        new DropSampleCommand(),
                        new WaitCommand(400),
                        new ConditionalCommand(
                                new LiftDownCommand(),
                                new InstantCommand(),
                                () -> robot.data.scoring
                        )
                ));

        g1.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new LiftDownCommand());

        g1.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new LiftMidCommand());

        g1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new SampleEjectCommand());

        g1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new LiftUpCommand());

        g1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new SpecimenLiftStateCommand(SpecimenSubsystem.SpecimenLiftState.GRAB));

        g1.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(new ConditionalCommand(
                        new SpecimenLiftStateCommand(SpecimenSubsystem.SpecimenLiftState.LOW),
                        new SpecimenLiftStateCommand(SpecimenSubsystem.SpecimenLiftState.HIGH),
                        () -> robot.specimenSubsystem.getSpecimenLiftState() == SpecimenSubsystem.SpecimenLiftState.HIGH
                ));

        g1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(new SpecimenGrabCommand()
                        .andThen(new SpecimenLiftStateCommand(SpecimenSubsystem.SpecimenLiftState.HIGH)));

        g1.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new SpecimenDepositCommand().alongWith(new InstantCommand(() -> gamepad1.rumble(500)))
                        .andThen(new WaitCommand(200).andThen(new SpecimenLiftStateCommand(SpecimenSubsystem.SpecimenLiftState.GRAB))));

        g1.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
                .whenActive(() -> {
                    Globals.LIMITS = false;
                    CommandScheduler.getInstance().schedule(new LiftPowerCommand(-0.5));
                });
        g1.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
                .whenReleased(() -> {
                    Globals.LIMITS = true;
                    CommandScheduler.getInstance().schedule(new LiftPowerCommand(0));
                });

        g1.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)
                .whenPressed(new ExtensionResetCommand().alongWith(new InstantCommand(() -> gamepad1.rumble(500))));


        g1.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(() -> {
                    Constants.robotCentric = !Constants.robotCentric;
                    gamepad1.rumble(500);
                    if (Constants.robotCentric) gamepad1.setLedColor(1, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
                    else gamepad1.setLedColor(0, 0, 1, Gamepad.LED_DURATION_CONTINUOUS);
                });

        g1.getGamepadButton(GamepadKeys.Button.START)
                .whenPressed(() -> {
                    teleOpEnabled = true;
                    gamepad1.rumble(2000);
                });

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
                    -gamepad1.right_stick_x * (data.scoring || data.intaking ? 0.5 : 1),
                    Constants.robotCentric
            );

        }

        if (robot.data.intaking) {
            robot.intakeSubsystem.setExtensionPower(-gamepad1.right_stick_y);
        }







        if (gamepad1.touchpad) {
            robot.follower.setPose(new Pose());
            gamepad1.rumble(500);
            gamepad1.setLedColor(0, 1, 0, 1000);
        }


        boolean leftTrigger = gamepad1.left_trigger > .5;
        boolean rightTrigger = gamepad1.right_trigger > .5;

        if (leftTrigger && !lastLeftTrigger) {
            CommandScheduler.getInstance().schedule(new IntakePushOutCommand(Constants.extIntake));
        }

        if (rightTrigger && !lastRightTrigger) {
            CommandScheduler.getInstance().schedule(
                    new ConditionalCommand(
                            new IntakePullBackCommand().andThen(new SampleTransferCommand()),
                            new SampleTransferCommand(),
                            () -> robot.data.intaking
                    )
            );
        }

        lastLeftTrigger = leftTrigger;
        lastRightTrigger = rightTrigger;


//        boolean sampleIn = robot.sensorSubsystem.getIntakeDistance() < Constants.samplePickupTolerance;
//
//        if (!lastSampleIn && sampleIn) {
//            gamepad1.rumble(500);
//        }
//
//        lastSampleIn = sampleIn;
//
//
//        boolean sampleInTurnSpeed = robot.sensorSubsystem.getIntakeSpeed() < Constants.samplePickupTurnSpeedTolerance;
//
//        if (!lastSampleInTurnSpeed && sampleInTurnSpeed) {
//            CommandScheduler.getInstance().schedule(new IntakePullBackCommand());
//        }
//
//        lastSampleIn = sampleIn;



    }
}