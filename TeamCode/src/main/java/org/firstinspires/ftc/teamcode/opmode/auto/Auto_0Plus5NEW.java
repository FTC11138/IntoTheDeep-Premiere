package org.firstinspires.ftc.teamcode.opmode.auto;

import static org.firstinspires.ftc.teamcode.opmode.auto.AutonomousMethods.buildCurve;
import static org.firstinspires.ftc.teamcode.opmode.auto.AutonomousMethods.buildPath;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commands.advancedcommand.DropSampleCommand;
import org.firstinspires.ftc.teamcode.commands.advancedcommand.LiftDownCommand;
import org.firstinspires.ftc.teamcode.commands.advancedcommand.LiftUpCommand;
import org.firstinspires.ftc.teamcode.commands.advancedcommand.NewIntakePullBackCommand;
import org.firstinspires.ftc.teamcode.commands.advancedcommand.NewIntakePushOutCommand;
import org.firstinspires.ftc.teamcode.commands.advancedcommand.NewSamplePickupCommand;
import org.firstinspires.ftc.teamcode.commands.advancedcommand.NewSamplePickupCommandAuto;
import org.firstinspires.ftc.teamcode.commands.advancedcommand.NewSampleTransferCommand;
import org.firstinspires.ftc.teamcode.commands.drivecommand.PathCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.BucketStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.RotateStateCommand;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystems.NewIntakeSubsystem;
import org.firstinspires.ftc.teamcode.util.Globals;
import org.firstinspires.ftc.teamcode.util.PoseConstants;

@Config
@Autonomous(name = "0+5", preselectTeleOp = "Solo")
public class Auto_0Plus5NEW extends LinearOpMode {

    // Start
    public static double start1X = 86;
    public static double start1Y = 9;
    public static double start1Degrees = 0;

    public static double pickup3and4Offset = 6.65;

    public static double pickup1and2Offset = 5.5;

    // Pickup 1
    public static double pickup1X = 114.5;
    public static double pickup1Y = 31;
    public static double pickup1Degrees = 90;
    public static int pickup1Ext = 420;

    // Drop 1
    public static double drop1X = 114.5;
    public static double drop1Y = 26;
    public static double drop1Degrees = 90;

    // Pickup 2
    public static double pickup2X = pickup1X;
    public static double pickup2Y = pickup1Y+pickup1and2Offset;
    public static double pickup2Degrees = pickup1Degrees;
    public static int pickup2Ext = 890;

    // Drop 2
    public static double drop2X = drop1X;
    public static double drop2Y = drop1Y;
    public static double drop2Degrees = drop1Degrees;

    // Pickup 3
    public static double pickup3X = pickup1X+pickup3and4Offset-0.08;
    public static double pickup3Y = pickup1Y;
    public static double pickup3Degrees = pickup1Degrees;
    public static int pickup3Ext = 390;

    // Drop 3
    public static double drop3X = 114.5+pickup3and4Offset;
    public static double drop3Y = 26;
    public static double drop3Degrees = 90;

    // Pickup 4
    public static double pickup4X = pickup3X+0.05;
    public static double pickup4Y = pickup1Y+pickup1and2Offset+0.05;
    public static double pickup4Degrees = pickup3Degrees;
    public static int pickup4Ext = 850;

    // Drop 4
    public static double drop4X = drop3X;
    public static double drop4Y = drop3Y;
    public static double drop4Degrees = drop3Degrees;

    // Score Pick 1
    public static double scorePick1X = 125;
    public static double scorePick1Y = 20;
    public static double scorePick1Degrees = 100;

    // Score 1
    public static double score1X = 130;
    public static double score1Y = 13.6;
    public static double score1Degrees = 125;

    // Score Pick 2
    public static double scorePick2X = 40.5;
    public static double scorePick2Y = 115;
    public static double scorePick2Degrees = 90;

    // Score 2
    public static double score2X = 132.1;
    public static double score2Y = 15.9;
    public static double score2Degrees = 100;

    // Score Pick 3
    public static double scorePick3X = 40;
    public static double scorePick3Y = 123;
    public static double scorePick3Degrees = 90;

    // Score 3
    public static double score3X = 16;
    public static double score3Y = 127;
    public static double score3Degrees = -45;


    // Pickup and Drop Paths
    public static Path pickup1Path, drop1Path;
    public static Path pickup2Path, drop2Path;
    public static Path pickup3Path, drop3Path;
    public static Path pickup4Path, drop4Path;

    // Scoring Paths
    public static Path scorePick1Path, score1Path;
    public static Path scorePick2Path, score2Path;
    public static Path scorePick3Path, score3Path;
    public Pose startPose = new Pose(start1X, start1Y, Math.toRadians(start1Degrees));

    public void buildPaths() {
        Pose pickup1Pose = new Pose(pickup1X, pickup1Y, Math.toRadians(pickup1Degrees));
        Pose pickup2Pose = new Pose(pickup2X, pickup2Y, Math.toRadians(pickup2Degrees));
        Pose pickup3Pose = new Pose(pickup3X, pickup3Y, Math.toRadians(pickup3Degrees));
        Pose pickup4Pose = new Pose(pickup4X, pickup4Y, Math.toRadians(pickup4Degrees));

        Pose drop1Pose = new Pose(drop1X, drop1Y, Math.toRadians(drop1Degrees));
        Pose drop2Pose = new Pose(drop2X, drop2Y, Math.toRadians(drop2Degrees));
        Pose drop3Pose = new Pose(drop3X, drop3Y, Math.toRadians(drop3Degrees));
        Pose drop4Pose = new Pose(drop4X, drop4Y, Math.toRadians(drop4Degrees));

        Pose scorePick1Pose = new Pose(scorePick1X, scorePick1Y, Math.toRadians(scorePick1Degrees));
        Pose scorePick2Pose = new Pose(scorePick2X, scorePick2Y, Math.toRadians(scorePick2Degrees));
        Pose scorePick3Pose = new Pose(scorePick3X, scorePick3Y, Math.toRadians(scorePick3Degrees));

        Pose score1Pose = new Pose(score1X, score1Y, Math.toRadians(score1Degrees));
        Pose score2Pose = new Pose(score2X, score2Y, Math.toRadians(score2Degrees));
        Pose score3Pose = new Pose(score3X, score3Y, Math.toRadians(score3Degrees));


        // Pickup/Drop Paths in correct order
        pickup1Path = buildPath(startPose, pickup1Pose);
        drop1Path   = buildPath(pickup1Pose, drop1Pose);

        pickup2Path = buildPath(drop1Pose, pickup2Pose);
        drop2Path   = buildPath(pickup2Pose, drop2Pose);

        pickup3Path = buildPath(drop2Pose, pickup3Pose);
        drop3Path   = buildPath(pickup3Pose, drop3Pose);

        pickup4Path = buildPath(drop3Pose, pickup4Pose);
//        drop4Path   = buildCurve(pickup4Pose, drop4Pose, new Point(75, 119));
        drop4Path   = buildPath(pickup4Pose, drop4Pose);

        // Scoring Paths
        scorePick1Path = buildPath(drop4Pose, scorePick1Pose);
        score1Path     = buildPath(scorePick1Pose, score1Pose);

        scorePick2Path = buildPath(score1Pose, scorePick2Pose);
        score2Path     = buildPath(scorePick2Pose, score2Pose);

        scorePick3Path = buildPath(score2Pose, scorePick3Pose);
        score3Path     = buildPath(scorePick3Pose, score3Pose);

    }

    @Override
    public void runOpMode() {
        Robot robot = Robot.getInstance();

        Globals.IS_AUTO = true;
        FollowerConstants.pathEndTimeoutConstraint = 500;

        robot.initialize(hardwareMap, telemetry);
        CommandScheduler.getInstance().reset();

        buildPaths();

        while (!isStarted()) {
            CommandScheduler.getInstance().run();
            robot.updateData();
            robot.periodic();
            robot.write();
        }

        robot.follower.setPose(startPose);
        robot.data.stopIntaking();
        robot.data.stopScoring();
//        robot.data.setSampleLoaded();

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new BucketStateCommand(DepositSubsystem.BucketState.INTAKE),

                        // === PICKUP 1 ===
                        new PathCommand(pickup1Path).alongWith(
                                new SequentialCommandGroup(
                                        new RotateStateCommand(NewIntakeSubsystem.RotateState.ANGLE_0),
                                        new NewIntakePushOutCommand(pickup1Ext),
                                        new RotateStateCommand(NewIntakeSubsystem.RotateState.ANGLE_0)
                                )
                        ),
                        new WaitCommand(1000),
                        new NewSamplePickupCommand(),

                        new PathCommand(drop1Path).alongWith(
                                new SequentialCommandGroup(
//                                        new RotateStateCommand(NewIntakeSubsystem.RotateState.ANGLE_90),
                                        new NewIntakePullBackCommand(),
                                        new NewSampleTransferCommand()
//                                        new RotateStateCommand(NewIntakeSubsystem.RotateState.ANGLE_90),
                                )
                        ),
                        new DropSampleCommand(),

                        // === PICKUP 2 ===
                        new PathCommand(pickup2Path).alongWith(
                                new SequentialCommandGroup(
                                        new RotateStateCommand(NewIntakeSubsystem.RotateState.ANGLE_0),
                                        new NewIntakePushOutCommand(pickup2Ext),
                                        new RotateStateCommand(NewIntakeSubsystem.RotateState.ANGLE_0)
                                )
                        ),
                        new NewSamplePickupCommand(),

                        new PathCommand(drop2Path).alongWith(
                                new SequentialCommandGroup(
                                        new RotateStateCommand(NewIntakeSubsystem.RotateState.ANGLE_0),
                                        new NewIntakePullBackCommand(),
                                        new NewSampleTransferCommand()
//                                        new RotateStateCommand(NewIntakeSubsystem.RotateState.ANGLE_0)
                                )
                        ),
//                        new WaitCommand(800),
//                        new NewSampleTransferCommand(),
                        new DropSampleCommand(),

                        // === PICKUP 3 ===
                        new PathCommand(pickup3Path).alongWith(
                                new SequentialCommandGroup(
                                        new RotateStateCommand(NewIntakeSubsystem.RotateState.ANGLE_0),
                                        new NewIntakePushOutCommand(pickup3Ext),
                                        new RotateStateCommand(NewIntakeSubsystem.RotateState.ANGLE_0)
                                )
                        ),
                        new NewSamplePickupCommand(),

                        new PathCommand(drop3Path).alongWith(
                                new SequentialCommandGroup(
                                        new RotateStateCommand(NewIntakeSubsystem.RotateState.ANGLE_0),
                                        new NewIntakePullBackCommand(),
                                        new NewSampleTransferCommand()
//                                        new RotateStateCommand(NewIntakeSubsystem.RotateState.ANGLE_0)
                                )
                        ),
//                        new WaitCommand(800),

                        new DropSampleCommand(),

                        // === PICKUP 4 ===
                        new PathCommand(pickup4Path).alongWith(
                                new SequentialCommandGroup(
                                        new RotateStateCommand(NewIntakeSubsystem.RotateState.ANGLE_0),
                                        new NewIntakePushOutCommand(pickup4Ext),
                                        new RotateStateCommand(NewIntakeSubsystem.RotateState.ANGLE_0)
                                )
                        ),
                        new NewSamplePickupCommand(),

                        new PathCommand(drop4Path).alongWith(
                                new SequentialCommandGroup(
                                        new RotateStateCommand(NewIntakeSubsystem.RotateState.ANGLE_0),
                                        new NewIntakePullBackCommand(),
//                                        new RotateStateCommand(NewIntakeSubsystem.RotateState.ANGLE_0)
                                        new NewSampleTransferCommand()
                                )
                        ),
//                        new WaitCommand(800),
//                        n
                        new DropSampleCommand()
                )


        );

        while (opModeIsActive() && !isStopRequested()) {
            CommandScheduler.getInstance().run();
            robot.updateData();
            robot.periodic();
            robot.write();
        }

    }

}