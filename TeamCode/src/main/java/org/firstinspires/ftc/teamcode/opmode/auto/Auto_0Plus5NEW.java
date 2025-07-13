package org.firstinspires.ftc.teamcode.opmode.auto;

import static org.firstinspires.ftc.teamcode.opmode.auto.AutonomousMethods.buildCurve;
import static org.firstinspires.ftc.teamcode.opmode.auto.AutonomousMethods.buildPath;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commands.advancedcommand.DropSampleCommand;
import org.firstinspires.ftc.teamcode.commands.advancedcommand.IntakePushOutCommand;
import org.firstinspires.ftc.teamcode.commands.advancedcommand.LiftDownCommand;
import org.firstinspires.ftc.teamcode.commands.advancedcommand.LiftUpCommand;
import org.firstinspires.ftc.teamcode.commands.advancedcommand.NewIntakePullBackCommand;
import org.firstinspires.ftc.teamcode.commands.advancedcommand.NewIntakePushOutCommand;
import org.firstinspires.ftc.teamcode.commands.advancedcommand.NewSamplePickupCommand;
import org.firstinspires.ftc.teamcode.commands.advancedcommand.NewSamplePickupCommandAuto;
import org.firstinspires.ftc.teamcode.commands.advancedcommand.NewSampleTransferCommand;
import org.firstinspires.ftc.teamcode.commands.advancedcommand.SpecimenDepositCommand;
import org.firstinspires.ftc.teamcode.commands.drivecommand.PathCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.BucketStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.RotateStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.SpecimenClawStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.SpecimenLiftStateCommand;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystems.NewIntakeSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystems.SpecimenSubsystem;
import org.firstinspires.ftc.teamcode.util.Globals;
import org.firstinspires.ftc.teamcode.util.PoseConstants;

@Config
@Autonomous(name = "0+5", preselectTeleOp = "Solo")
public class Auto_0Plus5NEW extends LinearOpMode {

    // Start
    public static double start1X = 87;
    public static double start1Y = 9;
    public static double start1Degrees = 90;

    public static double pickup3and4Offset = 6.65;

    public static double pickup1and2Offset = 5.5;

    // Pickup 1
    public static double pickup1X = 114.2;
    public static double pickup1Y = 23.5;
    public static double pickup1Degrees = 90;
    public static int pickup1Ext = 1000;

    // Drop 1
    public static double drop1X = 116;
    public static double drop1Y = 23.5;
    public static double drop1Degrees = 90;

    // Pickup 2
    public static double pickup2X = 114.2;
    public static double pickup2Y = 31;
    public static double pickup2Degrees = 90;
    public static int pickup2Ext = 1170;

    // Drop 2
    public static double drop2X = 116;
    public static double drop2Y = 22;
    public static double drop2Degrees = 90;

    // Pickup 3
    public static double pickup3X = 122.5;
    public static double pickup3Y = 23.5;
    public static double pickup3Degrees = 90;
    public static int pickup3Ext = 1000;

    // Drop 3
    public static double drop3X = 123;
    public static double drop3Y = 23.5;
    public static double drop3Degrees = 90;

    // Pickup 4
    public static double pickup4X = 124;
    public static double pickup4Y = 32;
    public static double pickup4Degrees = 90;
    public static int pickup4Ext = 1200;

    // Drop 4
    public static double drop4X = 123;
    public static double drop4Y = 23.5;
    public static double drop4Degrees = drop3Degrees;




    // Score Pick 1
    public static double scorePick1X = 116;
    public static double scorePick1Y = 8;
    public static double scorePick1Degrees = 0;

    // Score 1
    public static double score1X = 105;
    public static double score1Y = 64; //68
    public static double score1Degrees = -90;

    // Score Pick 2
    public static double scorePick2X = 112;
    public static double scorePick2Y = 10;
    public static double scorePick2Degrees = 0;

    // Score 2
    public static double score2X = 105;
    public static double score2Y = 71;
    public static double score2Degrees = -90;

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
    public static Path scorePrePick1Path, scorePick1Path, score1Path;
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

        Pose scorePrePick1Pose = new Pose(scorePick1X - 8, scorePick1Y, Math.toRadians(scorePick1Degrees));
        Pose scorePick1Pose = new Pose(scorePick1X, scorePick1Y, Math.toRadians(scorePick1Degrees));
        Pose scorePick2Pose = new Pose(scorePick2X, scorePick2Y, Math.toRadians(scorePick2Degrees));
        Pose scorePick3Pose = new Pose(scorePick3X, scorePick3Y, Math.toRadians(scorePick3Degrees));

        Pose score1Pose = new Pose(score1X, score1Y, Math.toRadians(score1Degrees));
        Pose score2Pose = new Pose(score2X, score2Y, Math.toRadians(score2Degrees));
        Pose score3Pose = new Pose(score3X, score3Y, Math.toRadians(score3Degrees));


        // Pickup/Drop Paths in correct order
        pickup1Path = buildPath(startPose, pickup1Pose);
        drop1Path   = buildPath(pickup1Pose, drop1Pose);

        pickup2Path = buildPath(pickup1Pose, pickup2Pose);
        drop2Path   = buildPath(pickup2Pose, drop2Pose);

        pickup3Path = buildPath(drop2Pose, pickup3Pose);
        drop3Path   = buildPath(pickup3Pose, drop3Pose);

        pickup4Path = buildPath(drop3Pose, pickup4Pose);
//        drop4Path   = buildCurve(pickup4Pose, drop4Pose, new Point(75, 119));
        drop4Path   = buildPath(pickup4Pose, drop4Pose);

        // Scoring Paths
        scorePrePick1Path = buildPath(drop2Pose, scorePrePick1Pose);
        scorePick1Path = buildPath(scorePrePick1Pose, scorePick1Pose);
        score1Path     = buildCurve(scorePick1Pose, score1Pose, new Point(100, 29), new Point(125, 71), 0.5);

        scorePick2Path = buildCurve(score1Pose, scorePrePick1Pose, new Point(127, 66), new Point(81, 3.5));
        scorePick2Path = buildPath(scorePrePick1Pose, scorePick1Pose);
        score2Path     = buildCurve(scorePick1Pose, score1Pose, new Point(100, 29), new Point(125, 71), 0.5);

        scorePick3Path = buildCurve(score1Pose, scorePrePick1Pose, new Point(127, 66), new Point(81, 3.5));
        scorePick3Path = buildPath(scorePrePick1Pose, scorePick1Pose);
        score3Path     = buildCurve(scorePick1Pose, score1Pose, new Point(100, 29), new Point(125, 71), 0.5);

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
//                                new WaitCommand(100000)
                                        new NewIntakePushOutCommand(pickup1Ext),
                                        new RotateStateCommand(NewIntakeSubsystem.RotateState.ANGLE_0)
                                )
                        ),
                        new WaitCommand(200),
                        new NewSamplePickupCommand(),
                        new NewIntakePullBackCommand(),
                        new WaitCommand(450),
                        new NewSampleTransferCommand(),
                        new NewIntakePushOutCommand(pickup2Ext).alongWith(new DropSampleCommand()),
                        new RotateStateCommand(NewIntakeSubsystem.RotateState.ANGLE_0),

                        // === PICKUP 2 ===
                        new PathCommand(pickup2Path),
                        new NewSamplePickupCommand(),

                        new PathCommand(drop2Path).alongWith(
                                new SequentialCommandGroup(
                                        new NewIntakePullBackCommand(),
                                        new WaitCommand(600),
                                        new NewSampleTransferCommand(),
                                        new DropSampleCommand()//
//                                        new NewIntakePushOutCommand(pickup3Ext).alongWith(new DropSampleCommand()),
//                                        new RotateStateCommand(NewIntakeSubsystem.RotateState.ANGLE_0)
                                )
                        ),
//
//                        new WaitCommand(500),
//
//                        new NewSamplePickupCommand(),
//                        new NewIntakePullBackCommand(),
//                        new WaitCommand(450),
//                        new NewSampleTransferCommand(),

//
//                        new WaitCommand(500),


                        new PathCommand(scorePrePick1Path).alongWith(new SpecimenClawStateCommand(SpecimenSubsystem.SpecimenClawState.OPEN)),
                        new PathCommand(scorePick1Path),
                        new SpecimenClawStateCommand(SpecimenSubsystem.SpecimenClawState.CLOSED),
                        new WaitCommand(300),
                        new SpecimenLiftStateCommand(SpecimenSubsystem.SpecimenLiftState.HIGH),
                        new WaitCommand(500),
                        new PathCommand(score1Path),
                        new SpecimenDepositCommand(),

                        new PathCommand(scorePick2Path).alongWith(new SpecimenClawStateCommand(SpecimenSubsystem.SpecimenClawState.OPEN)),
                        new PathCommand(scorePick2Path),
                        new SpecimenClawStateCommand(SpecimenSubsystem.SpecimenClawState.CLOSED),
                        new WaitCommand(300),
                        new SpecimenLiftStateCommand(SpecimenSubsystem.SpecimenLiftState.HIGH),
                        new WaitCommand(500),
                        new PathCommand(score2Path),
                        new SpecimenDepositCommand(),

                        new PathCommand(scorePick3Path).alongWith(new SpecimenClawStateCommand(SpecimenSubsystem.SpecimenClawState.OPEN)),
                        new PathCommand(scorePick3Path),
                        new SpecimenClawStateCommand(SpecimenSubsystem.SpecimenClawState.CLOSED),
                        new WaitCommand(300),
                        new SpecimenLiftStateCommand(SpecimenSubsystem.SpecimenLiftState.HIGH),
                        new WaitCommand(500),
                        new PathCommand(score3Path),
                        new SpecimenDepositCommand()

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