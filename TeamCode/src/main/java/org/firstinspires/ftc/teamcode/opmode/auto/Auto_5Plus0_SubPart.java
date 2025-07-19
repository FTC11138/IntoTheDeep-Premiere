package org.firstinspires.ftc.teamcode.opmode.auto;

import static org.firstinspires.ftc.teamcode.hardware.subsystems.NewIntakeSubsystem.RotateState.SAMPLE3PICKUP;
import static org.firstinspires.ftc.teamcode.opmode.auto.AutonomousMethods.buildCurve;
import static org.firstinspires.ftc.teamcode.opmode.auto.AutonomousMethods.buildPath;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
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
import org.firstinspires.ftc.teamcode.commands.advancedcommand.NewSamplePickupCommandAuto;
import org.firstinspires.ftc.teamcode.commands.advancedcommand.NewSamplePickupCommandSub;
import org.firstinspires.ftc.teamcode.commands.advancedcommand.NewSampleTransferCommand;
import org.firstinspires.ftc.teamcode.commands.drivecommand.PathCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.BucketStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.RotateStateCommand;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.util.Globals;

@Config
@Autonomous(name = "Sub test", preselectTeleOp = "Solo")
public class Auto_5Plus0_SubPart extends LinearOpMode {

    ///  -20.6, -8.4, 290.79
    /// 0,0,0

    //Pose(111.5, 7.5, Math.toRadians(180));

    public static double startX = 32.5;
    public static double startY = 7.5;
    public static double startDegrees = 90;

    public static double score1X = 15;
    public static double score1Y = 15;
    public static double score1Degrees = 45;

    public static double score2X = score1X;
    public static double score2Y = score1Y;
    public static double score2Degrees = score1Degrees;

    public static double score3X = score1X;
    public static double score3Y = score1Y;
    public static double score3Degrees = score1Degrees;

    public static double score4X = score1X;
    public static double score4Y = score1Y;
    public static double score4Degrees = score1Degrees;

    public static double score5X = score1X;
    public static double score5Y = score1Y;
    public static double score5Degrees = score1Degrees;


    public static double sample1x = 20;
    public static double sample1y = 20;
    public static double sample1degrees = 90;
    public static int sample1ext = 1050;

    public static double sample2x = 13;
    public static double sample2y = 17;
    public static double sample2degrees = 90;
    public static int sample2ext = 1150;

    public static double sample3x = 13.9;
    public static double sample3y = 20.4;
    public static double sample3degrees = 116;
    public static int sample3ext = 1200;

    public static double sample4x = 72.4;
    public static double sample4y = 42;
    public static double sample4degrees = 90;
    public static int submersibleExt = 1200;


    public static Path preload;
    public static Path sample1Path, sample1ScorePath;
    public static Path sample2Path, sample2ScorePath;
    public static Path sample3Path, sample3ScorePath;
    public static Path submersiblePath, submersibleScorePath;

    public void buildPaths() {
        Pose startPose = new Pose(startX, startY, Math.toRadians(startDegrees)); // Pose(111.5, 7.5, Math.toRadians(180));
        Pose scorePose = new Pose(score1X, score1Y, Math.toRadians(score1Degrees));
        Pose score1 = new Pose(score1X, score1Y, Math.toRadians(score1Degrees));
        Pose score2 = new Pose(score2X, score2Y, Math.toRadians(score2Degrees));
        Pose score3 = new Pose(score3X, score3Y, Math.toRadians(score3Degrees));
        Pose score4 = new Pose(score4X, score4Y, Math.toRadians(score4Degrees));
        Pose score5 = new Pose(score5X, score5Y, Math.toRadians(score5Degrees));

        Pose sample1Pose = new Pose(sample1x, sample1y, Math.toRadians(sample1degrees));
        Pose sample2Pose = new Pose(sample2x, sample2y, Math.toRadians(sample2degrees));
        Pose sample3Pose = new Pose(sample3x, sample3y, Math.toRadians(sample3degrees));
        Pose sample4Pose = new Pose(sample4x, sample4y, Math.toRadians(sample4degrees));

        // Preload path
        preload = buildPath(startPose, score1);

        // Sample 1 paths
        sample1Path = buildPath(score1, sample1Pose);
        sample1ScorePath = buildPath(sample1Pose, score2);

        // Sample 2 paths
        sample2Path = buildPath(score2, sample2Pose);
        sample2ScorePath = buildPath(sample2Pose, score3);

        // Sample 3 paths
        sample3Path = buildPath(score3, sample3Pose);
        sample3ScorePath = buildPath(sample3Pose, score4);

        // Sample 4 paths
        submersiblePath = buildCurve(startPose, sample4Pose, new Point(70.5, 25.5));
        submersibleScorePath = buildCurve(sample4Pose, score5, new Point(70.5, 25.5));
    }

    @Override
    public void runOpMode() {
        Robot robot = Robot.getInstance();

        Globals.IS_AUTO = true;

        robot.initialize(hardwareMap, telemetry);
        CommandScheduler.getInstance().reset();

        buildPaths();

        while (!isStarted()) {
            CommandScheduler.getInstance().run();
            robot.updateData();
            robot.periodic();
            robot.write();
        }

        robot.follower.setPose(new Pose(startX, startY, Math.toRadians(startDegrees)));
        robot.data.stopIntaking();
        robot.data.stopScoring();
        robot.data.setSampleLoaded();

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new BucketStateCommand(DepositSubsystem.BucketState.INTAKE),
//                        new NewIntakePushOutCommand(sample1ext),
//
//                        new PathCommand(preload)
//                            .alongWith(new SequentialCommandGroup(
//                                    new LiftUpCommand()
//                            )),
//                        new DropSampleCommand(),
//                        new PathCommand(sample1Path).alongWith(new LiftDownCommand()),
//                        new WaitCommand(100),
//                        new NewSamplePickupCommandAuto(),
//                        new PathCommand(sample1ScorePath).alongWith(
//                                new SequentialCommandGroup(
//                                        new NewIntakePullBackCommand(),
//                                        new WaitCommand(500),
//                                        new NewSampleTransferCommand(),
//                                        new LiftUpCommand(),
//                                        new NewIntakePushOutCommand(sample2ext),
//                                        new WaitCommand(500)
//                                )
//                        ),
//                        new DropSampleCommand(),
//
//                        new PathCommand(sample2Path).alongWith(new LiftDownCommand()),
//                        new WaitCommand(100),
//                        new NewSamplePickupCommandAuto(),
//                        new PathCommand(sample2ScorePath).alongWith(
//                                new SequentialCommandGroup(
//                                        new NewIntakePullBackCommand(),
//                                        new WaitCommand(500),
//                                        new NewSampleTransferCommand(),
//                                        new LiftUpCommand(),
//                                        new NewIntakePushOutCommand(sample3ext),
//                                        new WaitCommand(500)
//                                )
//                        ),
//                        new DropSampleCommand(),
//
//                        new PathCommand(sample3Path).alongWith(new LiftDownCommand()),
//                        new WaitCommand(100),
//                        new NewSamplePickupCommandAuto(),
//                        new RotateStateCommand(SAMPLE3PICKUP),
//                        new PathCommand(sample3ScorePath).alongWith(
//                                new SequentialCommandGroup(
//                                        new NewIntakePullBackCommand(),
//                                        new WaitCommand(500),
//                                        new NewSampleTransferCommand(),
//                                        new LiftUpCommand(),
//                                        new WaitCommand(500)
//                                )
//                        ),
//                        new DropSampleCommand(),
//                        new LiftDownCommand(),
                        new NewIntakePushOutCommand(submersibleExt),
                        new WaitCommand(1000),
                        new NewSamplePickupCommandSub(),
                        new SequentialCommandGroup(
                                new NewIntakePullBackCommand(),
                                new WaitCommand(500),
                                new NewSampleTransferCommand()
                        )
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