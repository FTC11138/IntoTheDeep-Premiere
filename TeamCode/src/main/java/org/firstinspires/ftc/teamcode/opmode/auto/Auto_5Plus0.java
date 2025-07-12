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
import org.firstinspires.ftc.teamcode.commands.advancedcommand.ExtensionJumpCommand;
import org.firstinspires.ftc.teamcode.commands.advancedcommand.IntakePullBackCommand;
import org.firstinspires.ftc.teamcode.commands.advancedcommand.IntakePushOutCommand;
import org.firstinspires.ftc.teamcode.commands.advancedcommand.LiftDownCommand;
import org.firstinspires.ftc.teamcode.commands.advancedcommand.LiftUpCommand;
import org.firstinspires.ftc.teamcode.commands.advancedcommand.NewIntakeAutoPushOutCommand;
import org.firstinspires.ftc.teamcode.commands.advancedcommand.NewIntakePullBackCommand;
import org.firstinspires.ftc.teamcode.commands.advancedcommand.NewIntakePushOutCommand;
import org.firstinspires.ftc.teamcode.commands.advancedcommand.NewSampleAlignCommand;
import org.firstinspires.ftc.teamcode.commands.advancedcommand.NewSamplePickupCommand;
import org.firstinspires.ftc.teamcode.commands.advancedcommand.NewSampleTransferCommand;
import org.firstinspires.ftc.teamcode.commands.advancedcommand.SampleTransferCommand;
import org.firstinspires.ftc.teamcode.commands.drivecommand.PathCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.ArmStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.BucketStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.ClawStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.RotateStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.WristPositionCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.WristStateCommand;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystems.NewIntakeSubsystem;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.Globals;
import org.firstinspires.ftc.teamcode.util.PoseConstants;

@Config
@Autonomous(name = "5+0", preselectTeleOp = "Solo")
public class Auto_5Plus0 extends LinearOpMode {

    ///  -20.6, -8.4, 290.79
    /// 0,0,0

    //Pose(111.5, 7.5, Math.toRadians(180));

    public static double score1X = 130;
    public static double score1Y = 13.6;
    public static double score1Degrees = 125;

    public static double score2X = 130;
    public static double score2Y = 13.6;
    public static double score2Degrees = 125;

    public static double score3X = 130;
    public static double score3Y = 13.6;
    public static double score3Degrees = 125;

    public static double score4X = 130;
    public static double score4Y = 13.6;
    public static double score4Degrees = 130;


    public static double sample1x = 125;
    public static double sample1y = 18;
    public static double sample1degrees = 94;
    public static int sample1ext = 1300;

    public static double sample2x = 131;
    public static double sample2y = 19;
    public static double sample2degrees = 90;
    public static int sample2ext = 1050;

    public static double sample3x = 132;
    public static double sample3y = 20.7;
    public static double sample3degrees = 68;
    public static int sample3ext = 1050;

    public static double sample4x = 72;
    public static double sample4y = 100;
    public static double sample4degrees = -90;
    public static int sample4ext = 250;


    public static Path preload;
    public static Path sample1Path, sample1ScorePath;
    public static Path sample2Path, sample2ScorePath;
    public static Path sample3Path, sample3ScorePath;
    public static Path sample4Path, sample4ScorePath;

    public void buildPaths() {
        Pose startPose = PoseConstants.Start.blueBasket; // Pose(111.5, 7.5, Math.toRadians(180));
        Pose scorePose = new Pose(score1X, score1Y, Math.toRadians(score1Degrees));
        Pose score1 = new Pose(score1X, score1Y, Math.toRadians(score1Degrees));
        Pose score2 = new Pose(score2X, score2Y, Math.toRadians(score2Degrees));
        Pose score3 = new Pose(score3X, score3Y, Math.toRadians(score3Degrees));
        Pose score4 = new Pose(score4X, score4Y, Math.toRadians(score4Degrees));

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
        sample2Path = buildPath(scorePose, sample2Pose);
        sample2ScorePath = buildPath(sample2Pose, score3);

        // Sample 3 paths
        sample3Path = buildPath(scorePose, sample3Pose);
        sample3ScorePath = buildPath(sample3Pose, score4);

        // Sample 4 paths
        sample4Path = buildCurve(scorePose, sample4Pose, new Point(75, 119));
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

        robot.follower.setPose(PoseConstants.Start.blueBasket);
        robot.data.stopIntaking();
        robot.data.stopScoring();
        robot.data.setSampleLoaded();

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new BucketStateCommand(DepositSubsystem.BucketState.INTAKE),

                        new PathCommand(preload)
                            .alongWith(new SequentialCommandGroup(
                                    new LiftUpCommand(),
                                    new NewIntakePushOutCommand(sample1ext)
                            )),
                        new DropSampleCommand(),
                        new PathCommand(sample1Path).alongWith(new LiftDownCommand()),
                        new NewSamplePickupCommand(),
                        new PathCommand(sample1ScorePath).alongWith(
                                new SequentialCommandGroup(
                                        new NewIntakePullBackCommand(),
                                        new WaitCommand(500),
                                        new NewSampleTransferCommand(),
                                        new LiftUpCommand(),
                                        new NewIntakePushOutCommand(sample2ext),
                                        new WaitCommand(500)
                                )
                        ),
                        new DropSampleCommand(),

                        new PathCommand(sample2Path).alongWith(new LiftDownCommand()),
                        new NewSamplePickupCommand(),
                        new PathCommand(sample2ScorePath).alongWith(
                                new SequentialCommandGroup(
                                        new NewIntakePullBackCommand(),
                                        new WaitCommand(500),
                                        new NewSampleTransferCommand(),
                                        new LiftUpCommand(),
                                        new NewIntakePushOutCommand(sample3ext),
                                        new WaitCommand(500)
                                )
                        ),
                        new DropSampleCommand(),

                        new PathCommand(sample3Path).alongWith(new LiftDownCommand()),
                        new NewSamplePickupCommand(),
                        new PathCommand(sample3ScorePath).alongWith(
                                new SequentialCommandGroup(
                                        new NewIntakePullBackCommand(),
                                        new WaitCommand(500),
                                        new NewSampleTransferCommand(),
                                        new LiftUpCommand(),
                                        new WaitCommand(500)
                                )
                        ),
                        new DropSampleCommand(),
                        new LiftDownCommand()
//                        new SequentialCommandGroup(
//                        new WaitCommand(300),
//                        new NewSamplePickupCommand(),
//                        new WaitCommand(300),
//                        new RotateStateCommand(NewIntakeSubsystem.RotateState.HORIZONTAL),
//                        new NewIntakePullBackCommand(),
//                        new NewSampleTransferCommand(),
//                        ),
//
//                        new WaitCommand(1500),
//                        new PathCommand(sample1ScorePath)
//                                .alongWith(
//                                        new SequentialCommandGroup(
//                                                new LiftUpCommand(),
//                                                new WaitCommand(700)
//                                        )
//                                ),
//                        new DropSampleCommand(),
//                        new LiftDownCommand()

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