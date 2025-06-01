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
import org.firstinspires.ftc.teamcode.commands.advancedcommand.SampleTransferCommand;
import org.firstinspires.ftc.teamcode.commands.drivecommand.PathCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.ArmStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.BucketStateCommand;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.util.Globals;
import org.firstinspires.ftc.teamcode.util.PoseConstants;

@Config
@Autonomous(name = "5+0", preselectTeleOp = "Solo")
public class Auto_5Plus0 extends LinearOpMode {

    public static double score1X = 12.5; //also scorepos
    public static double score1Y = 126; //also scorepos
    public static double score1Degrees = -45;

    public static double score2X = 16;
    public static double score2Y = 127;
    public static double score2Degrees = -45;

    public static double score3X = 16;
    public static double score3Y = 127;
    public static double score3Degrees = -45;

    public static double score4X = 18;
    public static double score4Y = 129;
    public static double score4Degrees = -45;


    public static double sample1x = 32;
    public static double sample1y = 109;
    public static double sample1degrees = 60;
    public static int sample1ext = 750;

    public static double sample2x = 40.5;
    public static double sample2y = 115;
    public static double sample2degrees = 90;
    public static int sample2ext = 0;

    public static double sample3x = 40;
    public static double sample3y = 120; //123
    public static double sample3degrees = 90;
    public static int sample3ext = 0;

    public static double sample4x = 45;
    public static double sample4y = 72;
    public static double sample4degrees = 0;
//    public static int sample4ext = 250;

    public static double test1X = 15.5;
    public static double test1Y = 104;
    public static double test1Deg = 0;

    public static double test2X = 46;
    public static double test2Y = 120;
    public static double test2Deg = 0;

    public static double test3X = 97;
    public static double test3Y = 120;
    public static double test3Deg = 0;


    public static Path preload;
    public static Path sample1Path, sample1ScorePath;
    public static Path sample2Path, sample2ScorePath;
    public static Path sample3Path, sample3ScorePath;
    public static Path sample4Path, sample4ScorePath;
    public static Path test1Path, test2Path, test3Path, test4Path;

    public void buildPaths() {
//        Pose startPose = PoseConstants.Start.redBasket;

        Pose scorePose = new Pose(score1X, score1Y, Math.toRadians(score1Degrees));
        Pose score1 = new Pose(score1X, score1Y, Math.toRadians(score1Degrees));
        Pose score2 = new Pose(score2X, score2Y, Math.toRadians(score2Degrees));
        Pose score3 = new Pose(score3X, score3Y, Math.toRadians(score3Degrees));
        Pose score4 = new Pose(score4X, score4Y, Math.toRadians(score4Degrees));

        Pose sample1Pose = new Pose(sample1x, sample1y, Math.toRadians(sample1degrees));
        Pose sample2Pose = new Pose(sample2x, sample2y, Math.toRadians(sample2degrees));
        Pose sample3Pose = new Pose(sample3x, sample3y, Math.toRadians(sample3degrees));
        Pose sample4Pose = new Pose(sample4x, sample4y, Math.toRadians(sample4degrees));
        Pose test1 = new Pose(test1X, test1Y, Math.toRadians(270));
        Pose test2 = new Pose(test2X, test2Y, Math.toRadians(270));
        Pose test3 = new Pose(test3X, test3Y, Math.toRadians(270));
        Pose startPose = test1;


        test1Path = buildPath(test1, test2);
        test2Path = buildPath(test2, test3);
        test3Path = buildPath(test3, test2);
        test4Path = buildPath(test2, test1);


        // Preload path
        preload = buildPath(startPose, score1);

        // Sample 1 paths
        sample1Path = buildPath(scorePose, sample1Pose);
        sample1ScorePath = buildPath(sample1Pose, score2);

        // Sample 2 paths
        sample2Path = buildPath(scorePose, sample2Pose);
        sample2ScorePath = buildPath(sample2Pose, score3);

        // Sample 3 paths
        sample3Path = buildPath(scorePose, sample3Pose);
        sample3ScorePath = buildPath(sample3Pose, score4);

        sample4Path = buildPath(scorePose, sample4Pose);
        sample4ScorePath = buildPath(sample4Pose, score4);

//        test1Path = buildPath(scorePose, sample4Pose);
//        test2Path = buildPath(sample4Pose, score4);
//        test3Path = buildPath(sample4Pose, score4);
//        test4Path = buildPath(sample4Pose, score4);


        // Sample 4 paths
//        sample4Path = buildCurve(scorePose, sample4Pose, new Point(75, 119));
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

        robot.follower.setPose(PoseConstants.Start.redBasket);
        robot.data.stopIntaking();
        robot.data.stopScoring();
        robot.data.setSampleLoaded();

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(

                        new PathCommand(test1Path),
//                                .alongWith(
//                        new WaitCommand(5000)),
                        new PathCommand(test2Path),


//                                .alongWith(
//                                        new WaitCommand(5000)),
                        new PathCommand(test3Path),
//                                .alongWith(
//                                        new WaitCommand(5000)),
                        new PathCommand(test4Path)
                                .alongWith(
                                        new WaitCommand(500000000)),

//                        new ArmStateCommand(IntakeSubsystem.ArmState.UP),
//                        new BucketStateCommand(DepositSubsystem.BucketState.INTAKE),
                        new PathCommand(preload),
//                                .alongWith(new LiftUpCommand()), new DropSampleCommand(),

                        new PathCommand(sample1Path),
//                                .alongWith(new SequentialCommandGroup(
//                                        new LiftDownCommand(),
//                                        new IntakePushOutCommand(0)
//                                )),

//                        new ExtensionJumpCommand(1, sample1ext),
//                        new WaitCommand(500),
                        new PathCommand(sample1ScorePath),
//                                                        .alongWith(
//                                        new SequentialCommandGroup(
//                                                new IntakePullBackCommand(),
//                                                new WaitCommand(500),
//                                                new SampleTransferCommand(),
//                                                new WaitCommand(200),
//                                                new LiftUpCommand(),
//                                                new WaitCommand(700)
//                                        )
//                                ),
//                        new DropSampleCommand(),

                        new PathCommand(sample2Path),
//                                .alongWith(new SequentialCommandGroup(
//                                        new LiftDownCommand(),
//                                        new IntakePushOutCommand(0)
//                                )),

//                        new ExtensionJumpCommand(1),
//                        new WaitCommand(500),
//                        new PathCommand(sample2ScorePath),
//                                .alongWith(
//                                        new SequentialCommandGroup(
//                                                new IntakePullBackCommand(),
//                                                new WaitCommand(500),
//                                                new SampleTransferCommand(),
//                                                new WaitCommand(200),
//                                                new LiftUpCommand(),
//                                                new WaitCommand(700)
//                                        )
//                                ),
//                        new DropSampleCommand(),

                        new PathCommand(sample3Path),
//                                .alongWith(new SequentialCommandGroup(
//                                        new LiftDownCommand(),
//                                        new IntakePushOutCommand(0)
//                                )),

//                        new ExtensionJumpCommand(1),
//                        new WaitCommand(500),
                        new PathCommand(sample3ScorePath),
                        new PathCommand(sample4Path),
                        new PathCommand(sample4ScorePath)

//                                .alongWith(
//                                        new SequentialCommandGroup(
//                                                new IntakePullBackCommand(),
//                                                new WaitCommand(500),
//                                                new SampleTransferCommand(),
//                                                new WaitCommand(200),
//                                                new LiftUpCommand(),
//                                                new WaitCommand(700)
//                                        )
//                                ),
//                        new DropSampleCommand(),
//
//                        new LiftDownCommand()

                )
        );

        while (opModeIsActive() && !isStopRequested()) {
            CommandScheduler.getInstance().run();
            telemetry.addData("Heading", robot.follower.getPose().getHeading());
            robot.updateData();
            robot.periodic();
            robot.write();
        }

    }

}
