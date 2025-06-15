package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.CustomPIDFCoefficients;
import com.pedropathing.util.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.advancedcommand.IntakePushOutCommand;
import org.firstinspires.ftc.teamcode.commands.advancedcommand.NewIntakePushOutCommand;
import org.firstinspires.ftc.teamcode.commands.advancedcommand.SampleExtendGrabCommand;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.util.Constants;

@Config
@TeleOp
public class SampleAngleOpMode extends OpMode {

    private final Robot robot = Robot.getInstance();

    @Override
    public void init() {
        robot.initialize(hardwareMap, telemetry);
        robot.follower.setPose(new Pose(0, 0, 0));
        robot.follower.startTeleopDrive();

        CommandScheduler.getInstance().schedule(new NewIntakePushOutCommand(600));

//        robot.startCamera();
    }

    @Override
    public void loop() {
        double angle = robot.cameraSubsystem.getCameraAngleSample();

        robot.periodic();
        robot.updateData();
        robot.write();
        CommandScheduler.getInstance().run();

        telemetry.addData("angle", angle);

        telemetry.update();
    }
}
