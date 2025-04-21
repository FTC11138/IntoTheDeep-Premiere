package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.CustomPIDFCoefficients;
import com.pedropathing.util.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.advancedcommand.IntakePushOutCommand;
import org.firstinspires.ftc.teamcode.commands.advancedcommand.SampleExtendGrabCommand;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.util.Constants;

@Config
@TeleOp
public class SampleAlignmentOpMode extends OpMode {

    private final Robot robot = Robot.getInstance();
    private PIDFController anglePID, movePID;

    private boolean sampleFound = false;

    @Override
    public void init() {
        robot.initialize(hardwareMap, telemetry);
        robot.follower.setPose(new Pose(0, 0, 0));
        anglePID = new PIDFController(new CustomPIDFCoefficients(Constants.kPAngle, Constants.kIAngle, Constants.kDAngle, Constants.kFAngle));
        robot.follower.startTeleopDrive();

        CommandScheduler.getInstance().schedule(new IntakePushOutCommand(600, true));

//        robot.startCamera();
    }

    @Override
    public void loop() {
        double angle = robot.sensorSubsystem.getCameraAngleSample();

        robot.periodic();
        robot.updateData();
        CommandScheduler.getInstance().run();
//        robot.write();

        anglePID.updatePosition(0);
        anglePID.setTargetPosition(angle);
        anglePID.setCoefficients(new CustomPIDFCoefficients(Constants.kPAngle, Constants.kIAngle, Constants.kDAngle, Constants.kFAngle));

        if (anglePID.getError() > 0) {
            sampleFound = true;
        }

        if (anglePID.getError() < 2 && sampleFound) {
            CommandScheduler.getInstance().schedule(new SampleExtendGrabCommand());
        }

        robot.follower.setTeleOpMovementVectors(0, 0, anglePID.runPIDF(), true);

        telemetry.addData("angle", angle);
        telemetry.addData("pid error", anglePID.getError());
        telemetry.addData("pid val", anglePID.runPIDF());

        telemetry.update();
    }
}
