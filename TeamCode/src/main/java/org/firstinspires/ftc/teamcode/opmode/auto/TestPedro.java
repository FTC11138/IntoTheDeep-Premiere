package org.firstinspires.ftc.teamcode.opmode.auto;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.util.Globals;
import org.firstinspires.ftc.teamcode.util.PoseConstants;

@Disabled
@Autonomous(name = "Pedro Test", preselectTeleOp = "Solo")
public class TestPedro extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = Robot.getInstance();

        Globals.IS_AUTO = true;

        robot.initialize(hardwareMap, telemetry);
        CommandScheduler.getInstance().reset();

        waitForStart();

        robot.follower.setPose(PoseConstants.Start.redBasket);

        robot.data.stopIntaking();
        robot.data.stopScoring();
        robot.data.setSampleLoaded();

        Globals.LIMITS = false;

        robot.depositSubsystem.setLiftPosition(1, -1200);

        while (opModeIsActive() && !isStopRequested()) {
            CommandScheduler.getInstance().run();
            robot.updateData();
            robot.periodic();
            robot.write();
        }

    }

}
