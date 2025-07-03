package org.firstinspires.ftc.teamcode.hardware;

import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.subsystems.CameraSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystems.NewIntakeSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystems.SensorSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystems.SpecimenSubsystem;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.Globals;

public class RobotData {

    public long loopTime = System.currentTimeMillis();

    public double intakeDistance = 0;
    public NormalizedRGBA intakeColorValues = new NormalizedRGBA();
    public Globals.COLORS intakeColor = Globals.COLORS.NONE;
    public double outtakeDistance = 0;
    public double intakeSpeed = 0;
    public CameraSubsystem.CameraState cameraState = CameraSubsystem.CameraState.OFF;
    public double sampleAngle = 0;
    public double sampleDx = 0;
    public double sampleDy = 0;
    public double extAdjustDistance = 0;

    public int liftPosition1 = 0;
    public int liftPosition2 = 0;
    public double bucketPosition = 0;
    public DepositSubsystem.BucketState bucketState = DepositSubsystem.BucketState.NONE;

    public int extensionPosition = 0;
    public int extensionTarget = 0;
    public double armPosition1 = 0;
    public double armPosition2 = 0;
    public IntakeSubsystem.ArmState armState = IntakeSubsystem.ArmState.NONE;
    public IntakeSubsystem.IntakePushState intakePushState = IntakeSubsystem.IntakePushState.STORE;
    public IntakeSubsystem.IntakeState intakeState = IntakeSubsystem.IntakeState.STOP;


    public NewIntakeSubsystem.ArmState armStateNew = NewIntakeSubsystem.ArmState.NONE;
    public NewIntakeSubsystem.ClawState clawState = NewIntakeSubsystem.ClawState.OPEN;
    public NewIntakeSubsystem.WristState wristState = NewIntakeSubsystem.WristState.NONE;
    public NewIntakeSubsystem.RotateState rotateState = NewIntakeSubsystem.RotateState.NONE;
    public double clawAngle = 0;
    public double rotatePosition = 0;

    public int specimenLiftPosition = 0;
    public SpecimenSubsystem.SpecimenClawState specimenClawState = SpecimenSubsystem.SpecimenClawState.OPEN;
    public SpecimenSubsystem.SpecimenLiftState specimenLiftState = SpecimenSubsystem.SpecimenLiftState.DOWN;

    public Pose currentPose = new Pose(0,0, Math.toRadians(0));

    public boolean scoring = false;
    public boolean sampleLoaded = false;
    public boolean intaking = false;

    public void write(Telemetry telemetry) {

        telemetry.addData("LOOP TIME", System.currentTimeMillis() - loopTime);
        loopTime = System.currentTimeMillis();

        telemetry.addLine();

        telemetry.addData("POSE", this.currentPose);
        telemetry.addData("PATH TIMEOUT CONSTRAINT", FollowerConstants.pathEndTimeoutConstraint);
        telemetry.addData("BUSY", Robot.getInstance().follower.isBusy());
        telemetry.addLine(Constants.robotCentric ? "ROBOT CENTRIC" : "FIELD CENTRIC");

//        telemetry.addData("DRIVE VECTOR", Robot.getInstance().follower.getVelocity());

        telemetry.addLine();

        telemetry.addData("SCORING", this.scoring);
        telemetry.addData("INTAKING", this.intaking);
        telemetry.addData("SAMPLE LOADED", this.sampleLoaded);

        telemetry.addLine();

//        telemetry.addData("Intake Distance", this.intakeDistance);
//        telemetry.addLine("Intake Color Values");
//        telemetry.addData("     Intake R", this.intakeColorValues.red);
//        telemetry.addData("     Intake G", this.intakeColorValues.green);
//        telemetry.addData("     Intake B", this.intakeColorValues.blue);
//        telemetry.addData("Intake Color", this.intakeColor);
//        telemetry.addData("Intake Speed", this.intakeSpeed);
//        telemetry.addData("Outtake Distance", this.outtakeDistance);
        telemetry.addData("Camera State", this.cameraState);
        telemetry.addData("Camera Sample Angle", this.sampleAngle);
        telemetry.addData("Camera Sample Dx", this.sampleDx);
        telemetry.addData("Camera Sample Dy", this.sampleDy);
        telemetry.addData("Ext Adjust Distance", this.extAdjustDistance);

        telemetry.addLine();

        telemetry.addData("Lift Position 1", this.liftPosition1);
        telemetry.addData("Lift Position 2", this.liftPosition2);
        telemetry.addData("Wrist Position", this.bucketPosition);
        telemetry.addData("Bucket State", this.bucketState);

        telemetry.addLine();

        telemetry.addData("Extension Position", this.extensionPosition);
        telemetry.addData("Extension Target", this.extensionTarget);
        telemetry.addData("Arm Position 1", this.armPosition1);
        telemetry.addData("Arm Position 2", this.armPosition2);
        telemetry.addData("Arm State", this.armState);
        telemetry.addData("Intake Push State", this.intakePushState);
        telemetry.addData("Intake State", this.intakeState);

        telemetry.addLine();

        telemetry.addData("Arm State New", this.armStateNew);
        telemetry.addData("Claw State", this.clawState);
        telemetry.addData("Wrist State", this.wristState);
        telemetry.addData("Rotate State", this.rotateState);
        telemetry.addData("Claw Angle", this.clawAngle);
        telemetry.addData("Rotate Position", this.rotatePosition);

        telemetry.addLine();

        telemetry.addData("Specimen Lift Position", this.specimenLiftPosition);
        telemetry.addData("Specimen Claw State", this.specimenClawState);
        telemetry.addData("Specimen Lift State", this.specimenLiftState);

        telemetry.addLine();

        telemetry.update();
    }

    public void startScoring() {
        scoring = true;
    }

    public void stopScoring() {
        scoring = false;
    }

    public void startIntaking() {
        intaking = true;
    }

    public void stopIntaking() {
        intaking = false;
    }

    public void setSampleLoaded() {
        sampleLoaded = true;
    }

    public void setSampleUnloaded() {
        sampleLoaded = false;
    }

}
