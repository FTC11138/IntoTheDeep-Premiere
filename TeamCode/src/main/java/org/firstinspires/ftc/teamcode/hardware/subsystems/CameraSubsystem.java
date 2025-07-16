package org.firstinspires.ftc.teamcode.hardware.subsystems;

//import static org.firstinspires.ftc.teamcode.util.Constants.SampleBackwardFarCorrectionConstant;
import static org.firstinspires.ftc.teamcode.util.Constants.*;
//import static org.firstinspires.ftc.teamcode.util.Constants.SampleDyForwardFarCorrectionConstant;
//import static org.firstinspires.ftc.teamcode.util.Constants.SampleDyMiddleCorrectionConstant;
//import static org.firstinspires.ftc.teamcode.util.Constants.SampleForwardFarCorrectionConstant;
//import static org.firstinspires.ftc.teamcode.util.Constants.SampleMiddleCorrectionConstant;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.Globals;
import org.firstinspires.ftc.teamcode.util.wrappers.RE_SubsystemBase;
import org.firstinspires.ftc.teamcode.vision.SampleAlignmentProcessor;
import org.firstinspires.ftc.teamcode.vision.SampleAngleProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class CameraSubsystem extends RE_SubsystemBase {

    private WebcamName intakeCamera;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;
    private SampleAngleProcessor sampleAngleProcessor;
    private CameraState cameraState;

    private double cameraAngleSample;
    private double sampleDx;
    private double sampleDy;
    private double extAdjustDistance;
    private double robotAdjustDistance;


    public enum CameraState {
        ON,
        OFF,
    }



    public CameraSubsystem(HardwareMap hardwareMap, String intakeCamera) {

        this.intakeCamera = hardwareMap.get(WebcamName.class, intakeCamera);

        setupCamera();
        cameraState = CameraState.ON;


        Robot.getInstance().subsystems.add(this);
    }

    public double getCameraAngleSample() {
        return cameraAngleSample;
    }

    public CameraState getCameraState() {
        return cameraState;
    }


    public double getExtDistanceSample() {
        return extAdjustDistance;
    }

    public double getRobotDistanceSample() {
        return robotAdjustDistance;
    }


    @Override
    public void updateData() {

        Robot.getInstance().data.cameraState = this.cameraState;
        Robot.getInstance().data.sampleAngle = this.cameraAngleSample;
        Robot.getInstance().data.sampleDx = this.sampleDx;
        Robot.getInstance().data.sampleDy = this.sampleDy;
        Robot.getInstance().data.extAdjustDistance = this.extAdjustDistance;

    }

    @Override
    public void periodic() {

        if (cameraState == CameraState.ON) {
            cameraAngleSample = sampleAngleProcessor.getBlockAngle();
            sampleDx = sampleAngleProcessor.getDx();
            sampleDy = sampleAngleProcessor.getDy();

            double extMove;

            if (sampleDy < SampleDyMiddleCorrectionConstant) {
                extMove = Constants.sampleDyCorrectionConstantForward;
            } else {//if (sampleDy <= 0) {
                extMove = Constants.sampleDyCorrectionConstantBehind;
            }

            if (sampleDy > SampleDyBackwardFarCorrectionConstant) {
                extMove += Constants.sampleDyCorrectionConstantFarBehind;
            } else if (sampleDy < SampleDyForwardFarCorrectionConstant) {
                extMove += Constants.sampleDyCorrectionConstantFarForward       ;
            }

            extMove += (
                    Math.pow(sampleDy, 1) * Constants.sampleDyCorrectionMultiplier1 +
                    Math.pow(sampleDy, 2) * Constants.sampleDyCorrectionMultiplier2 +
                    Math.pow(sampleDy, 3) * Constants.sampleDyCorrectionMultiplier3 +
                    Math.pow(sampleDy, 4) * Constants.sampleDyCorrectionMultiplier4
            );


            if (Robot.getInstance().newIntakeSubsystem.wristState == NewIntakeSubsystem.WristState.PREGRABBACK) {
                extMove -= Constants.extGrabBackOffset;
            } else if (Robot.getInstance().newIntakeSubsystem.wristState == NewIntakeSubsystem.WristState.PREGRABFORWARD) {
                extMove += Constants.extGrabFowardOffset;
            }
            extAdjustDistance = extMove;

            sampleDx += 30;


            robotAdjustDistance = sampleDx * sampleDxCorrectionMultiplier;
        }

    }


    private void setupCamera() {
        sampleAngleProcessor = new SampleAngleProcessor();
        visionPortal = new VisionPortal.Builder()
                .addProcessor(sampleAngleProcessor)
                .setCamera(intakeCamera)
                .build();
    }

    public void updateCameraState(CameraState state) {
        cameraState = state;
        if (state == CameraState.ON) {
            startCamera();
        } else {
            stopCamera();
        }
    }

    private void startCamera() {
        visionPortal.resumeStreaming();
    }

    private void stopCamera() {
        visionPortal.stopStreaming();
    }

}