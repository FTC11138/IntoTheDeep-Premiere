package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

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


    public enum CameraState {
        ON,
        OFF,
    }



    public CameraSubsystem(HardwareMap hardwareMap, String intakeCamera) {

        this.intakeCamera = hardwareMap.get(WebcamName.class, intakeCamera);

        cameraState = CameraState.OFF;
        if (Globals.IS_AUTO) {
            setupCamera();
            cameraState = CameraState.ON;
        }


        Robot.getInstance().subsystems.add(this);
    }

    public double getCameraAngleSample() {
        return cameraAngleSample;
    }

    public CameraState getCameraState() {
        return cameraState;
    }




    @Override
    public void updateData() {

        Robot.getInstance().data.cameraState = this.cameraState;
        Robot.getInstance().data.sampleAngle = this.cameraAngleSample;

    }

    @Override
    public void periodic() {

        if (cameraState == CameraState.ON) {
            cameraAngleSample = sampleAngleProcessor.getBlockAngle();
        }

    }


    private void setupCamera() {
        sampleAngleProcessor = new SampleAngleProcessor();
        aprilTagProcessor = new AprilTagProcessor.Builder().build();
        aprilTagProcessor.setDecimation(2);
        visionPortal = new VisionPortal.Builder()
                .addProcessor(aprilTagProcessor)
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