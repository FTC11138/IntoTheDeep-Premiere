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
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class SensorSubsystem extends RE_SubsystemBase {

    private RevColorSensorV3 intakeSensor;
    private RevColorSensorV3 outtakeSensor;
    private DcMotorEx intakeEncoder;

    private WebcamName intakeCamera;
    private SampleAlignmentProcessor sampleAlignmentProcessor;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;
    private CameraState cameraState;

    private Limelight3A limelight;


    private double intakeDistance;
    private NormalizedRGBA colorValues = new NormalizedRGBA();
    private Globals.COLORS color;

    private double intakeSpeed;

    private double outtakeDistance;

    private double cameraAngleSample;


    public enum CameraState {
        ON,
        OFF,
    }


    public SensorSubsystem(HardwareMap hardwareMap, String intakeSensor, String outtakeSensor, String intakeEncoder, String intakeCamera) {

        this.intakeSensor = hardwareMap.get(RevColorSensorV3.class, intakeSensor);
//        this.outtakeSensor = hardwareMap.get(RevColorSensorV3.class, outtakeSensor);
        this.intakeEncoder = hardwareMap.get(DcMotorEx.class, intakeEncoder);
        this.intakeCamera = hardwareMap.get(WebcamName.class, intakeCamera);

        cameraState = CameraState.OFF;
        if (Globals.IS_AUTO) {
            setupCamera();
            cameraState = CameraState.ON;
        }
//        stopCamera();


        Robot.getInstance().subsystems.add(this);
    }


    public double getIntakeDistance() {
        return intakeDistance;
    }

    public NormalizedRGBA getIntakeColorValues() {
        return colorValues;
    }

    public Globals.COLORS getIntakeColor() {
        return color;
    }

    public double getOuttakeDistance() {
        return outtakeDistance;
    }

    public double getIntakeSpeed() {
        return intakeSpeed;
    }

    public double getCameraAngleSample() {
        return cameraAngleSample;
    }

    public CameraState getCameraState() {
        return cameraState;
    }




    @Override
    public void updateData() {

        Robot.getInstance().data.intakeDistance = this.intakeDistance;
        Robot.getInstance().data.intakeColorValues = this.colorValues;
        Robot.getInstance().data.intakeColor = this.color;
        Robot.getInstance().data.intakeSpeed = this.intakeSpeed;
        Robot.getInstance().data.outtakeDistance = this.outtakeDistance;
        Robot.getInstance().data.cameraState = this.cameraState;
        Robot.getInstance().data.sampleAngle = this.cameraAngleSample;

    }

    @Override
    public void periodic() {
        intakeDistance = intakeSensor.getDistance(DistanceUnit.INCH);

        intakeSensor.setGain(Constants.colorSensorGain);

        colorValues = intakeSensor.getNormalizedColors();

        double red = colorValues.red;
        double green = colorValues.green;
        double blue = colorValues.blue;

        if (red > 0.4 && green > 0.4 && blue < 0.2) {
            color = Globals.COLORS.YELLOW;
        } else if (red > 0.5 && green < 0.3 && blue < 0.3) {
            color = Globals.COLORS.RED;
        } else if (blue > 0.5 && red < 0.3 && green < 0.3) {
            color = Globals.COLORS.BLUE;
        } else {
            color = Globals.COLORS.NONE;
        }


        intakeSpeed = Math.abs(intakeEncoder.getVelocity());

//        outtakeDistance = outtakeSensor.getDistance(DistanceUnit.INCH);


        if (cameraState == CameraState.ON) {
            cameraAngleSample = sampleAlignmentProcessor.getAngleToRotate();
        }

    }


    private void setupCamera() {
        sampleAlignmentProcessor = new SampleAlignmentProcessor();
        aprilTagProcessor = new AprilTagProcessor.Builder().build();
        aprilTagProcessor.setDecimation(2);
        visionPortal = new VisionPortal.Builder()
                .addProcessor(aprilTagProcessor)
                .addProcessor(sampleAlignmentProcessor)
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