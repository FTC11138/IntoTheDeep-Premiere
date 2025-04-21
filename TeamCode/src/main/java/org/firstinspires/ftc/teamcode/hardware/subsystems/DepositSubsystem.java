package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.wrappers.RE_DcMotorEx;
import org.firstinspires.ftc.teamcode.util.wrappers.RE_DcMotorExParams;
import org.firstinspires.ftc.teamcode.util.wrappers.RE_SubsystemBase;

public class DepositSubsystem extends RE_SubsystemBase {

    private RE_DcMotorEx lift1, lift2;
    private final RE_DcMotorExParams liftParams = new RE_DcMotorExParams(
            Constants.liftMin, Constants.liftMax, Constants.liftSlow,
            1, 1, Constants.liftUpRatio, Constants.liftDownRatio, Constants.liftSlowRatio
    );

    private final RE_DcMotorExParams lift2Params = new RE_DcMotorExParams(
            Constants.liftMin, Constants.liftMax, Constants.liftSlow,
            1, 1, Constants.liftUpRatio, Constants.liftDownRatio, Constants.liftSlowRatio
    );

    private final Servo bucket;

    private BucketState bucketState;

    public enum BucketState {
        DROP,
        INTAKE,
        ASCENT,
        NONE
    }


    public DepositSubsystem(HardwareMap hardwareMap, String lift1, String lift2, String wrist) {
        this.lift1 = new RE_DcMotorEx(hardwareMap.get(DcMotorEx.class, lift1), liftParams);
        this.lift2 = new RE_DcMotorEx(hardwareMap.get(DcMotorEx.class, lift2), lift2Params);
        this.lift2.motor.setDirection(DcMotorEx.Direction.REVERSE);
        this.bucket = hardwareMap.get(Servo.class, wrist);
        this.bucketState = BucketState.INTAKE;

        Robot.getInstance().subsystems.add(this);
    }

    @Override
    public void updateData() {
        Robot.getInstance().data.liftPosition1 = this.lift1.getPosition();
        Robot.getInstance().data.liftPosition2 = this.lift2.getPosition();
        Robot.getInstance().data.bucketState = bucketState;
        Robot.getInstance().data.bucketPosition = bucket.getPosition();
    }

    public void updateBucketState(BucketState state) {
        this.bucketState = state;
        this.bucket.setPosition(getBucketStatePosition(state));
    }

    private double getBucketStatePosition(BucketState state) {
        switch (state) {
            case DROP:
                return Constants.bucketDrop;
            case INTAKE:
                return Constants.bucketIntake;
            case ASCENT:
                return Constants.bucketAscent;
            default:
                return 0;
        }
    }

    public BucketState getBucketState() {
        return this.bucketState;
    }

    public int getLiftPosition() {
        return this.lift1.getPosition();
    }

    public void setLiftPower(double power) {
        this.lift1.setPower(power);
        this.lift2.setPower(power);
    }

    public void setTargetLiftPosition(int target, double power) {
        this.lift1.setTargetPosition(target, power);
        this.lift2.setTargetPosition(target, power);
    }

    public void setLiftPosition(double power, int target) {
        this.lift1.setPosition(power, target);
        this.lift2.setPosition(power, target);
    }

    public void resetLift() {
        this.lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.lift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.lift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void periodic() {
        this.lift1.periodic();
        this.lift2.periodic();

        bucket.setPosition(getBucketStatePosition(bucketState));
    }

}