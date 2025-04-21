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

public class SpecimenSubsystem extends RE_SubsystemBase {

    private final RE_DcMotorEx specimenLift;
    private final RE_DcMotorExParams specimenLiftParams = new RE_DcMotorExParams(
            Constants.specimenLiftMin, Constants.specimenLiftMax, Constants.specimenLiftSlow,
            Constants.specimenLiftMaxPower, 1, Constants.specimenLiftUpRatio, Constants.specimenLiftDownRatio, Constants.specimenLiftSlowRatio
    );

    private final Servo specimenClaw;
    private SpecimenClawState specimenClawState;
    private SpecimenLiftState specimenLiftState;

    public enum SpecimenClawState {
        CLOSED,
        OPEN,
        NONE
    }

    public enum SpecimenLiftState {
        DOWN,
        GRAB,
        GRABBED,
        LOW,
        HIGH,
        DEPOSITED_LOW,
        DEPOSITED_HIGH
    }

    public SpecimenSubsystem(HardwareMap hardwareMap, String specimenClaw, String specimenLift) {
        this.specimenLift = new RE_DcMotorEx(hardwareMap.get(DcMotorEx.class, specimenLift), specimenLiftParams);
        this.specimenClaw = hardwareMap.get(Servo.class, specimenClaw);
        this.specimenClawState = SpecimenClawState.OPEN;
        this.specimenLiftState = SpecimenLiftState.DOWN;

        Robot.getInstance().subsystems.add(this);
    }

    @Override
    public void updateData() {
        Robot.getInstance().data.specimenLiftPosition = this.specimenLift.getPosition();
        Robot.getInstance().data.specimenClawState = this.specimenClawState;
        Robot.getInstance().data.specimenLiftState = this.specimenLiftState;
    }

    public void setSpecimenLiftPower(double power) {
        this.specimenLift.setPower(power);
    }

    public void setTargetSpecimenLiftPosition(int target) {
        this.specimenLift.setTargetPosition(target, this.specimenLiftParams.maxPower);
    }

    public void setSpecimenLiftPosition(double power, int target) {
        this.specimenLift.setPosition(power, target);
    }

    public void updateSpecimenClawState(SpecimenClawState state) {
        this.specimenClawState = state;
        this.specimenClaw.setPosition(getSpecimenClawStatePosition(state));
    }

    public void updateSpecimenLiftState(SpecimenLiftState state) {
        this.specimenLiftState = state;
        this.specimenLift.setTargetPosition(getSpecimenLiftStatePosition(state), 1);
    }

    private double getSpecimenClawStatePosition(SpecimenClawState state) {
        switch (state) {
            case OPEN:
                return Constants.specimenClawOpen;
            case CLOSED:
                return Constants.specimenClawClose;
            default:
                return 0;
        }
    }

    private int getSpecimenLiftStatePosition(SpecimenLiftState state) {
        switch (state) {
            case DOWN:
                return Constants.specimenLiftMin;
            case GRAB:
                return Constants.specimenLiftGrab;
            case GRABBED:
                return Constants.specimenLiftGrab + Constants.specimenLiftGrabbedOffset;
            case LOW:
                return Constants.specimenLiftLow;
            case DEPOSITED_LOW:
                return Constants.specimenLiftLow - Constants.specimenLiftHangOffset;
            case HIGH:
                return Constants.specimenLiftHigh;
            case DEPOSITED_HIGH:
                return Constants.specimenLiftHigh - Constants.specimenLiftHangOffset;
            default:
                return 0;
        }
    }

    public void resetLift() {
        this.specimenLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.specimenLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public SpecimenClawState getSpecimenClawState() {
        return this.specimenClawState;
    }
    public SpecimenLiftState getSpecimenLiftState() {
        return this.specimenLiftState;
    }
    public int getSpecimenLiftPosition() {
        return this.specimenLift.getPosition();
    }

    @Override
    public void periodic() {
        this.specimenLift.periodic();

        this.specimenClaw.setPosition(getSpecimenClawStatePosition(this.specimenClawState));
//        this.specimenLift.setTargetPosition(getSpecimenLiftStatePosition(this.specimenLiftState), 1);
    }

}