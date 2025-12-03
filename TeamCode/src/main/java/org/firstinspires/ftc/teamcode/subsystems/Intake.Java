package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.Hware.hwMap;
import org.firstinspires.ftc.teamcode.teleOp.Constants;

public class Intake {
    private double runPower = 1;
    private final hwMap.IntakeHwMap h_intake;
    public enum IntakeState {
        INTAKE,
        EXTAKE,
        IDLE,
        STOP
    }
    private IntakeState currentState = IntakeState.IDLE;
    public Intake(hwMap.IntakeHwMap hardware) {
        this.h_intake = hardware;
    }
    public void setIntakeState(IntakeState state) {
        this.currentState = state;

        switch (state) {
            case INTAKE:
                h_intake.frontIntakeMotor.setPower(Constants.IntakeConstants.INTAKE_POWER);
                h_intake.backIntakeMotor.setPower(Constants.IntakeConstants.INTAKE_POWER);
                break;
            case EXTAKE:
                h_intake.frontIntakeMotor.setPower(Constants.IntakeConstants.EXTAKE_POWER);
                h_intake.backIntakeMotor.setPower(Constants.IntakeConstants.EXTAKE_POWER);
                break;
            case IDLE:
                h_intake.frontIntakeMotor.setPower(Constants.IntakeConstants.IDLE_POWER);
                h_intake.backIntakeMotor.setPower(Constants.IntakeConstants.IDLE_POWER);
                break;
            case STOP:
                break;
        }
    }
}
