package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Hware.hwMap;
import org.firstinspires.ftc.teamcode.teleOp.Constants;

public class Lift {
    private double runPower = 1;
    private final hwMap.LiftHwMap h_lift;
    private final hwMap.DriveHwMap h_driveTrain;

    // Initialize intake motors

    public enum LiftState {
        LIFT_UP,
        LIFT_DOWN,
        LOCK,
        IDLE
    }
    private Lift.LiftState currentState = Lift.LiftState.IDLE;
    public Lift(hwMap.LiftHwMap h_liftHwMap, hwMap.DriveHwMap h_driveHwMap) {
        this.h_lift = h_liftHwMap;
        this.h_driveTrain = h_driveHwMap;
    }

    public void setLiftState(Lift.LiftState state) {
        this.currentState = state;

        switch (state) {
            case LIFT_UP:
                h_driveTrain.setMotorTargetPositions(18);
                h_driveTrain.setMotorZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                h_driveTrain.setMotorTargetPositions(18);
                h_driveTrain.setDriveMotorModes(DcMotor.RunMode.RUN_TO_POSITION);
                h_driveTrain.setMotorPowers(1,1,1,1);
                break;
            case LIFT_DOWN:
                h_driveTrain.setMotorPowers(0,0,0,0);
                break;
            case LOCK:
                setLiftState(LiftState.LIFT_UP);
                break;
            case IDLE:
                break;
        }
    }

    public void setServoPosition() {
        h_lift.ptoLeft.setPosition(Constants.LiftConstants.PTO_LOCK);
        h_lift.ptoRight.setPosition(Constants.LiftConstants.PTO_LOCK);
    }
}
