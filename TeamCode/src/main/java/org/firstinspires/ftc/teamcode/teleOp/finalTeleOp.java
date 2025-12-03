package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hware.hwMap;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;


@TeleOp(name="FinalTeleOp", group="FINAL")
public class finalTeleOp extends LinearOpMode {

    private StateMachine stateMachine;

    @Override
    public void runOpMode() throws InterruptedException {
        hwMap.LiftHwMap h_lift = new hwMap.LiftHwMap(hardwareMap);
        hwMap.DriveHwMap h_driveTrain = new hwMap.DriveHwMap(hardwareMap);
        hwMap.IntakeHwMap h_intake = new hwMap.IntakeHwMap(hardwareMap);
        hwMap.TransferHwMap h_transfer = new hwMap.TransferHwMap(hardwareMap);

        stateMachine = new StateMachine(h_lift, h_driveTrain, h_intake, h_transfer);

        waitForStart();
        stateMachine.setRobotState(RobotState.TELEOP);

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            stateMachine.getDriveTrain().teleopDrive(x, y, rx);

            if (gamepad1.right_trigger > 0.5) {
                stateMachine.getDriveTrain().setDriveState(DriveTrain.DriveState.PRECISION);
            } else if (gamepad1.left_trigger > 0.5) {
                stateMachine.getDriveTrain().setDriveState(DriveTrain.DriveState.TURBO);
            } else {
                stateMachine.getDriveTrain().setDriveState(DriveTrain.DriveState.NORMAL);
            }

            if (gamepad1.right_bumper) {
                stateMachine.setGameState(GameState.INTAKING);
            } else if (gamepad1.left_bumper) {
                stateMachine.setGameState(GameState.EXTAKING);
            } else {
                stateMachine.setGameState(GameState.IDLE);
            }

            if (gamepad1.back) {
                stateMachine.emergencyStop();
            }
            if(gamepad1.start){
                stateMachine.setGameState(GameState.LIFTING);
            }

            // Telemetry
            telemetry.addData("Robot State", stateMachine.getCurrentRobotState());
            telemetry.addData("Drive State", stateMachine.getDriveTrain().getDriveState());
            telemetry.update();
        }

        stateMachine.setRobotState(RobotState.ESTOP);
    }
}