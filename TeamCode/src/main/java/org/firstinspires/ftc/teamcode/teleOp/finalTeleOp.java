package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hware.hwMap;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;

// TODO - SET THE ALLIANCE SIDE
@TeleOp(name="FinalTeleOp", group="FINAL")
public class finalTeleOp extends LinearOpMode {
    private int alliance;



    private StateMachine stateMachine;

    private boolean launching = false;

    @Override
    public void runOpMode() throws InterruptedException {
        alliance = 1;
        hwMap.LiftHwMap h_lift = new hwMap.LiftHwMap(hardwareMap);
        hwMap.DriveHwMap h_driveTrain = new hwMap.DriveHwMap(hardwareMap);
        hwMap.IntakeHwMap h_intake = new hwMap.IntakeHwMap(hardwareMap);
        hwMap.TransferHwMap h_transfer = new hwMap.TransferHwMap(hardwareMap);
        hwMap.TurretHwMap h_turret = new hwMap.TurretHwMap(hardwareMap);

        stateMachine = new StateMachine(h_lift, h_driveTrain, h_intake, h_transfer, h_turret);
        stateMachine.getTurret().setAlliance(alliance);

        waitForStart();
        stateMachine.setRobotState(RobotState.TELEOP);
        stateMachine.getDriveTrain().setDriveState(DriveTrain.DriveState.NORMAL);

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            stateMachine.getDriveTrain().teleopDrive(x, y, rx);

            if (gamepad1.dpad_up) {
                stateMachine.getDriveTrain().setDriveState(DriveTrain.DriveState.TURBO);
            }
            else if (gamepad1.dpad_left) {
                stateMachine.getDriveTrain().setDriveState(DriveTrain.DriveState.NORMAL);
            }
            else if (gamepad1.dpad_down) {
                stateMachine.getDriveTrain().setDriveState(DriveTrain.DriveState.PRECISION);
            }


            if (gamepad1.right_trigger > 0.2) {
                stateMachine.setGameState(GameState.INTAKING);
            } else if (gamepad1.left_trigger > 0.2) {
                stateMachine.setGameState(GameState.EXTAKING);
            }
            else {
                stateMachine.setGameState(GameState.IDLE);
            }

            if (gamepad2.a && stateMachine.getCurrentGameState() != GameState.SCORING) {
                stateMachine.setGameState(GameState.SCORING);
            }
            else if (gamepad2.a && stateMachine.getCurrentGameState() == GameState.SCORING)  {
                stateMachine.setGameState(GameState.IDLE);
            }

            if (gamepad2.x && !launching) {
                launching = true;
                stateMachine.getTurret().launchTurret();
            }
            else if (gamepad2.x && launching)  {
                launching = false;
                stateMachine.getTurret().stop();
            }


            if (gamepad2.dpad_up) {
                stateMachine.getTurret().setLaunchPower(Constants.TurretConstants.TURRET_POWER_MAX);
            }
            else if (gamepad2.dpad_left) {
                stateMachine.getTurret().setLaunchPower(Constants.TurretConstants.TURRET_POWER_MID);
            }
            else if (gamepad2.dpad_down) {
                stateMachine.getTurret().setLaunchPower(Constants.TurretConstants.TURRET_POWER_LOW);
            }



            if (gamepad1.back) {
                stateMachine.emergencyStop();
            }
            if(gamepad1.start){
                if (alliance == 1) {
                    alliance ++;
                    stateMachine.getTurret().setAlliance(alliance);
                }
                else {
                    alliance --;
                    stateMachine.getTurret().setAlliance(alliance);
                }
            }

            stateMachine.update();

            // Telemetry
            telemetry.addData("Robot State", stateMachine.getCurrentRobotState());
            telemetry.addData("Drive State", stateMachine.getDriveTrain().getDriveState());
            telemetry.addData("Alliance",  alliance == 1 ? "Blue" : "Red");
            telemetry.update();
        }

        stateMachine.setRobotState(RobotState.ESTOP);
    }
}