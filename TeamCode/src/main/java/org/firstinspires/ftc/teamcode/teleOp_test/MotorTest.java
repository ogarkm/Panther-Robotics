// #---------------------------#
//  Motor Test
//  
// #---------------------------#

package org.firstinspires.ftc.teamcode.teleOp_test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="MOTOR_TEST", group="TEST")
public class MotorTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Motor Declaration

        DcMotor motor = hardwareMap.dcMotor.get("motor");

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            motor.setPower(-gamepad1.left_stick_y);
        }
    }
}
