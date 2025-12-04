package org.firstinspires.ftc.teamcode.tuning;

import android.graphics.Color;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import org.firstinspires.ftc.teamcode.Hware.hwMap;

@TeleOp(name="Color Sensor Tuning", group="Diagnostics")
public class ColorSensorTuning extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        hwMap.TransferHwMap transfer = new hwMap.TransferHwMap(hardwareMap);

        telemetry.addLine("Color Sensor Tuning Mode");
        telemetry.addLine("Place artifacts in slots and observe HSV values");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Display all three sensors
            for (int i = 1; i <= 3; i++) {
                displaySensorData(transfer, i);
            }

            telemetry.addLine("\n--- Detection Results ---");
            telemetry.addData("Slot A", getColorName(transfer.detectArtifactColor(1)));
            telemetry.addData("Slot B", getColorName(transfer.detectArtifactColor(2)));
            telemetry.addData("Slot C", getColorName(transfer.detectArtifactColor(3)));

            telemetry.update();
            sleep(100);
        }
    }

    private void displaySensorData(hwMap.TransferHwMap transfer, int index) {
        NormalizedRGBA colors = transfer.sensors[index-1].getNormalizedColors();

        float[] hsv = new float[3];
        Color.colorToHSV(colors.toColor(), hsv);

        String slotName = (index == 1) ? "A" : (index == 2) ? "B" : "C";

        telemetry.addLine("\n--- Slot " + slotName + " ---");
        telemetry.addData("RGB", "R:%.2f G:%.2f B:%.2f",
                colors.red, colors.green, colors.blue);
        telemetry.addData("HSV", "H:%.1f S:%.2f V:%.2f",
                hsv[0], hsv[1], hsv[2]);
        telemetry.addData("Raw Alpha", "%.2f", colors.alpha);
    }

    private String getColorName(int colorCode) {
        switch (colorCode) {
            case 0: return "NONE/ERROR";
            case 1: return "PURPLE";
            case 2: return "GREEN";
            default: return "UNKNOWN";
        }
    }
}