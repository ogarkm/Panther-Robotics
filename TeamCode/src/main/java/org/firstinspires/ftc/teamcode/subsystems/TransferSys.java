package org.firstinspires.ftc.teamcode.subsystems;
import org.firstinspires.ftc.teamcode.Hware.hwMap;

import java.util.Arrays;
import java.util.List;
import java.util.ArrayList;
public class TransferSys {
    private final hwMap.TransferHwMap hardware;

    private final int[] artifactColors = {0, 0, 0};
    private int[] motif = {2, 1, 1};
    public enum TransferState {
        INDEXING,
        FLICKING,
        IDLING,
        STOP
    }
    private TransferState currentState = TransferState.IDLING;

    public TransferSys(hwMap.TransferHwMap hardware) {
        this.hardware = hardware;
    }


    public void stopTransfer() {
        if (currentState != TransferState.STOP) {
            setTransferState(TransferState.STOP);
        }
    }

    public void indexAllArtifacts() {
        for (int i = 1; i <= 3; i++) {
            int color = hardware.detectArtifactColor(i);
            artifactColors[i - 1] = color;
        }
    }

    public int[] getArtifactColors() {
        return Arrays.copyOf(artifactColors, artifactColors.length);
    }

    public void setArtifactColorManual(int index, int color) {
        if (index < 1 || index > 3) return;
        artifactColors[index - 1] = color;
    }

    public void setMotif(int[] newMotif) {
        if (newMotif == null || newMotif.length != 3) {
            throw new IllegalArgumentException("Motif must be length 3");
        }
        this.motif = Arrays.copyOf(newMotif, 3);
    }

    public int[] getMotif() {
        return Arrays.copyOf(motif, motif.length);
    }


    public void setTransferState(TransferState state) {
        this.currentState = state;

        switch (state) {
            case INDEXING:
                indexAllArtifacts();
                resetAllFlickers();
                break;

            case FLICKING:
                flickAll();
                break;

            case IDLING:
                resetAllFlickers();
                break;

            case STOP:
                resetAllFlickers();
                break;
        }
    }

    private void flickArtifact(int slot) {
        hardware.setTransferPos(slot, true);
    }

    public void resetFlicker(int slot) {
        hardware.setTransferPos(slot, false);
    }

    public void resetAllFlickers() {
        for (int i = 1; i <= 3; i++) {
            resetFlicker(i);
        }
    }

    public List<Integer> buildFlickPlan() {
        List<Integer> plan = new ArrayList<>(3);
        boolean[] used = new boolean[3];

        for (int desiredColor : motif) {
            for (int slot = 0; slot < 3; slot++) {
                if (!used[slot] && artifactColors[slot] == desiredColor) {
                    plan.add(slot + 1);
                    used[slot] = true;
                    break;
                }
            }
        }

        for (int slot = 0; slot < 3; slot++) {
            if (!used[slot]) {
                plan.add(slot + 1);
            }
        }

        return plan;
    }


    public void flickAll() {
        indexAllArtifacts();

        List<Integer> plan = buildFlickPlan();

        for (int slotIndex : plan) {
            int color = artifactColors[slotIndex - 1];
            if (color == 0) continue;


            flickArtifact(slotIndex);

            artifactColors[slotIndex - 1] = 0;
        }
    }

    public TransferState getTransferState() {
        return currentState;
    }

}
