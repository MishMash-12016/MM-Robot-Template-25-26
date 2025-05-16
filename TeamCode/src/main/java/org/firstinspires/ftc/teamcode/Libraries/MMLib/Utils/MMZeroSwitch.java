package org.firstinspires.ftc.teamcode.Libraries.MMLib.Utils;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleDigital;

public class MMZeroSwitch {
    private final CuttleDigital zeroSwitch;
    private final double resetPose;

    public MMZeroSwitch(CuttleDigital zeroSwitch, double resetPose){
        this.zeroSwitch = zeroSwitch;
        this.resetPose = resetPose;
    }

    public double getResetPose(){
        return resetPose;
    }

    public boolean getState(){
        return zeroSwitch.getState();
    }
}
