package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMRobotInner;

public class MMRobot extends MMRobotInner {

    public MMRobot(){
        setControlHubName("Control Hub");
        setExpansionHubName("Expansion Hub");
    }

    @Override
    public void initAuto() {
        super.initAuto();
    }

    @Override
    public void initTele() {super.initTele();}

    @Override
    public void initDebug() {
        super.initDebug();
    }
}
