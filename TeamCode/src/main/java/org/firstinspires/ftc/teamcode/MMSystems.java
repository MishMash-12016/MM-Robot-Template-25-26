package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleRevHub;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.Subsystems.Motor.MotorPositionProfiledPidSubsystem;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.Utils.MMBattery;

//TODO:everything(to build a class you first need to invent the entire universe)
public class MMSystems {

    public CuttleRevHub controlHub;

    public void initTeleopSystems(){

    }

    public MMSystems(){
    }


    /**
     * the robot instance
     */
    private static MMSystems instance;

    /**
     * the get method for the singleton
     * @return the robot instance
     */
    public static synchronized MMSystems getInstance() {
        if (instance == null) {
            instance = new MMSystems();
        }
        return instance;
    }


}
