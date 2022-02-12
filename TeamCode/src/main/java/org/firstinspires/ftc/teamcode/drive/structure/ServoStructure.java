package org.firstinspires.ftc.teamcode.drive.structure;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ServoStructure {

    public Servo servo;

    HardwareMap hwMap = null;

    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        // Define and Initialize Motors
        servo = hwMap.get(Servo.class, "Servo");
        servo.setPosition(0);
    }

    public void Open(){servo.setPosition(0.3);}

    public void Closed(){servo.setPosition(0);}
}
