package org.firstinspires.ftc.teamcode.drive.structure;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
public class Carusel extends LinearOpMode {
    public DcMotor Duck=null;

    public Carusel.DuckModes RobotDuck = Carusel.DuckModes.STOP;

    @Override
    public void runOpMode() throws InterruptedException {

    }
    HardwareMap hwMap = null;

    enum DuckModes {
        IN,
        OUT,
        STOP
    }

    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        // Define and Initialize Motors
        Duck = hwMap.get(DcMotor.class, "Duck");
        Duck.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Duck.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Duck.setPower(0);
    }

    public void update(){
        switch (RobotDuck){
            case IN:{
                Duck.setPower(0.3);
                break;
            }
            case OUT:{
                Duck.setPower(-0.3);
                break;
            }
            case STOP:{
                Duck.setPower(0);
                break;
            }
        }
    }

    public void switchToIN() {RobotDuck = Carusel.DuckModes.IN;}

    public void switchToOUT() {RobotDuck = Carusel.DuckModes.OUT;}

    public void switchToSTOP() {RobotDuck = Carusel.DuckModes.STOP;}
}

