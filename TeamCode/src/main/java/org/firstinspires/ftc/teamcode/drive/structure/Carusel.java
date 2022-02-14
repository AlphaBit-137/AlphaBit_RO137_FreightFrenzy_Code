package org.firstinspires.ftc.teamcode.drive.structure;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Carusel extends LinearOpMode {
    public DcMotor Duck=null;
    public boolean reset = true;

    public DuckModes RobotDuck = DuckModes.STOP;

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
        if(reset){ Duck.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); reset=false;}
        Duck.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Duck.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Duck.setPower(0);
    }

    public void update(){
        switch (RobotDuck){
            case IN:{
                Duck.setPower(0.5);
                break;
            }
            case OUT:{
                Duck.setPower(-0.5);
                break;
            }
            case STOP:{
                Duck.setPower(0);
                break;
            }
        }
    }

    public void switchToIN() {RobotDuck = DuckModes.IN;}

    public void switchToOUT() {RobotDuck = DuckModes.OUT;}

    public void switchToSTOP() {RobotDuck = DuckModes.STOP;}
}

