package org.firstinspires.ftc.teamcode.drive.structure;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake extends LinearOpMode {
    public DcMotor intakewing = null;

    public IntakeModes RobotIntake = IntakeModes.STOP;

    @Override
    public void runOpMode() throws InterruptedException {

    }

    public enum IntakeModes
    {
        IN,
        OUT,
        STOP
    }

    public Intake(){

    }

    HardwareMap hwMap = null;

    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        // Define and Initialize Motors
        intakewing = hwMap.get(DcMotor.class, "Intake");
        intakewing.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakewing.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakewing.setPower(0);
    }

    public void update(){
        switch (RobotIntake){
            case IN:{
                intakewing.setPower(-0.8);
                break;
            }
            case OUT:{
                intakewing.setPower(0.8);
                break;
            }
            case STOP:{
                intakewing.setPower(0);
                break;
            }
        }
    }

    public void switchToIN() {RobotIntake = IntakeModes.IN;}

    public void switchToOUT() {RobotIntake = IntakeModes.OUT;}

    public void switchToSTOP() {RobotIntake = IntakeModes.STOP;}
}
