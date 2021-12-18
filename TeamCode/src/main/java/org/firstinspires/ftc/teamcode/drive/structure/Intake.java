package org.firstinspires.ftc.teamcode.drive.structure;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake extends LinearOpMode {
    public DcMotor intake = null;

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

    HardwareMap hwMap = null;

    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        // Define and Initialize Motors
        intake = hwMap.get(DcMotor.class, "Intake");
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setPower(0);
    }

    public void update(){
        switch (RobotIntake){
            case IN:{
                intake.setPower(0.5);
                break;
            }
            case OUT:{
                intake.setPower(-0.5);
                break;
            }
            case STOP:{
                intake.setPower(0);
                break;
            }
        }
    }

    public void switchToIN() {RobotIntake = IntakeModes.IN;}

    public void switchToOUT() {RobotIntake = IntakeModes.OUT;}

    public void switchToSTOP() {RobotIntake = IntakeModes.STOP;}
}
