package org.firstinspires.ftc.teamcode.drive.structure;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ArmStructure {

    public DcMotor arm;
    ElapsedTime runtime = new ElapsedTime();
    Positions ArmPosition = Positions.STOP;
    HardwareMap hwMap = null;

    public enum Positions{
        UP,
        DOWN,
        STOP,
        RESET
    }

    public void init(HardwareMap ahwMap) {

        hwMap = ahwMap;
        // Define and Initialize Motors
        arm = hwMap.get(DcMotor.class, "Arm");
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setDirection(DcMotor.Direction.FORWARD);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setPower(0);
    }

    public void ArmMovement(int position, double power){
        arm.setTargetPosition(position);
        arm.setMode((DcMotor.RunMode.RUN_TO_POSITION));
        arm.setPower(power);
    }


    public void update(){
        switch (ArmPosition){
            case UP:{
                arm.setPower(-0.5);
                break;
            }
            case DOWN:{
                arm.setPower(0.5);
                break;
            }
            case STOP:{
                arm.setPower(0);
                break;
            }
            case RESET:{
                arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }
    }

    public void switchToArmUp() {ArmPosition = Positions.UP;}

    public void switchToArmDown() {ArmPosition = Positions.DOWN;}

    public void switchToArmSTOP() {ArmPosition = Positions.STOP;}

    public void switchToArmRESET() {ArmPosition = Positions.RESET;}

    public boolean ArmBUSY() {return arm.isBusy();}

}
