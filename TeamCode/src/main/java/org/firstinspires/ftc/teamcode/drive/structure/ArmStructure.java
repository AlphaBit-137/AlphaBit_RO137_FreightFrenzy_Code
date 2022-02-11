package org.firstinspires.ftc.teamcode.drive.structure;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ArmStructure {

    public DcMotor arm;
    public DcMotor slider;
    public Servo servo;
    ElapsedTime runtime = new ElapsedTime();
    Positions RobotSliderPos = Positions.STOP;
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
        slider = hwMap.get(DcMotor.class,"Slider");
        arm = hwMap.get(DcMotor.class, "Arm");
        servo = hwMap.get(Servo.class, "Servo");

        slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        arm.setDirection(DcMotor.Direction.FORWARD);
        slider.setDirection(DcMotor.Direction.FORWARD);
        servo.setDirection(Servo.Direction.FORWARD);

        slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        arm.setPower(0);
        slider.setPower(0);
        servo.setPosition(0);
    }


    public void SliderMovement(int position, double power)
    {
        slider.setTargetPosition(position);
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slider.setPower(power);
        runtime.reset();

        slider.setPower(0);
    }

    public void ArmMovement(int position, double power){
        arm.setTargetPosition(position);
        arm.setMode((DcMotor.RunMode.RUN_TO_POSITION));
        arm.setPower(power);
        runtime.reset();
        arm.setPower(0);
    }

    public void STOP(){
        ///Emergency Stop

        arm.setTargetPosition(0);
        arm.setPower(0.6);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        runtime.reset();
        while(arm.isBusy() && runtime.seconds()<2.0){

        }
        arm.setPower(0);


        slider.setTargetPosition(10);
        slider.setPower(0.5);
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        runtime.reset();
        while(slider.getCurrentPosition() > 10 && runtime.seconds()<2.0){

        }
        slider.setPower(0);

    }

    public void update(){
        switch (RobotSliderPos){
            case UP:{
                slider.setPower(-0.5);
                break;
            }
            case DOWN:{
                slider.setPower(0.5);
                break;
            }
            case STOP:{
                slider.setPower(0);
                break;
            }
            case RESET:{
                slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }
    }


    public void Opened(){servo.setPosition(0);}

    public void Closed(){servo.setPosition(0.3);}

    public void switchToArmUp() {RobotSliderPos = Positions.UP;}

    public void switchToArmDown() {RobotSliderPos = Positions.DOWN;}

    public void switchToSTOP() {RobotSliderPos = Positions.STOP;}

    public void switchToResetSlider() {RobotSliderPos = Positions.RESET;}

}
