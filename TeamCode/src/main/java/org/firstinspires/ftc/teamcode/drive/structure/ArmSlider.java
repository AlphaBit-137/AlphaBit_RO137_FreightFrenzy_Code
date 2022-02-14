package org.firstinspires.ftc.teamcode.drive.structure;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static android.os.SystemClock.sleep;


public class ArmSlider {
    public DcMotor arm;
    public DcMotor slider;
    public Servo servo;
    ElapsedTime runtime = new ElapsedTime();
    public ArmSlider.Armpos RobotPos = ArmSlider.Armpos.STOP;
    HardwareMap hwMap = null;
    public void init(HardwareMap ahwMap) {

        hwMap = ahwMap;
        // Define and Initialize Motors
        slider = hwMap.get(DcMotor.class,"Slider");
        arm = hwMap.get(DcMotor.class, "Arm");
        servo = hwMap.get(Servo.class, "Servo");

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        arm.setDirection(DcMotor.Direction.FORWARD);
        slider.setDirection(DcMotor.Direction.FORWARD);
        servo.setDirection(Servo.Direction.FORWARD);

        slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        arm.setPower(0);
        slider.setPower(0);
        servo.setPosition(0);
    }
    public enum Armpos{
        Level1,
        Level2,
        Level3,
        STOP,
    }

    public void update() {
        switch (RobotPos) {
            case Level1: {
                Closed();
                SliderMovement(1605, -0.5, 1, 2.0);
                ArmMovement(-500, 0.45, 2.0);
                ArmMovement(-1210, 0.33, 2.0);
                Opened();
                sleep(1200);
                ArmMovement(0, 0.3, 2.0);
                SliderMovement(10, 0.5, -1, 2.0);
                break;
            }
            case Level2: {
                Closed();
                SliderMovement(1600, 0.5, 1, 2.0);
                ArmMovement(-500, 0.4, 2.0);
                ArmMovement(-1300, 0.33, 2.0);
                Opened();
                sleep(1200);
                ArmMovement(0, 0.3, 2.0);
                SliderMovement(10, 0.5, -1, 2.0);
                break;
            }
            case Level3:{
                Closed();
                SliderMovement(1600, 0.5, 1, 2.0);
                ArmMovement(-500, 0.35, 2.0);
                SliderMovement(800, 0.5, -1, 2.0);
                ArmMovement(-1200, 0.23, 2.0);
                Opened();
                sleep(1200);
                SliderMovement(1600, 0.5, 1, 2.0);
                ArmMovement(0, 0.3, 2.0);
                SliderMovement(10, 0.5, -1, 2.0);
                break;
            }
            case STOP: {
                slider.setPower(0);
                arm.setPower(0);
                break;
            }
        }
    }
    public void SliderMovement(int position, double power, int direction, double limit)
    {
        slider.setTargetPosition(position);
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slider.setPower(power);
        runtime.reset();
        if (direction > 0)
            while (slider.getCurrentPosition() < position && runtime.seconds() < limit){

            }
        else
            while (slider.getCurrentPosition() > position && runtime.seconds() < limit) {

            }
        slider.setPower(0);
    }

    public void ArmMovement(int position, double power, double limit){
        arm.setTargetPosition(position);
        arm.setMode((DcMotor.RunMode.RUN_TO_POSITION));
        arm.setPower(power);
        runtime.reset();
        while (arm.isBusy() && runtime.seconds() < limit)
        {

        }
        if(arm.isBusy() && runtime.seconds() > limit)STOP();
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
    public void Opened(){servo.setPosition(0);}
    public void Closed(){servo.setPosition(0.3);}

    public void switchTo1() {RobotPos = Armpos.Level1;}

    public void switchTo2() {RobotPos = Armpos.Level2;}

    public void switchTo3() {RobotPos = Armpos.Level3;}

    public void switchToSTOP() {RobotPos = Armpos.STOP;}
}
