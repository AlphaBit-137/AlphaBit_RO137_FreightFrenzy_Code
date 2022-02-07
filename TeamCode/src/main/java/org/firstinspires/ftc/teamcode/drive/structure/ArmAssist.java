package org.firstinspires.ftc.teamcode.drive.structure;


import android.telephony.CellSignalStrength;
import android.transition.Slide;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.camera.delegating.DelegatingCaptureSequence;

public class ArmAssist  extends LinearOpMode {

    public DcMotor arm;
    public DcMotor slider;

    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

    }

    HardwareMap hwMap = null;
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        // Define and Initialize Motors
        slider = hwMap.get(DcMotor.class,"Slider");
        arm = hwMap.get(DcMotor.class, "Arm");


        slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        arm.setDirection(DcMotor.Direction.FORWARD);
        slider.setDirection(DcMotor.Direction.FORWARD);

        slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        arm.setPower(0);
        slider.setPower(0);
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
    public void Assist_Strong(){
        //Powerful Throw

        SliderMovement(-1150, 0.5, -1, 2.0);
        ArmMovement(-1076, 0.6, 2.0);
        ArmMovement(0, 0.6, 2.0);
        SliderMovement(-10, 0.5, 1, 2.0);

    }
    public void Assist_Weak(){

        //Slow Throw
        SliderMovement(-1150, 0.5, -1, 25);
        ArmMovement(-1076, 0.4, 4.0);
        ArmMovement(0, 0.4, 4.0);
        SliderMovement(-10, 0.5, 1, 5);

    }
    public void Complex(){
        //Balanced Throw

        SliderMovement(1050, 0.7, 1, 5.0);
        ArmMovement(-500, 0.6, 5.0);
        SliderMovement(10, 0.7, -1, 5.0);
        ArmMovement(-1100, 0.2, 8.0);
        SliderMovement(1000, 0.7, 1, 5.0);
        ArmMovement(10, 0.6, 5.0);
        SliderMovement(0, 0.5, -1, 5.0);
    }

    public void SliderMovement(int position, double power, int direction, double limit)
    {
        slider.setTargetPosition(-position);
        slider.setPower(power);
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        runtime.reset();
        if (direction > 0)
            while (slider.isBusy() && runtime.seconds() < limit) {

            }
        slider.setPower(0);
    }

    public void ArmMovement(int position, double power, double limit){
        arm.setTargetPosition(position);
        arm.setPower(power);
        arm.setMode((DcMotor.RunMode.RUN_TO_POSITION));
        runtime.reset();
        while (arm.isBusy() && runtime.seconds() < limit)
        {

        }
        if(arm.isBusy() && runtime.seconds() >limit)STOP();
        arm.setPower(0);
    }

    public void RobotDown(double power) {
        arm.setPower(power);
    }

    public void RobotUP(double power) {
        arm.setPower(-power);
    }

    public void RobotStop() {
        arm.setPower(0);
    }

    public void RobotDownS(){
        slider.setPower(0.6);
    }
    public void RobotUpS(){
        slider.setPower(-0.6);
    }
    public void RobotStopS(){
        slider.setPower(0);
    }

}
