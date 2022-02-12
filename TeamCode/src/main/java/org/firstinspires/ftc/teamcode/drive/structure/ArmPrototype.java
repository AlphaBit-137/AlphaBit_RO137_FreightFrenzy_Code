package org.firstinspires.ftc.teamcode.drive.structure;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.opmodes.First;
import org.firstinspires.ftc.teamcode.drive.opmodes.IntakeTEst;

@TeleOp
public class ArmPrototype extends LinearOpMode {

    ArmStructure arm = new ArmStructure();
    SliderStructure slider = new SliderStructure();
    ServoStructure servo = new ServoStructure();
    Intake intake = new Intake();

    Status SliderStatus = Status.STATIC;
    Status ArmStatus = Status.STATIC;

    public enum Status{
        MOVING,
        INITIALIZING,
        STATIC
    }

    @Override
    public void runOpMode() throws InterruptedException {

        slider.init(hardwareMap);
        arm.init(hardwareMap);
        servo.init(hardwareMap);
        intake.init(hardwareMap);

        waitForStart();

        while (opModeIsActive()){

            if (gamepad2.right_bumper || slider.SliderBUSY()){
                slider.switchToSliderUp();
            }else{
                if (gamepad2.left_bumper){
                    slider.switchToSliderDown();
                }
                else{
                    slider.switchToSliderSTOP();
                }
            }

            if (gamepad2.dpad_up){
                slider.SliderMovement(-1400, 0.5);
            }
            if (gamepad2.dpad_down)
                slider.SliderMovement(1400, 0.5);


            if (slider.SliderBUSY())
            {
                SliderStatus = Status.MOVING;
            }
            else if (SliderStatus == Status.MOVING)
                SliderStatus = Status.INITIALIZING;
            if (SliderStatus == Status.INITIALIZING)
            {
                SliderStatus = Status.STATIC;
                slider.switchToSliderRESET();
            }

            telemetry.addData("Pozitia Slider", slider.slider.getCurrentPosition());
            telemetry.addData("Status Slider", SliderStatus);

            if(gamepad2.right_trigger!=0 && gamepad2.left_trigger==0 || arm.ArmBUSY()){
                arm.switchToArmUp();
            }else{
                if(gamepad2.right_trigger==0 && gamepad2.left_trigger!=0){
                    arm.switchToArmDown();
                }else arm.switchToArmSTOP();
            }
            if (arm.ArmBUSY())
            {
                ArmStatus = Status.MOVING;
            }
            else if (ArmStatus == Status.MOVING)
                ArmStatus = Status.INITIALIZING;
            if (ArmStatus == Status.INITIALIZING)
            {
                ArmStatus = Status.STATIC;
                arm.switchToArmRESET();
            }

            telemetry.addData("Pozitia Arm", arm.arm.getCurrentPosition());
            telemetry.addData("Status Arm", ArmStatus);

            if (gamepad2.a){
                intake.switchToIN();
            }
            else{
                if (gamepad2.b){
                    intake.switchToOUT();
                }
                else{

                    intake.switchToSTOP();
                }
            }

            telemetry.update();
            slider.update();
            arm.update();
            intake.update();
        }
    }
}

