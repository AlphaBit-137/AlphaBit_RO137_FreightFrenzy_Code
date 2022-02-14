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

    private double pas = 0;

    Status SliderStatus = Status.STATIC;
    Status ArmStatus = Status.STATIC;
    Status IntakeStatus = Status.STATIC;

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
                telemetry.addData("DE DDE,]", 1);
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
                slider.switchToSliderRESET();
                SliderStatus = Status.STATIC;
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

            if (gamepad2.dpad_right && ArmStatus == Status.STATIC)
                arm.ArmMovement(1000, 0.5);
            if (gamepad2.dpad_left && ArmStatus == Status.STATIC)
                arm.ArmMovement(-1000, 0.5);

            if (gamepad2.y)
            {
                servo.Closed();
            }
            if (arm.ArmBUSY())
            {
                ArmStatus = Status.MOVING;
            }
            else if (ArmStatus == Status.MOVING)
                ArmStatus = Status.INITIALIZING;
            if (ArmStatus == Status.INITIALIZING)
            {
                arm.switchToArmRESET();
                ArmStatus = Status.STATIC;
            }

            telemetry.addData("Pozitia Arm", arm.arm.getCurrentPosition());
            telemetry.addData("Status Arm", ArmStatus);


            telemetry.addData("Pas", pas);

            if (gamepad2.a || IntakeStatus == Status.MOVING){
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

            if (gamepad2.x)
                intake.ResetPosition();

            if (intake.IntakeBUSY())
                IntakeStatus = Status.MOVING;
            else{
                if (IntakeStatus == Status.MOVING)
                    IntakeStatus = Status.INITIALIZING;
            }
            if (IntakeStatus == Status.INITIALIZING)
            {
                IntakeStatus = Status.STATIC;
                intake.switchToRESET();
            }


            if (Math.abs(intake.intakewing.getCurrentPosition()) >= 2850)
                intake.switchToRESET();

            telemetry.update();
            slider.update();
            arm.update();
            intake.update();
        }
    }

    public void test(){
        if (pas == 0){
            pas=1;
        }
        if (pas == 1)
        {
            pas=1.5;
            slider.SliderMovement(1350, 0.5);
        }
        if (pas == 1.5 && !slider.SliderBUSY()){
            pas = 2;
        }

        if (pas == 2)
        {
            pas=2.5;
            arm.ArmMovement(-1000, 0.5);
        }
        if (pas == 2.5 && !arm.ArmBUSY()){
            pas = 3;
        }

        if (pas == 3){
            pas = 3.5;
            slider.SliderMovement(-1000, 0.5);
        }
        if (pas == 3.5 && !slider.SliderBUSY())
        {
            pas = 0;
        }
    }

}

