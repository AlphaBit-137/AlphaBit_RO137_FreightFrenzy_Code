package org.firstinspires.ftc.teamcode.drive.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.structure.Intake;
import org.firstinspires.ftc.teamcode.drive.structure.ServoStructure;

@TeleOp
public class IntakeTEst extends LinearOpMode {

    Intake intake = new Intake();
    ServoStructure servo = new ServoStructure();
    int test = 0;

    Status IntakeStatus = Status.STATIC;

    public enum Status{

        MOVING,
        INITIALIZING,
        STATIC

    }

    @Override
    public void runOpMode() throws InterruptedException {

        intake.init(hardwareMap);
        servo.init(hardwareMap);

        waitForStart();

        while (opModeIsActive()){

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

            if(gamepad2.dpad_right)
                servo.Open();
            if (gamepad2.dpad_left)
                servo.Closed();


            if (Math.abs(intake.intakewing.getCurrentPosition()) >= 2850)
                intake.switchToRESET();

            telemetry.addData("Pozitia Intake:", intake.intakewing.getCurrentPosition());
            telemetry.addData("Status Intake", IntakeStatus);
            telemetry.update();

            intake.update();
        }
    }
}
