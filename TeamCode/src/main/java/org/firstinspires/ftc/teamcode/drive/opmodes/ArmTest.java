package org.firstinspires.ftc.teamcode.drive.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.sliders.Lift;

@TeleOp
public class ArmTest extends LinearOpMode {

    Lift slider = new Lift();

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        slider.init(hardwareMap);


        runtime.reset();
        waitForStart();

        if (opModeIsActive()){

            slider.lift.setTargetPosition(-1500);
            slider.lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            slider.lift.setPower(0.8);
            while(slider.lift.getCurrentPosition()>-1150)
            {
                telemetry.addData("Slider", slider.lift.getCurrentPosition());
                telemetry.update();
            }
            slider.lift.setPower(0);
          /*  sleep(5000);

            slider.lift.setTargetPosition(-50);
            slider.lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            slider.lift.setPower(1.0);
            while(slider.lift.getCurrentPosition()>0)
            {
                telemetry.addData("Slider", slider.lift.getCurrentPosition());
                telemetry.update();
            }
            slider.lift.setPower(0);*/

        }
    }
}
