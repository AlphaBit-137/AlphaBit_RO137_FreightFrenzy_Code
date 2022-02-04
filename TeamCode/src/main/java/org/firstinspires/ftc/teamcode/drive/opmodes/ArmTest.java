package org.firstinspires.ftc.teamcode.drive.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sun.tools.javac.util.Position;

import org.firstinspires.ftc.teamcode.drive.sliders.Lift;

@TeleOp
public class ArmTest extends LinearOpMode {

    Lift slider = new Lift();
    Positions poz = Positions.DOWN;

    private ElapsedTime runtime = new ElapsedTime();

    public enum Positions
    {
        UP,
        DOWN
    }

    @Override
    public void runOpMode() throws InterruptedException {

        slider.init(hardwareMap);

    }

        void SwitchToUP()
        {
            if (poz == Positions.DOWN)
            {
                slider.lift.setTargetPosition(-1150);
                slider.lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                slider.lift.setPower(0.8);
               /* while(slider.lift.getCurrentPosition()>-1150)
                {
                   // telemetry.addData("Slider", slider.lift.getCurrentPosition());
                   // telemetry.update();
                }
                slider.lift.setPower(0);*/
                poz = Positions.UP;
            }
        }

        void SwitchToDOWN()
        {
            if (poz == Positions.UP)
            {
                slider.lift.setTargetPosition(0);
                slider.lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                slider.lift.setPower(1.0);
                /*

                while(slider.lift.getCurrentPosition()<0)
                {
                    //telemetry.addData("Slider", slider.lift.getCurrentPosition());
                    //telemetry.update();
                }
                slider.lift.setPower(0);*/
                poz=Positions.DOWN;
            }
        }
}
