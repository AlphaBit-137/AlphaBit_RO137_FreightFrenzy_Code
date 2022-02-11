package org.firstinspires.ftc.teamcode.drive.structure;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.opmodes.First;

@TeleOp
public class ArmPrototype extends LinearOpMode {

    ArmStructure arm = new ArmStructure();

    int test=0;

    @Override
    public void runOpMode() throws InterruptedException {

        arm.init(hardwareMap);

        waitForStart();

        while (opModeIsActive()){

            if (gamepad2.right_bumper){
                arm.switchToArmUp();
            }else{
                if (gamepad2.left_bumper){
                    arm.switchToArmDown();
                }
                else{
                    arm.switchToSTOP();
                }
            }

            if (gamepad2.dpad_up){
                arm.SliderMovement(1600, 0.5);
            }

            if (arm.slider.isBusy())
                test=1;
            else
                test=0;
            if (test == 0)
            {
                test=-1;
                arm.switchToResetSlider();
            }

            arm.update();

        }
    }
}
