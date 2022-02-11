package org.firstinspires.ftc.teamcode.drive.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.autonomous.EncoderMovement;

@Autonomous
public class EncoderTest extends LinearOpMode {

    EncoderMovement movement = new EncoderMovement();

    @Override
    public void runOpMode() throws InterruptedException {

        movement.init(hardwareMap);

        waitForStart();



        if(opModeIsActive())
        {
            movement.encoderDriveLinear(0.3,60,10,1);
            sleep(1000);
            movement.encoderDriveLinear(0.3,60,10,-1);
            sleep(1000);
            movement.encoderDriveRotate(0.3,60,10,1);
            sleep(1000);
            movement.encoderDriveRotate(0.3,60,10,-1);
            sleep(1000);
            movement.encoderDriveStrafe(0.1,60,10,1);
            sleep(1000);
            movement.encoderDriveStrafe(0.1,60,10,-1);

        }

    }

}
