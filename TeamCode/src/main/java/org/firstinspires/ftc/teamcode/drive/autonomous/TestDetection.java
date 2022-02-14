package org.firstinspires.ftc.teamcode.drive.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

//Program temporar
@Autonomous
public class TestDetection extends LinearOpMode {

    BlockDetection bloc = new BlockDetection();

    public int caz=1;

    @Override
    public void runOpMode() throws InterruptedException {

        bloc.init(hardwareMap);

        while (!isStarted()) {
            if (bloc.Right_percent < 100) {
                telemetry.addData("Cazul", 3);
                caz = 3;
            }
            if  (bloc.Left_percent < 100) {
                telemetry.addData("Cazul", 2);
                caz = 2;
            }
            if (bloc.Left_percent > 80 && bloc.Right_percent > 80){
               telemetry.addData("Cazul",1);
                caz = 1;
            }
        }

        waitForStart();
        while(opModeIsActive()){
            telemetry.addData("cazul",caz);

            telemetry.addData("LeftPercent", bloc.Left_percent);
            telemetry.addData("RightPercent", bloc.Right_percent);
            telemetry.update();
        }

    }
}