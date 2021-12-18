package org.firstinspires.ftc.teamcode.drive.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

//Program temporar
@Autonomous
public class TestDetection extends LinearOpMode {

    BlockDetection bloc = new BlockDetection();

    public int caz;

    @Override
    public void runOpMode() throws InterruptedException {

        bloc.init(hardwareMap);

        while (!isStarted()) {
            if (bloc.Left_percent < 85 && bloc.Right_percent < 85) {
                telemetry.addData("Cazul", 4);
                caz = 4;
            }
            if (bloc.Left_percent <85 && bloc.Right_percent > 85) {
                telemetry.addData("Cazul", 1);
                caz = 1;
            }
            if (bloc.Left_percent > 85 && bloc.Right_percent > 85) {
                telemetry.addData("Cazul", 0);
                caz = 0;
            }
            telemetry.addData("Big percentage", bloc.Left_percent);
            telemetry.addData("Small percentage", bloc.Right_percent);
            telemetry.update();
        }

    }
}
