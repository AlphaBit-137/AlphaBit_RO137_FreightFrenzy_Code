package org.firstinspires.ftc.teamcode.drive.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.structure.ArmAssist;

@Disabled
@Autonomous(group = "Autonoame Basic")
public class OnlyCaruselBlue extends LinearOpMode {

    //  BlockDetection test = new BlockDetection();
    int caz;
    ArmAssist asist = new ArmAssist();


    public DcMotor carusel;
    @Override
    public void runOpMode() throws InterruptedException {

        asist.init(hardwareMap);

        while (!isStarted()) {
               /* if (test.Left_percent < 85 && test.Right_percent < 85) {
                    telemetry.addData("Cazul", 4);
                    caz = 3;
                }
                if (test.Left_percent < 85 && test.Right_percent > 85) {
                    telemetry.addData("Cazul", 1);
                    caz = 1;
                }
                if (test.Left_percent > 85 && test.Right_percent > 85) {
                    telemetry.addData("Cazul", 0);
                    caz = 2;
                }*/

        }
        SampleMecanumDrive drivetrain = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(10, -8, Math.toRadians(90));

        Vector2d vector = new Vector2d(-15,45);
        Vector2d vectori = new Vector2d(-15,40);
        Vector2d vectorii = new Vector2d(-15,35);
        Vector2d vector2 = new Vector2d(-59,35);

        Pose2d pose;
        waitForStart();
        Trajectory goforward = drivetrain.trajectoryBuilder(new Pose2d (-35,60,Math.toRadians(90)))
                .forward(10)
                .build();
        Trajectory spline1caz1 =drivetrain.trajectoryBuilder(goforward.end())
                .splineTo(vector,Math.toRadians(90))
                .build();

        Trajectory spline1caz2 = drivetrain.trajectoryBuilder(new Pose2d(-35,60,90))
                .splineTo(vectori,Math.toRadians(90))
                .build();


        Trajectory spline1caz3 = drivetrain.trajectoryBuilder(new Pose2d(-35,60,90))
                .splineTo(vectorii,Math.toRadians(90))
                .build();

        Trajectory spline2 = drivetrain.trajectoryBuilder(spline1caz1.end())
                .splineTo(vector2,Math.toRadians(0))
                .build();
        Trajectory GoForward = drivetrain.trajectoryBuilder(new Pose2d(-59,-60,-180))
                .forward(20)
                .build();




        if(opModeIsActive()){
            // if(caz == 1){
            drivetrain.followTrajectory(goforward);
            drivetrain.followTrajectory(spline1caz1);
            asist.Assist_Strong();
               /* }else if(caz == 2){
                    drivetrain.followTrajectory(spline1caz2);
                    asist.SwitchToUP();
                }else if(caz == 3){
                    drivetrain.followTrajectory(spline1caz3);
                    asist.SwitchToUP();
                }*/
            drivetrain.followTrajectory(spline2);
            // carusel.setPower(0.6);





        }
        if(isStopRequested()){
            return;
        }
    }

}
