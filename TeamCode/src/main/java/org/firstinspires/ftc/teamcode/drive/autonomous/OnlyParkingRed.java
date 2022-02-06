package org.firstinspires.ftc.teamcode.drive.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.structure.ArmAssist;

@Disabled
@Autonomous(name = "OnlyParkingRed", group = "Autonoame basic")
public class OnlyParkingRed extends LinearOpMode {

    ArmAssist assist = new ArmAssist();

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d starting_position = new Pose2d(10, -65, 0);

        drive.setPoseEstimate(starting_position);

        Trajectory trajp = drive.trajectoryBuilder(starting_position)
                .forward(10)
                .build();

        Trajectory traj1 = drive.trajectoryBuilder(starting_position)
                .strafeLeft(40)
                .build();

        Trajectory traj2 =drive.trajectoryBuilder(traj1.end())
                .strafeRight(40)
                .splineTo(new Vector2d(40, -65), 0)
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .strafeLeft(20)
                .build();

        waitForStart();
        if (opModeIsActive())
        {

            drive.followTrajectory(trajp);
            //drive.followTrajectory(traj1);
            //assist.Assist();
           // drive.followTrajectory(traj2);
           // drive.followTrajectory(traj);

        }

    }
}
