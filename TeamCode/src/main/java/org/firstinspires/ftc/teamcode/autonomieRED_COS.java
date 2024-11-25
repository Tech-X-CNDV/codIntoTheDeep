package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Autonomous(name = "autoRED_COS")

public class autonomieRED_COS extends LinearOpMode{

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap,new Pose2d(0,0,0));
        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(-36, -56, Math.toRadians(90)))
                        .splineTo(new Vector2d(-7.5,-34),Math.toRadians(90))
                        .build()
        );
    }
}
