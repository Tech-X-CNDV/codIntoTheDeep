package org.firstinspires.ftc.teamcode;
 
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
 
@Autonomous(name = "autoRED_COS")
public class autonomieRED_COS extends LinearOpMode{
    CRobo robot = new CRobo();
    @Override
    public void runOpMode() throws InterruptedException {
        robot.outtakeAxonRight.setPosition(1);
        robot.outtakeAxonLeft.setPosition(1);
        robot.servoGhiaraOut.setPosition(0.3);
        robot.outtakeSliderUp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.outtakeSliderUp.setTargetPosition(0);
        robot.outtakeSliderUp.setMode(DcMotor.RunMode.RUN_TO_POSITION);
 
        robot.outtakeSliderDown.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.outtakeSliderDown.setTargetPosition(0);
        robot.outtakeSliderDown.setMode(DcMotor.RunMode.RUN_TO_POSITION);
 
        robot.outtakeSliderUp.setPower(1);
        robot.outtakeSliderDown.setPower(1);
        MecanumDrive drive = new MecanumDrive(hardwareMap,new Pose2d(0,0,0));
        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(-36, -56, Math.toRadians(90)))
                        .turn(Math.toRadians(-45))
                        .strafeTo(new Vector2d(-46.7,-45.7))
                        .build()
        );
        robot.outtakeSliderUp.setTargetPosition(1500);
        robot.outtakeSliderDown.setTargetPosition(1500);
        if(1500-robot.outtakeSliderUp.getCurrentPosition()<2) {
            robot.servoGhiaraOut.setPosition(0);
            robot.outtakeSliderUp.setTargetPosition(0);
            robot.outtakeSliderDown.setTargetPosition(0);
        }
        //de la cos sa ia a treia piesa
 
        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(-36, -56, Math.toRadians(90)))
                        .turn(Math.toRadians(45))
                        .strafeTo(new Vector2d(-47.3,-40.2))
                        .build()
        );
        //cobora axonul si prinde piesa
        robot.intakeAxonLeft.setPosition();
        robot.intakeAxonRight.setPosition();
    }
}