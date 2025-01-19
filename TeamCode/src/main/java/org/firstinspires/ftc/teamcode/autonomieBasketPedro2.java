package org.firstinspires.ftc.teamcode;


import android.transition.Slide;

import com.pedropathing.pathgen.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.config.RobotConstants;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.robot.Robot;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.config.subsystem.ClawSubsystem;
import org.firstinspires.ftc.teamcode.config.subsystem.SliderSubsystem;
import org.firstinspires.ftc.teamcode.config.subsystem.AxonSubsystem;


@Autonomous(name = "AutonomieBasketPedro2")
public class autonomieBasketPedro2 extends OpMode{

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    // Stagiul in care suntem momentan
    private int pathState;
    public double alpha = 2.8;
    public ClawSubsystem claw;
    public SliderSubsystem slider;
    public AxonSubsystem axon;

    private final Pose startPose = new Pose(7,84);
    private final Pose scorePosePreLoad = new Pose(13.5,129.5);
    private final Pose scorePoseControl = new Pose(50,89);
    private final Pose ReachFirstPose = new Pose(19,121.5);
    private final Pose scorePoseFirst = new Pose(10.5,126.5 );
    private final Pose NextToSecondPose = new Pose(58,131);
    private final Pose NextToSecondControl = new Pose(72,88);
    private final Pose PushSecond = new Pose(21,131);
    private final Pose goBackSecond = new Pose(58,131);
    private final Pose SlideLeft = new Pose(58,135.5);
    private final Pose pushThrid = new Pose(25,136);
    private final Pose parkPose = new Pose(60,97);
    private final Pose parkControl = new Pose(63,126);
    //
    private PathChain Score,ReachFirst,ScoreFirst,goNextToFirst,pushLast,goPark;
    public void buildPaths() {
        Score = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(startPose),new Point(scorePoseControl),new Point(scorePosePreLoad)))
                .setLinearHeadingInterpolation(Math.toRadians(0),Math.toRadians(-45))
                .setPathEndTimeoutConstraint(1.0)
                .build();
        ReachFirst = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePosePreLoad),new Point(ReachFirstPose)))
                .setLinearHeadingInterpolation(Math.toRadians(-45),Math.toRadians(0))
                .setPathEndTimeoutConstraint(5.0)
                .build();
        ScoreFirst = follower.pathBuilder()
                .addPath(new BezierLine(new Point(ReachFirstPose),new Point(scorePoseFirst)))
                .setLinearHeadingInterpolation(Math.toRadians(0),Math.toRadians(-45))
                .setPathEndTimeoutConstraint(1.0)
                .build();
        goNextToFirst = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePoseFirst),new Point(NextToSecondControl),new Point(NextToSecondPose)))
                .setLinearHeadingInterpolation(Math.toRadians(-45),Math.toRadians(0))
                .setPathEndTimeoutConstraint(1.0)
                .build();
        pushLast = follower.pathBuilder()
                .addPath(new BezierLine(new Point(NextToSecondPose),new Point(PushSecond)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new BezierLine(new Point(PushSecond), new Point(goBackSecond)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new BezierLine(new Point(goBackSecond),new Point(SlideLeft)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new BezierLine(new Point(SlideLeft),new Point(pushThrid)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        goPark = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(pushThrid),new Point(parkControl),new Point(parkPose)))
                .setLinearHeadingInterpolation(Math.toRadians(0),Math.toRadians(270))
                .setPathEndTimeoutConstraint(1.0)
                .build();
    }
    public void autonomousPathUpdate(){
        switch (pathState) {
            case 0:
                //aici adaug miscarea sliderelor
                axon.SetIntakePosition(RobotConstants.intakeDownPos);
                claw.CloseOuttake();
                slider.MoveOuttake(RobotConstants.outtakeSliderExtendPosition, 1);
                axon.SetOuttakePosition(RobotConstants.outtakeUpPos);
                follower.followPath(Score, true);
                setPathState(1);
                break;
            case 1:
                if(follower.getPose().getX() > (scorePosePreLoad.getX() - 2) && follower.getPose().getY() > (scorePosePreLoad.getY() - 2))
                {
                    claw.OpenOuttake();
                    axon.SetIntakePosition(RobotConstants.intakeUpPos);
                    claw.OpenIntake();
                    slider.MoveIntake(1740, 1);
                    axon.SetIntakePosition(0.2);
                    follower.setMaxPower(0.5);
                    follower.followPath(ReachFirst, true);
                    setPathState(2);
                } break;
            case 2:
                if(follower.getPose().getX() > (ReachFirstPose.getX() - 1) && follower.getPose().getY() < (ReachFirstPose.getY() + 1)){
                    slider.MoveOuttake(0,1);
                    if(slider.GetUpOuttakePosition()<100){

                    if(pathTimer.getElapsedTimeSeconds() > (1.6+2.8) && pathTimer.getElapsedTimeSeconds() < (2.5+2.8))
                        axon.SetIntakePosition(RobotConstants.intakeDownPos);
                    if(pathTimer.getElapsedTimeSeconds()>(2.6+2.8)) {
                        claw.CloseIntake();
                        axon.SetOuttakePosition(RobotConstants.outtakeMidPos);
                    }
                    if(pathTimer.getElapsedTimeSeconds()>(2.8+2.8)) {
                        axon.SetIntakePosition(RobotConstants.intakeUpPos);
                        if(pathTimer.getElapsedTimeSeconds()>(2.8+3.3))
                        claw.OpenIntake();
                        if(pathTimer.getElapsedTimeSeconds()>(2.8+3.8))
                            slider.MoveIntake(0,1);
                        if(pathTimer.getElapsedTimeSeconds()>(4.1+2.8))
                        setPathState(3);
                    }
                    follower.followPath(ScoreFirst);}
                } break;
            case 3: if(follower.getPose().getX() < (scorePosePreLoad.getX() + 1) && follower.getPose().getY() < (scorePosePreLoad.getY() + 1)){
                claw.CloseOuttake();
                if(pathTimer.getElapsedTimeSeconds()>2.3)
                    axon.SetIntakePosition(0.22);
                if(pathTimer.getElapsedTimeSeconds()>2.7) {
                    axon.SetOuttakePosition(RobotConstants.outtakeUpPos);

                    slider.MoveOuttake(RobotConstants.outtakeSliderExtendPosition, 1);
                }
                if(pathTimer.getElapsedTimeSeconds()>4.8)
                    claw.OpenOuttake();
                if(pathTimer.getElapsedTimeSeconds()>5.0) {
                    follower.setMaxPower(0.9);
                    setPathState(4);
                    follower.followPath(goNextToFirst,true);
                    }
                } break;

            case 4:
                if(follower.getPose().getX() > (NextToSecondPose.getX() - 1) && follower.getPose().getY() > (NextToSecondPose   .getY() - 1)){
                    slider.MoveOuttake(0,1);
                    axon.SetOuttakePosition(RobotConstants.outtakeMidPos);
                    follower.setMaxPower(0.9);
                    setPathState(5);
                    follower.followPath(pushLast,true);
                }break;
            case 5:
                if(follower.getPose().getX() < (pushThrid.getX() + 1) && follower.getPose().getY() > (pushThrid.getY() - 1)){
                setPathState(6);
                follower.followPath(goPark,true);
                }break;
            case 6:
                if(follower.getPose().getX() > (parkPose.getX() - 1) && follower.getPose().getY() < (parkPose.getY() + 1)) {
                    slider.MoveIntake(RobotConstants.intakeSliderRetractPosition,1);
                    setPathState(-1);
                }
                break;
        }
    }


    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();
        telemetry.addData("Secunde ",pathTimer.getElapsedTimeSeconds());
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        // Ghiara outtake telemetry
        telemetry.addData("Outtake Position", claw.GetGrabOuttakePosition());
        telemetry.addData("Outtake Angle", claw.GetGrabOuttakePosition() * 180);
        // Outtake slider telemetry
        telemetry.addData("OuttakeSliderPowerUp", slider.GetUpOuttakePower());
        telemetry.addData("OuttakeSliderPowerDown", slider.GetDownOuttakePower());
        telemetry.addData("OuttakeSliderPositionUp", slider.GetUpOuttakePosition());
        telemetry.addData("OuttakeSliderPositionDown", slider.GetDownOuttakePosition());
        telemetry.update();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();

        opmodeTimer.resetTimer();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        claw = new ClawSubsystem(hardwareMap);
        claw.InitIntake();
        claw.InitOuttake();
        claw.InitPivot();
        slider = new SliderSubsystem(hardwareMap);
        slider.InitIntake();
        slider.InitOuttake();
        axon = new AxonSubsystem(hardwareMap);
        axon.InitIntake();
        axon.InitOuttake(RobotConstants.outtakeMidPos);

        buildPaths();
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void stop() {
    }
}