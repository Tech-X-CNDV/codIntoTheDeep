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
    private final Pose scorePosePreLoad = new Pose(12.5,128.5);
    private final Pose scorePoseControl = new Pose(50,89);
    private final Pose ReachFirstPose = new Pose(18,121);
    //10.5,126.5
    private final Pose scorePoseFirst = new Pose(10.5,126.5 );

    //test
    private final Pose ReachSecondPose = new Pose(18,131);
    //test
    private final Pose NextToSecondPose = new Pose(58,131);
    private final Pose NextToSecondControl = new Pose(72,88);
    private final Pose PushSecond = new Pose(21,131);
    private final Pose goBackSecond = new Pose(58,131);
    private final Pose SlideLeft = new Pose(58,135.5);
    private final Pose pushThrid = new Pose(25,136);
    private final Pose parkPose = new Pose(60,97);
    private final Pose parkControl = new Pose(63,126);
    //
    private PathChain Score,ReachFirst,ScoreFirst,goNextToFirst,pushLast,goPark,ScoreSecond,putSecond;
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
                .addPath(new BezierLine(new Point(ReachFirstPose),new Point(scorePosePreLoad)))
                .setLinearHeadingInterpolation(Math.toRadians(0),Math.toRadians(-45))
                .setPathEndTimeoutConstraint(1.0)
                .build();
        //test
        ScoreSecond = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePosePreLoad),new Point(ReachSecondPose)))
                .setLinearHeadingInterpolation(Math.toRadians(-45),Math.toRadians(1))
                .setPathEndTimeoutConstraint(1.0)
                .build();
        //test
        putSecond = follower.pathBuilder()
                .addPath(new BezierLine(new Point(ReachSecondPose),new Point(scorePosePreLoad)))
                .setLinearHeadingInterpolation(Math.toRadians(1),Math.toRadians(-45))
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
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                //aici adaug miscarea sliderelor
                //axon.SetIntakePosition(RobotConstants.intakeDownPos);
                claw.CloseOuttake();
                follower.setMaxPower(0.85);
                slider.MoveOuttake(RobotConstants.outtakeSliderExtendPosition, 1);
                axon.SetOuttakePosition(RobotConstants.outtakeUpPos);
                follower.followPath(Score, true);
                setPathState(1);
                break;
            case 1:
                if (follower.getPose().getX() > (scorePosePreLoad.getX() - 2) && follower.getPose().getY() > (scorePosePreLoad.getY() - 2)) {
                    claw.OpenOuttake();
                    if (pathTimer.getElapsedTimeSeconds() > 8) {
                        slider.MoveOuttake(0, 1);
                        axon.SetIntakePosition(RobotConstants.intakeMiddlePos);
                        axon.SetOuttakePosition(0.38);

                    }
                    if (slider.GetUpOuttakePosition() > 1000) {
                        axon.SetIntakePosition(0.2);
                    }
                    follower.setMaxPower(0.5);
                    setPathState(2);
                    follower.followPath(ReachFirst, true);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() > 0 && pathTimer.getElapsedTimeSeconds() < 3.4)
                        slider.MoveOuttake(0, 1);
                    if (pathTimer.getElapsedTimeSeconds() > 0.5 && pathTimer.getElapsedTimeSeconds() < 3.4) {
                        axon.SetIntakePosition(0.15);
                        claw.OpenIntake();
                        slider.MoveIntake(800, 1);
                        axon.SetOuttakePosition(0.38);
                    }
                    if (slider.GetUpOuttakePosition() < 100 && pathTimer.getElapsedTimeSeconds() > 3.4) {

                        if (pathTimer.getElapsedTimeSeconds() > 5.0 && pathTimer.getElapsedTimeSeconds() < (6.5))
                            axon.SetIntakePosition(0.27); //axondownpos-2
                        if (pathTimer.getElapsedTimeSeconds() > (6.5) && pathTimer.getElapsedTimeSeconds() < 7.4) {
                            claw.CloseIntake();
                            //axon.SetIntakePosition(0.38);
                        }
                        if (pathTimer.getElapsedTimeSeconds() > (7.4)) {
                            if (pathTimer.getElapsedTimeSeconds() < 8.3 && pathTimer.getElapsedTimeSeconds() > 7.4)
                                axon.SetIntakePosition(RobotConstants.intakeUpPos);
                            if (pathTimer.getElapsedTimeSeconds() < 8.8 && pathTimer.getElapsedTimeSeconds() > 8.3)
                                claw.OpenIntake();
                            if (pathTimer.getElapsedTimeSeconds() >9.1)
                                slider.MoveIntake(0, 1);
                            if (pathTimer.getElapsedTimeSeconds() > 9.4){

                                slider.MoveOuttake(0, 1);

                                claw.CloseOuttake();
                            }
                            if (pathTimer.getElapsedTimeSeconds() > 9.5) {
                                axon.SetIntakePosition(RobotConstants.intakeMiddlePos);
                                setPathState(3);
                                follower.followPath(ScoreFirst);
                            }
                        }

                    }
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() < 0.8 && pathTimer.getElapsedTimeSeconds() > 0.1)
                        axon.SetIntakePosition(RobotConstants.intakeMiddlePos);
                    if (pathTimer.getElapsedTimeSeconds() < 2.5 && pathTimer.getElapsedTimeSeconds() > 0.8)
                        slider.MoveOuttake(RobotConstants.outtakeSliderExtendPosition, 1);
                    if (pathTimer.getElapsedTimeSeconds() < 3.9 && pathTimer.getElapsedTimeSeconds() > 2.5)
                        axon.SetOuttakePosition(RobotConstants.outtakeUpPos);
                    if (pathTimer.getElapsedTimeSeconds() < 4.4 && pathTimer.getElapsedTimeSeconds() > 3.9) {
                        claw.OpenOuttake();
                        claw.OpenIntake();
                        axon.SetIntakePosition(RobotConstants.intakeMiddlePos);
                        setPathState(4);
                        //aici am pus in cos prima piesa din cele trei

                        follower.followPath(ScoreSecond);
                    }
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    slider.MoveOuttake(0, 1);
                    if (pathTimer.getElapsedTimeSeconds() < 2.9 && pathTimer.getElapsedTimeSeconds() > 0.9) {
                        slider.MoveIntake(740, 1);
                        axon.SetIntakePosition(0.21);
                    }
                    if (pathTimer.getElapsedTimeSeconds() < 3.1 && pathTimer.getElapsedTimeSeconds() > 0.9) {
                        axon.SetOuttakePosition(0.38);
                        axon.SetIntakePosition(RobotConstants.intakeDownPos);
                    }
                    if (pathTimer.getElapsedTimeSeconds() < 3.7 && pathTimer.getElapsedTimeSeconds() > 3.1)
                        claw.CloseIntake();
                    if (pathTimer.getElapsedTimeSeconds() < 4.9 && pathTimer.getElapsedTimeSeconds() > 3.7)
                        axon.SetIntakePosition(RobotConstants.intakeUpPos);
                    if (pathTimer.getElapsedTimeSeconds() < 5.9 && pathTimer.getElapsedTimeSeconds() > 4.9) {
                        claw.OpenIntake();
                        slider.MoveIntake(0, 0.75);
                    }
                    if (pathTimer.getElapsedTimeSeconds() < 6.7 && pathTimer.getElapsedTimeSeconds() > 6.5) {
                        //test
                        axon.SetIntakePosition(RobotConstants.intakeMiddlePos);
                        if (slider.GetIntakePosition() < 500)
                            claw.CloseOuttake();
                        //test
                        setPathState(5);
                        follower.followPath(putSecond, true);
                    }
                }
                break;
            /*
            //case 4 asta e bun insa vreau sa testez si sa iau pe al doilea din cele 3
            case 4:
                if(follower.getPose().getX() > (NextToSecondPose.getX() - 2) && follower.getPose().getY() > (NextToSecondPose   .getY() - 2)){
                    slider.MoveOuttake(0,1);
                    axon.SetOuttakePosition(0.38);
                    follower.setMaxPower(0.9);
                    setPathState(5);
                    follower.followPath(pushLast,true);
                }break;

                */
            case 5:
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() > 0.1 && pathTimer.getElapsedTimeSeconds() < 0.8) {
                        claw.CloseOuttake();
                        axon.SetIntakePosition(RobotConstants.intakeMiddlePos);
                    }
                    if (pathTimer.getElapsedTimeSeconds() > 0.8 && pathTimer.getElapsedTimeSeconds() < 4.2)
                        slider.MoveOuttake(RobotConstants.outtakeSliderExtendPosition, 1);
                    if (pathTimer.getElapsedTimeSeconds() > 4.2 && pathTimer.getElapsedTimeSeconds() < 4.5)
                        axon.SetOuttakePosition(RobotConstants.outtakeUpPos);
                    if (pathTimer.getElapsedTimeSeconds() > 4.9 && pathTimer.getElapsedTimeSeconds() < 6.1) {
                        claw.OpenOuttake();
                    }
                    if (pathTimer.getElapsedTimeSeconds() > 6.1 && pathTimer.getElapsedTimeSeconds() < 6.2){
                        follower.setMaxPower(1);
                        setPathState(6);
                        follower.followPath(goPark,true);}
                    }
                    break;

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
        //axon.InitIntake();
        claw = new ClawSubsystem(hardwareMap);
        claw.InitIntake();
        claw.InitOuttake();
        claw.InitPivot();
        slider = new SliderSubsystem(hardwareMap);
        slider.InitIntake(true);
        slider.InitOuttake(true);
        axon = new AxonSubsystem(hardwareMap);
        axon.SetIntakePosition(RobotConstants.intakeDownPos);
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