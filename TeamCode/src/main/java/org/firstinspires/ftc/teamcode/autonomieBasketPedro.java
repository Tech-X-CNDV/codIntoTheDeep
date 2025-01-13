package org.firstinspires.ftc.teamcode;


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


@Autonomous(name = "AutonomieBasketPedro")
public class autonomieBasketPedro extends OpMode{

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    // Stagiul in care suntem momentan
    private int pathState;

    public ClawSubsystem claw;
    public SliderSubsystem slider;
    public AxonSubsystem axon;
    private final Pose startPose = new Pose(8, 85, Math.toRadians(180));
    private final Pose scorePosePreLoad = new Pose(13, 130, Math.toRadians(180));
    private final Pose scorePoseControl1 = new Pose(48.6,77.7);
    private final Pose pickFirstSample = new Pose(37,121);
    private final Pose goBackFirst = new Pose(13,130);
    private final Pose goSecondSample = new Pose(37,132);
    private final Pose goBackSecond = new Pose(13,130);

    private final Pose thirdFirstLine = new Pose(60,129);
    private final Pose thirdSecondLine = new Pose(60,138);
    private final Pose thirdThirdLine = new Pose(17,138);

    private final Pose parkPose = new Pose(60,94);
    private final Pose parkControl = new Pose(69,121);

    //
    private PathChain scorePreload,goToFirst,goToBasketFirst,goToSecond,goToBasketSecond,pushTheThird,park;
    public void buildPaths() {
        scorePreload = follower.pathBuilder()//line 1
                .addPath(new BezierCurve(new Point(startPose),new Point(scorePoseControl1),new Point(scorePosePreLoad)))
                .setLinearHeadingInterpolation(180,-45)
                .setPathEndTimeoutConstraint(2.0)
                .build();
        goToFirst = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePosePreLoad),new Point(pickFirstSample)))
                .setLinearHeadingInterpolation(-45,360)
                .setPathEndTimeoutConstraint(2.0)
                .build();
        goToBasketFirst = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickFirstSample),new Point(goBackFirst)))
                .setLinearHeadingInterpolation(360,-45)
                .setPathEndTimeoutConstraint(2.0)
                .build();
        goToSecond = follower.pathBuilder()
                .addPath(new BezierLine(new Point(goBackFirst),new Point(goSecondSample)))
                .setLinearHeadingInterpolation(-45,360)
                .setPathEndTimeoutConstraint(2.0)
                .build();
        goToBasketSecond = follower.pathBuilder()
                .addPath(new BezierLine(new Point(goSecondSample),new Point(goBackSecond)))
                .setLinearHeadingInterpolation(360,-45)
                .setPathEndTimeoutConstraint(2.0)
                .build();
        pushTheThird = follower.pathBuilder()
                .addPath(new BezierLine(new Point(goBackSecond), new Point(thirdFirstLine)))
                .setLinearHeadingInterpolation(-45,360)
                .addPath(new BezierLine(new Point(thirdFirstLine), new Point(thirdSecondLine)))
                .setConstantHeadingInterpolation(0)
                .addPath(new BezierLine(new Point(thirdSecondLine), new Point(thirdThirdLine)))
                .setConstantHeadingInterpolation(0)
                .setPathEndTimeoutConstraint(2.0)
                .build();
        park = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(thirdThirdLine), new Point(parkControl), new Point(parkPose)))
                .setLinearHeadingInterpolation(0,-90)
                .setPathEndTimeoutConstraint(2.0)
                .build();
    }
    public void autonomousPathUpdate(){
        switch (pathState) {
            case 0:

                claw.CloseOuttake();
                slider.MoveOuttake(RobotConstants.outtakeSliderExtendPosition, 1);
                axon.SetOuttakePosition(RobotConstants.outtakeUpPos);
                follower.followPath(scorePreload, true);
                if(slider.GetUpOuttakePosition() > RobotConstants.outtakeSliderExtendPosition)
                    setPathState(1);
                break;
            case 1:

                if (follower.getPose().getX() > (scorePosePreLoad.getX() - 1) && follower.getPose().getY() > (scorePosePreLoad.getY() - 1)) {
                    claw.OpenOuttake();
                    if(pathTimer.getElapsedTimeSeconds()>2.1)
                    slider.MoveOuttake(RobotConstants.outtakeSliderRetractPosition, 1);
                    follower.followPath(goToFirst, true);
                    setPathState(2);

                }
                break;
            case 2:
                if(follower.getPose().getHeading()>(pickFirstSample.getX()-1) && follower.getPose().getY() > (pickFirstSample.getY() - 1)){
                    claw.OpenIntake();
                    axon.SetIntakePosition(RobotConstants.intakeDownPos);
                    //doar acum
                    axon.SetOuttakePosition(RobotConstants.outtakeMidPos);
                    //doar acum
                    claw.Rotated();
                    claw.CloseIntake();
                    claw.InitialRot();
                    axon.SetIntakePosition(RobotConstants.intakeUpPos);
                    claw.CloseOuttake();
                    pathTimer.resetTimer();
                    if(pathTimer.getElapsedTimeSeconds()>3.1)
                    slider.MoveOuttake(RobotConstants.outtakeSliderExtendPosition, 1);
                    axon.SetOuttakePosition(RobotConstants.outtakeUpPos);
                    follower.followPath(goToBasketFirst, true);
                    setPathState(3);
                }
            case 3:
                if(follower.getPose().getHeading()>(goBackFirst.getX()-1) && follower.getPose().getY() > (goBackFirst.getY() - 1)){
                    claw.OpenOuttake();
                    pathTimer.resetTimer();
                    if(pathTimer.getElapsedTimeSeconds()>2.1)
                        slider.MoveOuttake(RobotConstants.outtakeSliderRetractPosition, 1);
                    follower.followPath(park, true);
                    setPathState(4);
                }
            case 4:
                if(follower.getPose().getHeading()>(parkPose.getX()-1) && follower.getPose().getY() > (parkPose.getY() - 1)){
                    setPathState(-1);
                }
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
        axon.InitOuttake(RobotConstants.outtakeUpPos);

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