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

    private final Pose startPose = new Pose(7,84);
    private final Pose scorePosePreLoad = new Pose(14,129);
    private final Pose scorePoseControl = new Pose(50,89);

    private final Pose moveNextToFirst = new Pose(61,122);
    private final Pose moveNextToFirstControl = new Pose(39,83);

    private final Pose pushFirst = new Pose(12,122);

    private final Pose pushSecond1 = new Pose(61,122);
    private final Pose pushSecond2 = new Pose(61,131);
    private final Pose pushSecond3 = new Pose(20,131);

    private final Pose pushThird1 = new Pose(61,131);
    private final Pose pushThird2 = new Pose(61,137.5);
    private final Pose pushThird3 = new Pose(30,137.5);

    private final Pose parkPose = new Pose(60,94);
    private final Pose parkControl = new Pose(63,126);

    //
    private PathChain Score,goToFirst,goToBasketFirst,goPushSecond,goPushThird,goPark;
    public void buildPaths() {
        Score = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(startPose),new Point(scorePoseControl),new Point(scorePosePreLoad)))
                .setLinearHeadingInterpolation(Math.toRadians(-180),Math.toRadians(-45))
                .build();
        goToFirst = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePosePreLoad),new Point(moveNextToFirstControl),new Point(moveNextToFirst)))
                .setLinearHeadingInterpolation(Math.toRadians(-45),Math.toRadians(0))
                .build();
        goToBasketFirst = follower.pathBuilder()
                .addPath(new BezierLine(new Point(moveNextToFirst),new Point(pushFirst)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        goPushSecond = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pushFirst),new Point(pushSecond1)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new BezierLine(new Point(pushSecond1),new Point(pushSecond2)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new BezierLine(new Point(pushSecond2),new Point(pushSecond3)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        goPushThird = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pushSecond3),new Point(pushThird1)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new BezierLine(new Point(pushThird1),new Point(pushThird2)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new BezierLine(new Point(pushThird2),new Point(pushThird3)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        goPark = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(pushThird3),new Point(parkControl),new Point(parkPose)))
                .setLinearHeadingInterpolation(Math.toRadians(0),Math.toRadians(270))
                .build();
    }
    public void autonomousPathUpdate(){
        switch (pathState) {
            case 0:
                //aici adaug miscarea sliderelor
                claw.CloseOuttake();
                follower.followPath(Score, true);
                setPathState(1);
                break;
            case 1:
                if(follower.getPose().getX() > (scorePosePreLoad.getX() - 1) && follower.getPose().getY() > (scorePoseControl.getY() - 1))
                    claw.OpenOuttake();
                follower.followPath(goToFirst, true);
                setPathState(2);
            case 2:
                if(follower.getPose().getX() > (moveNextToFirst.getX() - 1) && follower.getPose().getY() < (moveNextToFirst.getY() + 1))
                    follower.followPath(goToBasketFirst,true);
                setPathState(3);
            case 3:
                if(follower.getPose().getX() < (pushFirst.getX() + 1) && follower.getPose().getY() > (pushFirst.getY() - 1))
                    follower.followPath(goPushSecond,true);
                setPathState(4);
            case 4:
                if(follower.getPose().getX() > (pushSecond3.getX() - 1) && follower.getPose().getY() > (pushSecond3.getY() - 1))
                    follower.followPath(goPushThird,true);
                setPathState(5);
            case 5:
                if(follower.getPose().getX() > (pushThird3.getX() - 1) && follower.getPose().getY() > (pushThird3.getY() - 1))
                    follower.followPath(goPark,true);
                setPathState(6);
            case 6:
                if(follower.getPose().getX() > (parkPose.getX() - 1) && follower.getPose().getY() < (parkPose.getY() + 1))
                    setPathState(-1);
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