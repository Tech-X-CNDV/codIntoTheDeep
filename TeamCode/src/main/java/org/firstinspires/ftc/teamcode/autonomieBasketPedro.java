package org.firstinspires.ftc.teamcode;


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
    private final Pose pushFirstPose = new Pose(8,123.5);
    private final Pose pushFirstPoseControl1 = new Pose(61.5,57);
    private final Pose pushFirstPoseControl2 = new Pose(144,138);
    private final Pose pushFirstPoseControl3 = new Pose(69, 120);
    private final Pose pushSecondPart1 = new Pose(64,122);
    private final Pose pushSecondPart2 = new Pose(64,131);
    private final Pose pushSecondPart3 = new Pose(13,131);
    private final Pose pushThirdPart1 = new Pose(64,131);
    private final Pose pushThirdPart2 = new Pose(64,136);
    private final Pose pushThirdPart3 = new Pose(16,136);
    private final Pose getUp = new Pose (64,136);
    private final Pose Parcare = new Pose(64,94);
    //
    private PathChain scorePreload,pushFirst,pushSecond,pushThird,Park;
    public void buildPaths() {
        scorePreload = follower.pathBuilder()//line 1
                .addPath(new BezierCurve(new Point(startPose),new Point(scorePoseControl1),new Point(scorePosePreLoad)))
                .setLinearHeadingInterpolation(180,-45)
                .setPathEndTimeoutConstraint(2.0)
                .build();
        pushFirst = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePosePreLoad),new Point(pushFirstPoseControl1),new Point(pushFirstPoseControl2),new Point(pushFirstPoseControl3),new Point(pushFirstPose)))
                .setTangentHeadingInterpolation()
                .setPathEndTimeoutConstraint(2.0)
                .build();
        pushSecond = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pushFirstPose),new Point(pushSecondPart1)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(new BezierLine(new Point(pushSecondPart1),new Point(pushSecondPart2)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(new BezierLine(new Point(pushSecondPart2),new Point(pushSecondPart3)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        pushThird = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pushThirdPart1),new Point(pushThirdPart2)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(new BezierLine(new Point(pushThirdPart2),new Point(pushThirdPart3)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        Park = follower.pathBuilder()//getUp,Parcare
                .addPath(new BezierLine(new Point(getUp),new Point(Parcare)))
                .setLinearHeadingInterpolation(Math.toRadians(180),Math.toRadians(-90))
                .build();

    }
    public void autonomousPathUpdate(){
        switch (pathState) {
            case 0:
                claw.CloseOuttake();
                slider.MoveOuttake(RobotConstants.outtakeSliderExtendPosition, 1);
                axon.SetOuttakePosition(RobotConstants.outtakeMidPos);
                follower.followPath(scorePreload, true);
                if(slider.GetUpOuttakePosition() > RobotConstants.outtakeSliderExtendPosition)
                    setPathState(1);
                break;
            case 1:
                if (follower.getPose().getX() > (scorePosePreLoad.getX() - 1) && follower.getPose().getY() > (scorePosePreLoad.getY() - 1)) {
                    slider.MoveOuttake(RobotConstants.outtakeSliderRetractPosition, 1);
                        claw.OpenOuttake();
                        follower.followPath(pushFirst, true);
                        setPathState(2);

                }
                break;
            case 2:
                if (follower.getPose().getX() > (pushFirstPose.getX() - 2) && follower.getPose().getY() < (pushFirstPose.getY() + 2)) {
                    follower.followPath(pushSecond, true);
                    setPathState(3);
                }
                break;
            case 3:
                if (follower.getPose().getX() < (pushSecondPart3.getX() + 2) && follower.getPose().getY() < (pushSecondPart3.getY() + 2)) {
                    follower.followPath(pushThird, true);
                    setPathState(4);
                }
                break;
            case 4:
                if (follower.getPose().getX() < (pushThirdPart3.getX() + 2) && follower.getPose().getY() < (pushThirdPart3.getY() + 2)) {
                    follower.followPath(Park, true);
                    setPathState(5);
                }
                break;
            case 5:
                if (follower.getPose().getX() < (Parcare.getX() + 2) && follower.getPose().getY() < (Parcare.getY() + 2)) {
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
        axon.InitOuttake(RobotConstants.outtakeBehindPos);

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