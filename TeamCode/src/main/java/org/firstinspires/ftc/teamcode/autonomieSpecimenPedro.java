package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.config.RobotConstants;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.pedropathing.util.Constants;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

import org.firstinspires.ftc.teamcode.config.subsystem.ClawSubsystem;
import org.firstinspires.ftc.teamcode.config.subsystem.SliderSubsystem;
import org.firstinspires.ftc.teamcode.config.subsystem.AxonSubsystem;


@Autonomous(name = "AutonomieBlue")
public class autonomieSpecimenPedro extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    // Stagiul in care suntem momentan
    private int pathState;

    public ClawSubsystem claw;
    public SliderSubsystem slider;
    public AxonSubsystem axon;


    /** Start Pose of our robot */
    private final Pose startPose = new Pose(8, 60, Math.toRadians(180));

    // Scor la specimen
    private final Pose scorePosePreLoad = new Pose(40, 66, Math.toRadians(180));
    private final Pose scorePose1 = new Pose(40, 71, Math.toRadians(180));
    private final Pose scorePose2 = new Pose(40, 76, Math.toRadians(180));
    private final Pose scorePose3 = new Pose(40, 81, Math.toRadians(180));

    // Pregatire pentru specimene
    private final Pose specimenReadyPos = new Pose(62, 36, Math.toRadians(0));
    private final Pose specimenReadyControl = new Pose(20, 33);

    // Specimen 1
    private final Pose specimen1Pos = new Pose(62, 23, Math.toRadians(0));
    private final Pose specimen1HPlayer = new Pose(10, 23, Math.toRadians(0));

    // Specimen 2
    private final Pose specimen2Pos = new Pose(62, 13, Math.toRadians(0));
    private final Pose specimen2HPlayer = new Pose(10, 13, Math.toRadians(0));

    // Specimen 3
    private final Pose specimen3Pos = new Pose(62, 6, Math.toRadians(0));
    private final Pose specimen3HPlayer = new Pose(10, 6, Math.toRadians(0));

    // Pozitia de parcare
    private final Pose parkPose = new Pose(59, 95, Math.toRadians(180));
    private final Pose parkControlPose1 = new Pose(11, 103);
    private final Pose  parkControlPose2 = new Pose(70, 129);

    // Aici stocam traiectoriile robotului
    private Path scorePreload, park;
    private PathChain specimenReady, specimen, subScore1, subScore2, subScore3, hPlayer1, hPlayer2;

    public void buildPaths() {

        /* Aici punem preload-ul cu care incepem. Folosim BezierLine pentru o linie dreapta */
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePosePreLoad)));
        scorePreload.setConstantHeadingInterpolation(startPose.getHeading());

        /* Aici ne pregatim pentru a impinge specimenele. Folosim BezierCurve pentru o linie curbata. */
        specimenReady = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePosePreLoad), new Point(specimenReadyControl), new Point(specimenReadyPos)))
                .setLinearHeadingInterpolation(scorePosePreLoad.getHeading(), specimenReadyPos.getHeading())
                .build();

        specimen = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimenReadyPos), new Point(specimen1Pos)))
                .setConstantHeadingInterpolation(specimen1Pos.getHeading())
                .addPath(new BezierLine(new Point(specimen1Pos), new Point(specimen1HPlayer)))
                .setConstantHeadingInterpolation(specimen1Pos.getHeading())
                .addPath(new BezierLine(new Point(specimen1HPlayer), new Point(specimen1Pos)))
                .setConstantHeadingInterpolation(specimen1Pos.getHeading())
                .addPath(new BezierLine(new Point(specimen1Pos), new Point(specimen2Pos)))
                .setConstantHeadingInterpolation(specimen2Pos.getHeading())
                .addPath(new BezierLine(new Point(specimen2Pos), new Point(specimen2HPlayer)))
                .setConstantHeadingInterpolation(specimen2Pos.getHeading())
                .addPath(new BezierLine(new Point(specimen2HPlayer), new Point(specimen2Pos)))
                .setConstantHeadingInterpolation(specimen2Pos.getHeading())
                .addPath(new BezierLine(new Point(specimen2Pos), new Point(specimen3Pos)))
                .setConstantHeadingInterpolation(specimen3Pos.getHeading())
                .addPath(new BezierLine(new Point(specimen3Pos), new Point(specimen3HPlayer)))
                .setConstantHeadingInterpolation(specimen3Pos.getHeading())
                .build();

        subScore1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimen3Pos), new Point(scorePose1)))
                .setLinearHeadingInterpolation(specimen3Pos.getHeading(), scorePose1.getHeading())
                .build();

        subScore2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimen3Pos), new Point(scorePose2)))
                .setLinearHeadingInterpolation(specimen3Pos.getHeading(), scorePose2.getHeading())
                .build();

        subScore3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimen3Pos), new Point(scorePose3)))
                .setLinearHeadingInterpolation(specimen3Pos.getHeading(), scorePose3.getHeading())
                .build();

        hPlayer1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose1), new Point(specimen3HPlayer)))
                .setLinearHeadingInterpolation(scorePose1.getHeading(), specimen3HPlayer.getHeading())
                .build();

        hPlayer2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose2), new Point(specimen3HPlayer)))
                .setLinearHeadingInterpolation(scorePose2.getHeading(), specimen3HPlayer.getHeading())
                .build();

        park = new Path(new BezierCurve(new Point(scorePose3), /* Control Point */ new Point(parkControlPose1), new Point(parkControlPose2), new Point(parkPose)));
        park.setLinearHeadingInterpolation(scorePose3.getHeading(), parkPose.getHeading());
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                claw.CloseOuttake();
                //TODO make specimen extend slider
                slider.MoveOuttake(RobotConstants.outtakeSliderExtendPosition, 1);
                axon.SetOuttakePosition(RobotConstants.outtakeBehindPos);
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:
                /* Urmatoarea miscare incepe doar dupa ce robotul este la 1inch dinstanta de cealalta */
                if (follower.getPose().getX() > (scorePosePreLoad.getX() - 1) && follower.getPose().getY() > (scorePosePreLoad.getY() - 1)) {
                    // TODO make specimen retract slider
                    slider.MoveOuttake(RobotConstants.outtakeSliderRetractPosition, 1);
                    claw.OpenOuttake();
                    follower.followPath(specimenReady, true);
                    setPathState(2);
                }
                break;
            case 2:
                /* Urmatoarea miscare incepe doar dupa ce robotul este la 1inch dinstanta de cealalta */
                if (follower.getPose().getX() > (specimenReadyPos.getX() - 1) && follower.getPose().getY() < (specimenReadyPos.getY() + 1)) {
                    follower.followPath(specimen, true);
                    setPathState(3);
                }
                break;
            case 3:
                /* Urmatoarea miscare incepe doar dupa ce robotul este la 1inch dinstanta de cealalta */
                if (follower.getPose().getX() < (specimen3HPlayer.getX() + 1) && follower.getPose().getY() < (specimen3HPlayer.getY() + 1)) {
                    claw.CloseOuttake();
                    follower.followPath(subScore1, true);
                    setPathState(4);
                }
                break;
            case 4:
                // TODO make specimen extend slider
                slider.MoveOuttake(RobotConstants.outtakeSliderExtendPosition, 1);
                /* Urmatoarea miscare incepe doar dupa ce robotul este la 1inch dinstanta de cealalta */
                if (follower.getPose().getX() > (scorePose1.getX() - 1) && follower.getPose().getY() > (scorePose1.getY() - 1)) {
                    // TODO make specimen retract slider
                    slider.MoveOuttake(RobotConstants.outtakeSliderRetractPosition, 1);
                    claw.OpenOuttake();
                    follower.followPath(hPlayer1, true);
                    setPathState(5);
                }
                break;
            case 5:
                /* Urmatoarea miscare incepe doar dupa ce robotul este la 1inch dinstanta de cealalta */
                if (follower.getPose().getX() < (specimen3HPlayer.getX() + 1) && follower.getPose().getY() < (specimen3HPlayer.getY() + 1)) {
                    claw.CloseOuttake();
                    follower.followPath(subScore2, true);
                    setPathState(6);
                }
                break;
            case 6:
                // TODO make specimen extend slider
                slider.MoveOuttake(RobotConstants.outtakeSliderExtendPosition, 1);
                /* Urmatoarea miscare incepe doar dupa ce robotul este la 1inch dinstanta de cealalta */
                if (follower.getPose().getX() > (scorePose2.getX() - 1) && follower.getPose().getY() > (scorePose2.getY() - 1)) {
                    // TODO make specimen retract slider
                    slider.MoveOuttake(RobotConstants.outtakeSliderRetractPosition, 1);
                    claw.OpenOuttake();
                    follower.followPath(hPlayer2, true);
                    setPathState(7);
                }
                break;
            case 7:
                /* Urmatoarea miscare incepe doar dupa ce robotul este la 1inch dinstanta de cealalta */
                if (follower.getPose().getX() < (specimen3HPlayer.getX() + 1) && follower.getPose().getY() < (specimen3HPlayer.getY() + 1)) {
                    claw.CloseOuttake();
                    follower.followPath(subScore3, true);
                    setPathState(8);
                }
                break;
            case 8:
                // TODO make specimen extend slider
                slider.MoveOuttake(RobotConstants.outtakeSliderExtendPosition, 1);
                /* Urmatoarea miscare incepe doar dupa ce robotul este la 1inch dinstanta de cealalta */
                if (follower.getPose().getX() > (scorePose3.getX() - 1) && follower.getPose().getY() > (scorePose3.getY() - 1)) {
                    // TODO make specimen retract slider
                    slider.MoveOuttake(RobotConstants.outtakeSliderRetractPosition, 1);
                    claw.OpenOuttake();
                    follower.followPath(park, true);
                    setPathState(9);
                }
                break;
            case 9:
                claw.CloseOuttake();
                axon.SetOuttakePosition(RobotConstants.outtakeMidPos);
                slider.MoveOuttake(RobotConstants.outtakeSliderRetractPosition, 1);
                /* Urmatoarea miscare incepe doar dupa ce robotul este la 1inch dinstanta de cealalta */
                if (follower.getPose().getX() > (parkPose.getX() - 1) && follower.getPose().getY() > (parkPose.getY() - 1)) {
                    /* Setam stagiul la ceva care nu exista pentru a opri miscarile */
                    setPathState(-1);
                }
                break;
        }
    }

    /** Aceasta functie seteaza stagiul la care suntem si reseteaza timer-ul (optional) **/
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
        axon.InitOuttake();

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
