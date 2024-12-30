package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.config.RobotConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.*;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;
import org.firstinspires.ftc.teamcode.config.subsystem.ClawSubsystem;
import org.firstinspires.ftc.teamcode.config.subsystem.SliderSubsystem;
import org.firstinspires.ftc.teamcode.config.subsystem.AxonSubsystem;


@Autonomous(name = "AutonomieBlue")
public class autonomieBLUEPedro extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    // Stagiul in care suntem momentan
    private int pathState;

    public ClawSubsystem claw;
    public SliderSubsystem slider;
    public AxonSubsystem axon;


    /** Start Pose of our robot */
    private final Pose startPose = new Pose(6, 60, Math.toRadians(180));

    // Scor la specimen
    private final Pose scorePose = new Pose(41, 66, Math.toRadians(180));

    // Pregatire pentru specimene
    private final Pose specimenReadyPos = new Pose(62, 36, Math.toRadians(0));
    private final Pose specimenReadyControl = new Pose(20, 33, Math.toRadians(0));

    // Specimen 1
    private final Pose specimen1Pos = new Pose(62, 23, Math.toRadians(0));
    private final Pose specimen1HPlayer = new Pose(8, 23, Math.toRadians(0));

    // Specimen 2
    private final Pose specimen2Pos = new Pose(62, 13, Math.toRadians(0));
    private final Pose specimen2HPlayer = new Pose(8, 13, Math.toRadians(0));

    // Specimen 3
    private final Pose specimen3Pos = new Pose(62, 7, Math.toRadians(0));
    private final Pose specimen3HPlayer = new Pose(8, 7, Math.toRadians(0));

    // Pozitia de parcare
    private final Pose parkPose = new Pose(65, 95, Math.toRadians(180));
    private final Pose parkControlPose = new Pose(27, 135, Math.toRadians(180));

    // Aici stocam traiectoriile robotului
    private Path scorePreload, park;
    private PathChain specimenReady, specimen1, specimen2, specimen3, subScore, hPlayer;

    public void buildPaths() {

        /* Aici punem preload-ul cu care incepem. Folosim BezierLine pentru o linie dreapta */
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePose)));
        scorePreload.setConstantHeadingInterpolation(startPose.getHeading());

        /* Aici ne pregatim pentru a impinge specimenele. Folosim BezierCurve pentru o linie curbata. */
        specimenReady = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePose), new Point(specimenReadyControl) ,new Point(specimenReadyPos)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), specimenReadyPos.getHeading())
                .build();

        specimen1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimenReadyPos), new Point(specimen1Pos)))
                .setConstantHeadingInterpolation(specimen1Pos.getHeading())
                .addPath(new BezierLine(new Point(specimen1Pos), new Point(specimen1HPlayer)))
                .setConstantHeadingInterpolation(specimen1Pos.getHeading())
                .addPath(new BezierLine(new Point(specimen1HPlayer), new Point(specimen1Pos)))
                .setConstantHeadingInterpolation(specimen1Pos.getHeading())
                .build();

        specimen2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimen1Pos), new Point(specimen2Pos)))
                .setConstantHeadingInterpolation(specimen2Pos.getHeading())
                .addPath(new BezierLine(new Point(specimen2Pos), new Point(specimen2HPlayer)))
                .setConstantHeadingInterpolation(specimen2Pos.getHeading())
                .addPath(new BezierLine(new Point(specimen2HPlayer), new Point(specimen2Pos)))
                .setConstantHeadingInterpolation(specimen2Pos.getHeading())
                .build();

        specimen3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimen2Pos), new Point(specimen3Pos)))
                .setConstantHeadingInterpolation(specimen3Pos.getHeading())
                .addPath(new BezierLine(new Point(specimen3Pos), new Point(specimen3HPlayer)))
                .setConstantHeadingInterpolation(specimen3Pos.getHeading())
                .build();

        subScore = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimen3Pos), new Point(scorePose)))
                .setLinearHeadingInterpolation(specimen3Pos.getHeading(), scorePose.getHeading())
                .build();

        hPlayer = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(specimen3Pos)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), specimen3Pos.getHeading())
                .build();

        park = new Path(new BezierCurve(new Point(scorePose), /* Control Point */ new Point(parkControlPose), new Point(parkPose)));
        park.setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading());
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                claw.closeOuttakeClaw();
                //TODO make specimen extend slider
                slider.MoveOuttakeSlider(RobotConstants.outtakeSliderExtendPosition, 1);
                axon.SetOuttakeAxonPosition(RobotConstants.outtakeBehindPos);
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:
                /* Urmatoarea miscare incepe doar dupa ce robotul este la 1inch dinstanta de cealalta */
                if(follower.getPose().getX() > (scorePose.getX() - 1) && follower.getPose().getY() > (scorePose.getY() - 1)) {
                    // TODO make specimen retract slider
                    slider.MoveOuttakeSlider(RobotConstants.outtakeSliderRetractPosition, 1);
                    claw.openOuttakeClaw();
                    follower.followPath(specimenReady,true);
                    setPathState(2);
                }
                break;
            case 2:
                /* Urmatoarea miscare incepe doar dupa ce robotul este la 1inch dinstanta de cealalta */
                if(follower.getPose().getX() > (specimenReadyPos.getX() - 1) && follower.getPose().getY() > (specimenReadyPos.getY() - 1)) {
                    follower.followPath(specimen1,true);
                    setPathState(3);
                }
                break;
            case 3:
                /* Urmatoarea miscare incepe doar dupa ce robotul este la 1inch dinstanta de cealalta */
                if(follower.getPose().getX() > (specimen1Pos.getX() - 1) && follower.getPose().getY() > (specimen1Pos.getY() - 1)) {
                    follower.followPath(specimen2,true);
                    setPathState(4);
                }
                break;
            case 4:
                /* Urmatoarea miscare incepe doar dupa ce robotul este la 1inch dinstanta de cealalta */
                if(follower.getPose().getX() > (specimen2Pos.getX() - 1) && follower.getPose().getY() > (specimen2Pos.getY() - 1)) {
                    follower.followPath(specimen3,true);
                    setPathState(5);
                }
                break;
            case 5:
                /* Urmatoarea miscare incepe doar dupa ce robotul este la 1inch dinstanta de cealalta */
                if(follower.getPose().getX() > (specimen3Pos.getX() - 1) && follower.getPose().getY() > (specimen3Pos.getY() - 1)) {
                    claw.closeOuttakeClaw();
                    //TODO make specimen extend slider
                    slider.MoveOuttakeSlider(RobotConstants.outtakeSliderExtendPosition, 1);
                    follower.followPath(subScore,true);
                    setPathState(6);
                }
                break;
            case 6:
                /* Urmatoarea miscare incepe doar dupa ce robotul este la 1inch dinstanta de cealalta */
                if(follower.getPose().getX() > (scorePose.getX() - 1) && follower.getPose().getY() > (scorePose.getY() - 1)) {
                    // TODO make specimen retract slider
                    slider.MoveOuttakeSlider(RobotConstants.outtakeSliderRetractPosition, 1);
                    claw.openOuttakeClaw();
                    follower.followPath(hPlayer, true);
                    setPathState(7);
                }
                break;
            case 7:
                /* Urmatoarea miscare incepe doar dupa ce robotul este la 1inch dinstanta de cealalta */
                if(follower.getPose().getX() > (specimen3Pos.getX() - 1) && follower.getPose().getY() > (specimen3Pos.getY() - 1)) {
                    claw.closeOuttakeClaw();
                    //TODO make specimen extend slider
                    slider.MoveOuttakeSlider(RobotConstants.outtakeSliderExtendPosition, 1);
                    follower.followPath(subScore,true);
                    setPathState(8);
                }
                break;
            case 8:
                /* Urmatoarea miscare incepe doar dupa ce robotul este la 1inch dinstanta de cealalta */
                if(follower.getPose().getX() > (scorePose.getX() - 1) && follower.getPose().getY() > (scorePose.getY() - 1)) {
                    // TODO make specimen retract slider
                    slider.MoveOuttakeSlider(RobotConstants.outtakeSliderRetractPosition, 1);
                    claw.openOuttakeClaw();
                    follower.followPath(hPlayer, true);
                    setPathState(9);
                }
                break;
            case 9:
                /* Urmatoarea miscare incepe doar dupa ce robotul este la 1inch dinstanta de cealalta */
                if(follower.getPose().getX() > (specimen3Pos.getX() - 1) && follower.getPose().getY() > (specimen3Pos.getY() - 1)) {
                    claw.closeOuttakeClaw();
                    //TODO make specimen extend slider
                    slider.MoveOuttakeSlider(RobotConstants.outtakeSliderExtendPosition, 1);
                    follower.followPath(subScore,true);
                    setPathState(10);
                }
                break;
            case 10:
                /* Urmatoarea miscare incepe doar dupa ce robotul este la 1inch dinstanta de cealalta */
                if(follower.getPose().getX() > (scorePose.getX() - 1) && follower.getPose().getY() > (scorePose.getY() - 1)) {
                    // TODO make specimen retract slider
                    slider.MoveOuttakeSlider(RobotConstants.outtakeSliderRetractPosition, 1);
                    claw.openOuttakeClaw();
                    follower.followPath(park,true);
                    setPathState(11);
                }
                break;
            case 11:
                /* Urmatoarea miscare incepe doar dupa ce robotul este la 1inch dinstanta de cealalta */
                if(follower.getPose().getX() > (parkPose.getX() - 1) && follower.getPose().getY() > (parkPose.getY() - 1)) {
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
        telemetry.update();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();

        opmodeTimer.resetTimer();

        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        buildPaths();

        claw = new ClawSubsystem(hardwareMap);
        slider = new SliderSubsystem(hardwareMap);
        axon = new AxonSubsystem(hardwareMap);

        claw.closeOuttakeClaw();
        claw.closeIntakeClaw();
        axon.SetOuttakeAxonPosition(RobotConstants.outtakeMidPos);
        axon.SetIntakeAxonPosition(RobotConstants.intakeUpPos);
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
