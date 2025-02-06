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


@Autonomous(name = "AutonomieSpecimen")
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
    private final Pose scorePosePreLoad = new Pose(33, 63, Math.toRadians(180));
    private final Pose scorePose1 = new Pose(34, 68, Math.toRadians(180)); // off 36 70
    private final Pose scorePose2 = new Pose(36, 74, Math.toRadians(180)); // off 42 74
    private final Pose scorePose3 = new Pose(37, 77, Math.toRadians(180)); // off 44 77

    // Pregatire pentru specimene
    private final Pose specimenReadyPos = new Pose(62, 36, Math.toRadians(0));
    private final Pose specimenReadyControl = new Pose(20, 33, Point.CARTESIAN);

    // Specimen 1
    private final Pose specimen1Pos = new Pose(62, 24, Math.toRadians(0));
    private final Pose specimen1HPlayer = new Pose(16, 24, Math.toRadians(0));

    // Specimen 2
    private final Pose specimen2Pos = new Pose(62, 16, Math.toRadians(0));
    private final Pose specimen2HPlayer = new Pose(16, 16, Math.toRadians(0));

    // Specimen 3
    private final Pose specimen3Pos = new Pose(62, 11, Math.toRadians(0));
    private final Pose specimen3HPlayer = new Pose(16, 11, Math.toRadians(0));
    private final Pose specimenHPlayer1 = new Pose(16, 23, Math.toRadians(0));
    private final Pose specimenHPlayer2 = new Pose(4, 23, Math.toRadians(0));
    private final Pose specimenHPlayerFin = new Pose(4, 23, Math.toRadians(0));

    // Pozitia de parcare
    private final Pose parkPose = new Pose(59, 95, Math.toRadians(180));
    private final Pose parkControlPose1 = new Pose(11, 103, Point.CARTESIAN);
    private final Pose  parkControlPose2 = new Pose(70, 129, Point.CARTESIAN);

    // Aici stocam traiectoriile robotului
    private PathChain scorePreload, park, specimenReady, specimen1, specimen2, specimen3, specimen4, specimen5, specimenHPlayer, specimenHPlayerGet, subScore1, subScore2, subScore3, hPlayer1, hPlayer2;

    public void buildPaths() {
        /* Aici punem preload-ul cu care incepem. Folosim BezierLine pentru o linie dreapta */
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(scorePosePreLoad)))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .build();

        /* Aici ne pregatim pentru a impinge specimenele. Folosim BezierCurve pentru o linie curbata. */
        specimenReady = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePosePreLoad), new Point(specimenReadyControl), new Point(specimenReadyPos)))
                .setLinearHeadingInterpolation(scorePosePreLoad.getHeading(), specimenReadyPos.getHeading())
                .setPathEndTimeoutConstraint(1.0)
                .build();

        specimen1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimenReadyPos), new Point(specimen1Pos)))
                .setConstantHeadingInterpolation(specimen1Pos.getHeading())
                .addPath(new BezierLine(new Point(specimen1Pos), new Point(specimen1HPlayer)))
                .setConstantHeadingInterpolation(specimen1Pos.getHeading())
                .setPathEndTimeoutConstraint(1.0)
                .build();

        specimen2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimen1HPlayer), new Point(specimen1Pos)))
                .setConstantHeadingInterpolation(specimen1Pos.getHeading())
                .addPath(new BezierLine(new Point(specimen1Pos), new Point(specimen2Pos)))
                .setConstantHeadingInterpolation(specimen2Pos.getHeading())
                .setPathEndTimeoutConstraint(1.0)
                .build();

        specimen3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimen2Pos), new Point(specimen2HPlayer)))
                .setConstantHeadingInterpolation(specimen2Pos.getHeading())
                .setPathEndTimeoutConstraint(1.0)
                .build();

        specimen4 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimen2HPlayer), new Point(specimen2Pos)))
                .setConstantHeadingInterpolation(specimen2Pos.getHeading())
                .addPath(new BezierLine(new Point(specimen2Pos), new Point(specimen3Pos)))
                .setConstantHeadingInterpolation(specimen3Pos.getHeading())
                .setPathEndTimeoutConstraint(1.0)
                .build();

        specimen5 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimen3Pos), new Point(specimen3HPlayer)))
                .setConstantHeadingInterpolation(specimen3Pos.getHeading())
                .setPathEndTimeoutConstraint(1.0)
                .build();

        specimenHPlayer = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimen2HPlayer), new Point(specimenHPlayer1)))
                .setConstantHeadingInterpolation(specimen2HPlayer.getHeading())
                .setPathEndTimeoutConstraint(0.5)
                .build();

        specimenHPlayerGet = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimenHPlayer1), new Point(specimenHPlayer2)))
                .setConstantHeadingInterpolation(specimenHPlayer1.getHeading())
                .build();

        hPlayer1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose1), new Point(specimenHPlayer1)))
                .setLinearHeadingInterpolation(scorePose1.getHeading(), specimenHPlayer1.getHeading())
                .build();

        hPlayer2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose2), new Point(specimenHPlayer1)))
                .setLinearHeadingInterpolation(scorePose2.getHeading(), specimenHPlayer1.getHeading())
                .build();

        park = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePose3), new Point(specimenHPlayer1)))
                .setConstantHeadingInterpolation(scorePose3.getHeading())
                .build();
    }

    public void buildDynamicPaths(int pathNumber){
        switch (pathNumber){
            case 1:
                subScore1 = follower.pathBuilder()
                        .addPath(new BezierLine(new Point(specimenHPlayerFin), new Point(scorePose1)))
                        .setLinearHeadingInterpolation(specimenHPlayerFin.getHeading(), scorePose1.getHeading())
                        .build();
                break;
            case 2:
                subScore2 = follower.pathBuilder()
                        .addPath(new BezierLine(new Point(specimenHPlayerFin), new Point(scorePose2)))
                        .setLinearHeadingInterpolation(specimenHPlayerFin.getHeading(), scorePose2.getHeading())
                        .build();
                break;
            case 3:
                subScore3 = follower.pathBuilder()
                        .addPath(new BezierLine(new Point(specimenHPlayerFin), new Point(scorePose3)))
                        .setLinearHeadingInterpolation(specimenHPlayerFin.getHeading(), scorePose3.getHeading())
                        .build();
                break;
        }
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload, true);
                claw.CloseOuttake();
                slider.MoveOuttake(RobotConstants.outtakeSliderSpecimenPosition, 0.7);
                axon.SetOuttakePosition(RobotConstants.outtakeBehindPos);
                if(slider.GetUpOuttakePosition() > RobotConstants.outtakeSliderSpecimenPosition - 2)
                    setPathState(1);
                break;
            case 1:
                /* Urmatoarea miscare incepe doar dupa ce robotul este la 1inch dinstanta de cealalta */
                if (follower.getPose().getX() > (scorePosePreLoad.getX() - 1) && follower.getPose().getY() > (scorePosePreLoad.getY() - 1)) {
                    slider.MoveOuttake(RobotConstants.outtakeSliderReleasePosition, 0.7);
                    if(slider.GetUpOuttakePosition() < RobotConstants.outtakeSliderReleasePosition + 200) {
                        claw.OpenOuttake();
                        follower.followPath(specimenReady, true);
                        setPathState(2);
                    }
                }
                break;
            case 2:
                slider.MoveOuttake(RobotConstants.outtakeSliderRetractPosition, 0.3);
                /* Urmatoarea miscare incepe doar dupa ce robotul este la 1inch dinstanta de cealalta */
                if (follower.getPose().getX() > (specimenReadyPos.getX() - 2) && follower.getPose().getY() < (specimenReadyPos.getY() + 2)) {
                    follower.followPath(specimen1, true);
                    setPathState(3);
                }
                break;
            case 3:
                /* Urmatoarea miscare incepe doar dupa ce robotul este la 1inch dinstanta de cealalta */
                if (follower.getPose().getX() < (specimen1HPlayer.getX() + 2) && follower.getPose().getY() < (specimen1HPlayer.getY() + 2)) {
                    follower.followPath(specimen2, true);
                    setPathState(4);
                }
                break;
            case 4:
                /* Urmatoarea miscare incepe doar dupa ce robotul este la 1inch dinstanta de cealalta */
                if (follower.getPose().getX() > (specimen2Pos.getX() - 2) && follower.getPose().getY() < (specimen2Pos.getY() + 2)) {
                    follower.followPath(specimen3, true);
                    setPathState(7);
                }
                break;
            case 5:
                /* Urmatoarea miscare incepe doar dupa ce robotul este la 1inch dinstanta de cealalta */
                if (follower.getPose().getX() < (specimen2HPlayer.getX() + 2) && follower.getPose().getY() < (specimen2HPlayer.getY() + 2)) {
                    follower.followPath(specimen4, true);
                    setPathState(6);
                }
                break;
            case 6:
                /* Urmatoarea miscare incepe doar dupa ce robotul este la 1inch dinstanta de cealalta */
                if (follower.getPose().getX() > (specimen3Pos.getX() - 2) && follower.getPose().getY() < (specimen3Pos.getY() + 2)) {
                    follower.followPath(specimen5, true);
                    setPathState(7);
                }
                break;
            case 7:
                /* Urmatoarea miscare incepe doar dupa ce robotul este la 1inch dinstanta de cealalta */
                if (follower.getPose().getX() < (specimen2HPlayer.getX() + 2) && follower.getPose().getY() < (specimen2HPlayer.getY() + 2)) {
                    follower.followPath(specimenHPlayer, true);
                    setPathState(8);
                }
                break;
            case 8:
                if (follower.getPose().getX() > (specimenHPlayer1.getX() - 2) && follower.getPose().getY() > (specimenHPlayer1.getY() - 2)) {
                    follower.followPath(specimenHPlayerGet, true);
                    setPathState(9);
                }
                break;
            case 9:
                /* Urmatoarea miscare incepe doar dupa ce robotul este la 1inch dinstanta de cealalta */
                if (pathTimer.getElapsedTimeSeconds() > 0.1) {
                    if(pathTimer.getElapsedTimeSeconds() > 0.3)
                        claw.CloseOuttake();
                    if(pathTimer.getElapsedTimeSeconds() > 0.5) {
                        specimenHPlayerFin.setX(follower.getPose().getX());
                        specimenHPlayerFin.setY(follower.getPose().getY());
                        buildDynamicPaths(1);
                        slider.MoveOuttake(RobotConstants.outtakeSliderHPlayerSpecimenPosition, 0.55);
                        follower.followPath(subScore1, true);
                        setPathState(10);
                    }
                }
                break;
            case 10:
                /* Urmatoarea miscare incepe doar dupa ce robotul este la 1inch dinstanta de cealalta */
                if (follower.getPose().getX() > (scorePose1.getX() - 1) && follower.getPose().getY() > (scorePose1.getY() - 1)) {
                    slider.MoveOuttake(RobotConstants.outtakeSliderReleasePosition, 0.7);
                    if (slider.GetUpOuttakePosition() < RobotConstants.outtakeSliderReleasePosition + 200) {
                        claw.OpenOuttake();
                        follower.followPath(hPlayer1, true);
                        slider.MoveOuttake(RobotConstants.outtakeSliderRetractPosition, 0.3);
                        setPathState(11);
                    }
                }
                break;
            case 11:
                if (follower.getPose().getX() < (specimenHPlayer1.getX() + 2) && follower.getPose().getY() < (specimenHPlayer1.getY() + 2)) {
                    follower.followPath(specimenHPlayerGet, true);
                    setPathState(12);
                }
                break;
            case 12:
                /* Urmatoarea miscare incepe doar dupa ce robotul este la 1inch dinstanta de cealalta */
                if (pathTimer.getElapsedTimeSeconds() > 0.1) {
                    if(pathTimer.getElapsedTimeSeconds() > 0.3)
                        claw.CloseOuttake();
                    if(pathTimer.getElapsedTimeSeconds() > 0.5) {
                        specimenHPlayerFin.setX(follower.getPose().getX());
                        specimenHPlayerFin.setY(follower.getPose().getY());
                        buildDynamicPaths(2);
                        slider.MoveOuttake(RobotConstants.outtakeSliderHPlayerSpecimenPosition, 0.55);
                        follower.followPath(subScore2, true);
                        setPathState(13);
                    }
                }
                break;
            case 13:
                /* Urmatoarea miscare incepe doar dupa ce robotul este la 1inch dinstanta de cealalta */
                if (follower.getPose().getX() > (scorePose2.getX() - 1) && follower.getPose().getY() > (scorePose2.getY() - 1)) {
                    slider.MoveOuttake(RobotConstants.outtakeSliderReleasePosition, 0.7);
                    if(slider.GetUpOuttakePosition() < RobotConstants.outtakeSliderReleasePosition + 200) {
                        claw.OpenOuttake();
                        follower.followPath(hPlayer2, true);
                        slider.MoveOuttake(RobotConstants.outtakeSliderRetractPosition, 0.3);
                        setPathState(14);
                    }
                }
                break;
            case 14:
                if (follower.getPose().getX() < (specimenHPlayer1.getX() + 2) && follower.getPose().getY() < (specimenHPlayer1.getY() + 2)) {
                    follower.followPath(specimenHPlayerGet, true);
                    setPathState(15);
                }
                break;
            case 15:
                /* Urmatoarea miscare incepe doar dupa ce robotul este la 1inch dinstanta de cealalta */
                if (pathTimer.getElapsedTimeSeconds() > 0.1) {
                    if(pathTimer.getElapsedTimeSeconds() > 0.3)
                        claw.CloseOuttake();
                    if(pathTimer.getElapsedTimeSeconds() > 0.5) {
                        specimenHPlayerFin.setX(follower.getPose().getX());
                        specimenHPlayerFin.setY(follower.getPose().getY());
                        buildDynamicPaths(3);
                        slider.MoveOuttake(RobotConstants.outtakeSliderHPlayerSpecimenPosition, 0.55);
                        follower.followPath(subScore3, true);
                        setPathState(16);
                    }
                }
                break;
            case 16:
                /* Urmatoarea miscare incepe doar dupa ce robotul este la 1inch dinstanta de cealalta */
                if (follower.getPose().getX() > (scorePose3.getX() - 1) && follower.getPose().getY() > (scorePose3.getY() - 1)) {
                    slider.MoveOuttake(RobotConstants.outtakeSliderReleasePosition, 0.7);
                    if(slider.GetUpOuttakePosition() < RobotConstants.outtakeSliderReleasePosition + 200) {
                        claw.OpenOuttake();
                        slider.MoveIntake(RobotConstants.intakeSliderRetractPosition, 0.3);
                        follower.followPath(park, true);
                        setPathState(17);
                    }
                }
                break;
            case 17:
                claw.OpenOuttake();
                axon.SetOuttakePosition(RobotConstants.outtakeAutoMidPos);
                slider.MoveOuttake(RobotConstants.outtakeSliderRetractPosition, 0.3);
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
        telemetry.addData("TimeElapsed", pathTimer.getElapsedTimeSeconds());
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
        slider.InitIntake(true);
        slider.InitOuttake(true);
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
