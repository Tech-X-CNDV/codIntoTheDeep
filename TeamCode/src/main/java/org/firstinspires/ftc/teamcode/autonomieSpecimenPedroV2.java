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


@Autonomous(name = "AutonomieSpecimenV2")
public class autonomieSpecimenPedroV2 extends OpMode {

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
    private final Pose scorePosePreLoad = new Pose(35, 63, Math.toRadians(180));
    private final Pose scorePose1 = new Pose(38, 67, Math.toRadians(180)); // off 36 70
    private final Pose scorePose2 = new Pose(41, 73, Math.toRadians(180)); // off 42 74
    private final Pose scorePose3 = new Pose(43, 77, Math.toRadians(180)); // off 44 77

    // Specimen 1
    private final Pose specimen1Pos = new Pose(34, 35, Math.toRadians(322));
    private final Pose specimen1HPlayer = new Pose(25, 36, Math.toRadians(240));

    // Specimen 2
    private final Pose specimen2Pos = new Pose(33, 25, Math.toRadians(325));
    private final Pose specimen2HPlayer = new Pose(25, 36, Math.toRadians(240));

    // Specimen 3
    private final Pose specimen3Pos = new Pose(34.5, 14, Math.toRadians(318));
    private final Pose specimen3HPlayer = new Pose(25, 36, Math.toRadians(240));
    private final Pose specimenHPlayer1 = new Pose(16, 23, Math.toRadians(0));
    private final Pose specimenHPlayer2 = new Pose(4, 23, Math.toRadians(0));
    private final Pose specimenHPlayerFin = new Pose(5, 23, Math.toRadians(0));

    // Pozitia de parcare
    private final Pose parkPose = new Pose(59, 95, Math.toRadians(270));

    // Aici stocam traiectoriile robotului
    private PathChain scorePreload, park, specimen1, specimen2, specimen3, specimen4, specimen5, specimen6, specimenHPlayer, specimenHPlayerGet, subScore1, subScore2, subScore3, hPlayer1, hPlayer2, hPlayerGet1, hPlayerGet2;

    public void buildPaths() {

        /* Aici punem preload-ul cu care incepem. Folosim BezierLine pentru o linie dreapta */
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(scorePosePreLoad)))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .build();

        specimen1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePosePreLoad), new Point(specimen1Pos)))
                .setLinearHeadingInterpolation(scorePosePreLoad.getHeading(), specimen1Pos.getHeading())
                .build();

        specimen2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimen1Pos), new Point(specimen1HPlayer)))
                .setLinearHeadingInterpolation(specimen1Pos.getHeading(), specimen1HPlayer.getHeading())
                .build();

        specimen3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimen1HPlayer), new Point(specimen2Pos)))
                .setLinearHeadingInterpolation(specimen1HPlayer.getHeading(), specimen2Pos.getHeading())
                .build();

        specimen4 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimen2Pos), new Point(specimen2HPlayer)))
                .setLinearHeadingInterpolation(specimen2Pos.getHeading(), specimen2HPlayer.getHeading())
                .build();

        specimen5 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimen2HPlayer), new Point(specimen3Pos)))
                .setLinearHeadingInterpolation(specimen2HPlayer.getHeading(), specimen3Pos.getHeading())
                .build();

        specimen6 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimen3Pos), new Point(specimen3HPlayer)))
                .setLinearHeadingInterpolation(specimen3Pos.getHeading(), specimen3HPlayer.getHeading())
                .build();

        specimenHPlayer = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimen3HPlayer), new Point(specimenHPlayer1)))
                .setLinearHeadingInterpolation(specimen3HPlayer.getHeading(), specimenHPlayer1.getHeading())
                .build();

        specimenHPlayerGet = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimenHPlayer1), new Point(specimenHPlayer2)))
                .setConstantHeadingInterpolation(specimenHPlayer1.getHeading())
                .build();

        subScore1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimenHPlayer2), new Point(scorePose1)))
                .setLinearHeadingInterpolation(specimenHPlayer2.getHeading(), scorePose1.getHeading())
                .build();

        subScore2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimenHPlayer2), new Point(scorePose2)))
                .setLinearHeadingInterpolation(specimenHPlayer2.getHeading(), scorePose2.getHeading())
                .build();

        subScore3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimenHPlayer2), new Point(scorePose3)))
                .setLinearHeadingInterpolation(specimenHPlayer2.getHeading(), scorePose3.getHeading())
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
                .addPath(new BezierCurve(new Point(scorePose3), new Point(specimenHPlayer2)))
                .setLinearHeadingInterpolation(scorePose3.getHeading(), parkPose.getHeading())
                .build();
    }

    public void buildDynamicPaths(int pathNumber){
        switch (pathNumber){
            case 1:
                subScore1 = follower.pathBuilder()
                        .addPath(new BezierLine(new Point(specimenHPlayerFin), new Point(scorePose1)))
                        .setLinearHeadingInterpolation(specimenHPlayer2.getHeading(), scorePose1.getHeading())
                        .build();
                break;
            case 2:
                subScore2 = follower.pathBuilder()
                        .addPath(new BezierLine(new Point(specimenHPlayerFin), new Point(scorePose2)))
                        .setLinearHeadingInterpolation(specimenHPlayer2.getHeading(), scorePose2.getHeading())
                        .build();
                break;
            case 3:
                subScore3 = follower.pathBuilder()
                        .addPath(new BezierLine(new Point(specimenHPlayerFin), new Point(scorePose3)))
                        .setLinearHeadingInterpolation(specimenHPlayer2.getHeading(), scorePose3.getHeading())
                        .build();
                break;
        }
    }

    boolean timerReseted = false;

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload, 0.45, true);
                claw.CloseOuttake();
                axon.SetOuttakePosition(RobotConstants.outtakeBehindPos);
                setPathState(1);
                break;
            case 1:
                /* Urmatoarea miscare incepe doar dupa ce robotul este la 1inch dinstanta de cealalta */
                if (follower.getPose().getX() > (scorePosePreLoad.getX() - 1) && follower.getPose().getY() > (scorePosePreLoad.getY() - 1)) {
                    slider.MoveOuttake(RobotConstants.outtakeSliderReleasePosition, 0.9);
                    if(slider.GetUpOuttakePosition() < RobotConstants.outtakeSliderReleasePosition + 200) {
                        claw.OpenOuttake();
                        follower.followPath(specimen1, 0.9, true);
                        setPathState(2);
                    }
                }else{
                    slider.MoveOuttake(RobotConstants.outtakeSliderSpecimenPosition, 1);
                }
                break;
            case 2:
                slider.MoveOuttake(RobotConstants.outtakeSliderRetractPosition, 0.3);
                /* Urmatoarea miscare incepe doar dupa ce robotul este la 1inch dinstanta de cealalta */
                if (!follower.isBusy()) {
                    if(slider.GetIntakePosition() > RobotConstants.intakeSliderExtendPosition - 520){
                        claw.Rotated(true);
                        if(pathTimer.getElapsedTimeSeconds() > 0.1)
                            axon.SetIntakePosition(RobotConstants.intakeDownPos + 0.01);
                        if(pathTimer.getElapsedTimeSeconds() > 0.4)
                            claw.CloseIntake();
                        if(pathTimer.getElapsedTimeSeconds() > 0.6) {
                            slider.MoveIntake(RobotConstants.intakeSliderExtendPosition, 1);
                            axon.SetIntakePosition(RobotConstants.intakeHPlayerMiddlePos);
                            follower.followPath(specimen2, 0.9, true);
                            setPathState(3);
                        }
                    }else{
                        slider.MoveIntake(RobotConstants.intakeSliderExtendPosition - 500, 1);
                        axon.SetIntakePosition(RobotConstants.intakeMiddlePos);
                        claw.OpenIntake();
                        pathTimer.resetTimer();
                    }
                }
                break;
            case 3:
                /* Urmatoarea miscare incepe doar dupa ce robotul este la 1inch dinstanta de cealalta */
                if (follower.getPose().getHeading() <= Math.toRadians(260)) {
                    claw.OpenIntake();
                    slider.MoveIntake(RobotConstants.intakeSliderExtendPosition - 500, 1);
                    axon.SetIntakePosition(RobotConstants.intakeMiddlePos);
                    timerReseted = false;
                    follower.followPath(specimen3, 0.9, true);
                    setPathState(4);
                }
                break;
            case 4:
                /* Urmatoarea miscare incepe doar dupa ce robotul este la 1inch dinstanta de cealalta */
                if (!follower.isBusy()) {
                    if(slider.GetIntakePosition() < RobotConstants.intakeSliderExtendPosition - 480){
                        if(!timerReseted){
                            pathTimer.resetTimer();
                            timerReseted = true;
                        }
                        if(pathTimer.getElapsedTimeSeconds() > 0.1)
                            axon.SetIntakePosition(RobotConstants.intakeDownPos + 0.01);
                        if(pathTimer.getElapsedTimeSeconds() > 0.4)
                            claw.CloseIntake();
                        if(pathTimer.getElapsedTimeSeconds() > 0.6) {
                            slider.MoveIntake(RobotConstants.intakeSliderExtendPosition, 1);
                            axon.SetIntakePosition(RobotConstants.intakeHPlayerMiddlePos);
                            follower.followPath(specimen4, 0.9, true);
                            setPathState(5);
                        }
                    }
                }
                break;
            case 5:
                /* Urmatoarea miscare incepe doar dupa ce robotul este la 1inch dinstanta de cealalta */
                if (follower.getPose().getHeading() <= Math.toRadians(260)) {
                    claw.OpenIntake();
                    slider.MoveIntake(RobotConstants.intakeSliderExtendPosition - 600, 1);
                    axon.SetIntakePosition(RobotConstants.intakeMiddlePos);
                    timerReseted = false;
                    follower.followPath(specimen5, 0.9, true);
                    setPathState(6);
                }
                break;
            case 6:
                /* Urmatoarea miscare incepe doar dupa ce robotul este la 1inch dinstanta de cealalta */
                if (!follower.isBusy()) {
                    slider.MoveIntake(RobotConstants.intakeSliderExtendPosition - 480, 1);
                    if(slider.GetIntakePosition() < RobotConstants.intakeSliderExtendPosition - 450){
                        if(!timerReseted){
                            pathTimer.resetTimer();
                            timerReseted = true;
                        }
                        if(pathTimer.getElapsedTimeSeconds() > 0.1)
                            axon.SetIntakePosition(RobotConstants.intakeDownPos + 0.01);
                        if(pathTimer.getElapsedTimeSeconds() > 0.4)
                            claw.CloseIntake();
                        if(pathTimer.getElapsedTimeSeconds() > 0.6) {
                            axon.SetIntakePosition(RobotConstants.intakeHPlayerMiddlePos);
                            follower.followPath(specimen6, 0.9, true);
                            setPathState(7);
                        }
                    }
                }
                break;
            case 7:
                if(pathTimer.getElapsedTimeSeconds() > 1.0 && pathTimer.getElapsedTimeSeconds() < 1.2)
                    slider.MoveIntake(RobotConstants.intakeSliderExtendPosition, 1);
                /* Urmatoarea miscare incepe doar dupa ce robotul este la 1inch dinstanta de cealalta */
                if (follower.getPose().getHeading() <= Math.toRadians(250)) {
                    claw.OpenIntake();
                    slider.MoveIntake(RobotConstants.intakeSliderRetractPosition, 1);
                    follower.followPath(specimenHPlayer, true);
                    setPathState(8);
                }
                break;
            case 8:
                axon.SetIntakePosition(RobotConstants.intakeUpPos);
                if (follower.getPose().getX() < (specimenHPlayer1.getX() + 1) && follower.getPose().getY() < (specimenHPlayer1.getY() + 1)) {
                    follower.followPath(specimenHPlayerGet, true);
                    timerReseted = false;
                    setPathState(9);
                }
                break;
            case 9:
                /* Urmatoarea miscare incepe doar dupa ce robotul este la 1inch dinstanta de cealalta */
                if(!timerReseted){
                    pathTimer.resetTimer();
                    timerReseted = true;
                }
                if (pathTimer.getElapsedTimeSeconds() > 0.1) {
                    if(pathTimer.getElapsedTimeSeconds() > 0.3)
                        claw.CloseOuttake();
                    if(pathTimer.getElapsedTimeSeconds() > 0.5) {
                        specimenHPlayerFin.setX(follower.getPose().getX());
                        specimenHPlayerFin.setY(follower.getPose().getY());
                        buildDynamicPaths(1);
                        slider.MoveOuttake(RobotConstants.outtakeSliderHPlayerSpecimenPosition, 0.65);
                        follower.followPath(subScore1, true);
                        setPathState(10);
                    }
                }
                break;
            case 10:
                /* Urmatoarea miscare incepe doar dupa ce robotul este la 1inch dinstanta de cealalta */
                if (follower.getPose().getX() > (scorePose1.getX() - 1) && follower.getPose().getY() > (scorePose1.getY() - 1)) {
                    slider.MoveOuttake(RobotConstants.outtakeSliderReleasePosition, 0.9);
                    if (slider.GetUpOuttakePosition() < RobotConstants.outtakeSliderReleasePosition + 200) {
                        claw.OpenOuttake();
                        follower.followPath(hPlayer1, true);
                        slider.MoveOuttake(RobotConstants.outtakeSliderRetractPosition, 0.4);
                        setPathState(11);
                    }
                }
                break;
            case 11:
                if (follower.getPose().getX() < (specimenHPlayer1.getX() + 1) && follower.getPose().getY() < (specimenHPlayer1.getY() + 1)) {
                    follower.followPath(specimenHPlayerGet, true);
                    timerReseted = false;
                    setPathState(12);
                }
                break;
            case 12:
                /* Urmatoarea miscare incepe doar dupa ce robotul este la 1inch dinstanta de cealalta */
                if(!timerReseted){
                    pathTimer.resetTimer();
                    timerReseted = true;
                }
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
                    slider.MoveOuttake(RobotConstants.outtakeSliderReleasePosition, 0.9);
                    if(slider.GetUpOuttakePosition() < RobotConstants.outtakeSliderReleasePosition + 200) {
                        claw.OpenOuttake();
                        follower.followPath(hPlayer2, true);
                        slider.MoveOuttake(RobotConstants.outtakeSliderRetractPosition, 0.4);
                        setPathState(14);
                    }
                }
                break;
            case 14:
                if (follower.getPose().getX() < (specimenHPlayer1.getX() + 1) && follower.getPose().getY() < (specimenHPlayer1.getY() + 1)) {
                    follower.followPath(specimenHPlayerGet, true);
                    timerReseted = false;
                    setPathState(15);
                }
                break;
            case 15:
                /* Urmatoarea miscare incepe doar dupa ce robotul este la 1inch dinstanta de cealalta */
                if(!timerReseted){
                    pathTimer.resetTimer();
                    timerReseted = true;
                }
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
                    slider.MoveOuttake(RobotConstants.outtakeSliderReleasePosition, 0.9);
                    if(slider.GetUpOuttakePosition() < RobotConstants.outtakeSliderReleasePosition + 200) {
                        claw.OpenOuttake();
                        follower.followPath(park, true);
                        setPathState(17);
                    }
                }
                break;
            case 17:
                slider.MoveOuttake(RobotConstants.outtakeSliderRetractPosition, 0.4);
                claw.OpenOuttake();
                axon.SetOuttakePosition(RobotConstants.outtakeAutoMidPos);
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

    public void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}
