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
    private final Pose scorePosePreLoad = new Pose(36.2, 70, Math.toRadians(180));
    private final Pose scorePose = new Pose(38, 67, Math.toRadians(180)); // off 36 70
    private final Pose moveHangedSpecimenPose = new Pose(38, 70, Math.toRadians(180));

    // Specimen 1
    private final Pose specimen1Pos = new Pose(32, 33, Math.toRadians(325));
    private final Pose specimen1HPlayer = new Pose(25, 36, Math.toRadians(240));

    // Specimen 2
    private final Pose specimen2Pos = new Pose(31, 23, Math.toRadians(325));
    private final Pose specimen2HPlayer = new Pose(25, 36, Math.toRadians(240));

    // Specimen 3
    private final Pose specimen3Pos = new Pose(34, 16, Math.toRadians(312));
    private final Pose specimen3HPlayer = new Pose(25, 36, Math.toRadians(240));
    private final Pose specimenHPlayer1 = new Pose(16, 23, Math.toRadians(0));
    private final Pose specimenHPlayer2 = new Pose(4, 23, Math.toRadians(0));
    private final Pose specimenHPlayerFin = new Pose(5, 23, Math.toRadians(0));

    // Pozitia de parcare
    private final Pose parkPose = new Pose(4, 23, Math.toRadians(270));

    // Aici stocam traiectoriile robotului
    private PathChain scorePreload, park, specimen1, specimen2, specimen3, specimen4, specimen5, specimen6, specimenHPlayer, specimenHPlayerGet, subScore, moveHangedSpecimen, hPlayer;

    public void buildPaths() {

        /* Aici punem preload-ul cu care incepem. Folosim BezierLine pentru o linie dreapta */
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePosePreLoad))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .build();

        specimen1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePosePreLoad, specimen1Pos))
                .setLinearHeadingInterpolation(scorePosePreLoad.getHeading(), specimen1Pos.getHeading())
                .build();

        specimen2 = follower.pathBuilder()
                .addPath(new BezierLine(specimen1Pos, specimen1HPlayer))
                .setLinearHeadingInterpolation(specimen1Pos.getHeading(), specimen1HPlayer.getHeading())
                .build();

        specimen3 = follower.pathBuilder()
                .addPath(new BezierLine(specimen1HPlayer, specimen2Pos))
                .setLinearHeadingInterpolation(specimen1HPlayer.getHeading(), specimen2Pos.getHeading())
                .build();

        specimen4 = follower.pathBuilder()
                .addPath(new BezierLine(specimen2Pos, specimen2HPlayer))
                .setLinearHeadingInterpolation(specimen2Pos.getHeading(), specimen2HPlayer.getHeading())
                .build();

        specimen5 = follower.pathBuilder()
                .addPath(new BezierLine(specimen2HPlayer, specimen3Pos))
                .setLinearHeadingInterpolation(specimen2HPlayer.getHeading(), specimen3Pos.getHeading())
                .build();

        specimen6 = follower.pathBuilder()
                .addPath(new BezierLine(specimen3Pos, specimen3HPlayer))
                .setLinearHeadingInterpolation(specimen3Pos.getHeading(), specimen3HPlayer.getHeading())
                .build();

        specimenHPlayer = follower.pathBuilder()
                .addPath(new BezierLine(specimen3HPlayer, specimenHPlayer1))
                .setLinearHeadingInterpolation(specimen3HPlayer.getHeading(), specimenHPlayer1.getHeading())
                .build();

        specimenHPlayerGet = follower.pathBuilder()
                .addPath(new BezierLine(specimenHPlayer1, specimenHPlayer2))
                .setConstantHeadingInterpolation(specimenHPlayer1.getHeading())
                .build();

        moveHangedSpecimen = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, moveHangedSpecimenPose))
                .setConstantHeadingInterpolation(scorePose.getHeading())
                .build();

        hPlayer = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, specimenHPlayer1))
                .setLinearHeadingInterpolation(scorePose.getHeading(), specimenHPlayer1.getHeading())
                .build();

        park = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, parkPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading())
                .build();
    }

    public void buildDynamicPaths(){
        subScore = follower.pathBuilder()
                .addPath(new BezierLine(specimenHPlayerFin, scorePose))
                .setLinearHeadingInterpolation(specimenHPlayerFin.getHeading(), scorePose.getHeading())
                .build();
    }

    boolean timerReseted = false;

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload, 0.6, true);
                claw.CloseOuttake();
                axon.SetOuttakePosition(RobotConstants.outtakeBehindPos);
                setPathState(1);
                break;
            case 1:
                /* Urmatoarea miscare incepe doar dupa ce robotul este la 1inch dinstanta de cealalta */
                if (follower.getPose().getX() > (scorePosePreLoad.getX() - 1) && follower.getPose().getY() > (scorePosePreLoad.getY() - 1)) {
                    slider.MoveOuttake(RobotConstants.outtakeSliderReleasePosition, 0.9);
                    if(slider.GetUpOuttakePosition() < RobotConstants.outtakeSliderReleasePosition + 10) {
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
                /* Urmatoarea miscare incepe cand robotul a terminat ultima actiune */
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
                /* Urmatoarea miscare incepe doar dupa ce robotul ajunge la o anumita rotatie */
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
                /* Urmatoarea miscare incepe cand robotul a terminat ultima actiune */
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
                            follower.followPath(specimen4, 0.8, true);
                            setPathState(5);
                        }
                    }
                }
                break;
            case 5:
                /* Urmatoarea miscare incepe doar dupa ce robotul ajunge la o anumita rotatie */
                if (follower.getPose().getHeading() <= Math.toRadians(260)) {
                    claw.OpenIntake();
                    slider.MoveIntake(RobotConstants.intakeSliderExtendPosition - 800, 1);
                    axon.SetIntakePosition(RobotConstants.intakeMiddlePos);
                    timerReseted = false;
                    follower.followPath(specimen5, 0.9, true);
                    setPathState(6);
                }
                break;
            case 6:
                /* Urmatoarea miscare incepe cand robotul a terminat ultima actiune */
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
                /* Urmatoarea miscare incepe doar dupa ce robotul ajunge la o anumita rotatie */
                if (follower.getPose().getHeading() <= Math.toRadians(250)) {
                    claw.OpenIntake();
                    slider.MoveIntake(RobotConstants.intakeSliderRetractPosition, 1);
                    follower.followPath(specimenHPlayer, true);
                    setPathState(8);
                }
                break;
            case 8:
                axon.SetIntakePosition(RobotConstants.intakeUpPos);
                /* Urmatoarea miscare incepe doar dupa ce robotul este la 1inch dinstanta de cealalta */
                if (follower.getPose().getX() < (specimenHPlayer1.getX() + 1) && follower.getPose().getY() < (specimenHPlayer1.getY() + 1)) {
                    follower.followPath(specimenHPlayerGet, true);
                    timerReseted = false;
                    setPathState(9);
                }
                break;
            case 9:
                /* Urmatoarea miscare incepe in functie de timer */
                if(!timerReseted){
                    pathTimer.resetTimer();
                    timerReseted = true;
                }
                if(pathTimer.getElapsedTimeSeconds() > 0.45)
                    claw.CloseOuttake();
                if(pathTimer.getElapsedTimeSeconds() > 0.65) {
                    specimenHPlayerFin.setX(follower.getPose().getX());
                    specimenHPlayerFin.setY(follower.getPose().getY());
                    buildDynamicPaths();
                    slider.MoveOuttake(RobotConstants.outtakeSliderHPlayerSpecimenPosition, 0.65);
                    follower.followPath(subScore, true);
                    setPathState(10);
                }
                break;
            case 10:
                /* Urmatoarea miscare incepe doar dupa ce robotul este la 1inch dinstanta de cealalta */
                if (follower.getPose().getX() > (scorePose.getX() - 1) && follower.getPose().getY() > (scorePose.getY() - 1)) {
                    slider.MoveOuttake(RobotConstants.outtakeSliderReleasePosition, 0.9);
                    if (slider.GetUpOuttakePosition() < RobotConstants.outtakeSliderReleasePosition + 10) {
                        follower.followPath(moveHangedSpecimen, true);
                        setPathState(11);
                    }
                }
                break;
            case 11:
                /* Urmatoarea miscare incepe doar dupa ce robotul este la 1inch dinstanta de cealalta */
                if (follower.getPose().getX() > (moveHangedSpecimenPose.getX() - 1) && follower.getPose().getY() > (moveHangedSpecimenPose.getY() - 1)) {
                    claw.OpenOuttake();
                    slider.MoveOuttake(RobotConstants.outtakeSliderRetractPosition, 0.4);
                    follower.followPath(hPlayer);
                    setPathState(12);
                }
                break;
            case 12:
                /* Urmatoarea miscare incepe doar dupa ce robotul este la 1inch dinstanta de cealalta */
                if (follower.getPose().getX() < (specimenHPlayer1.getX() + 1) && follower.getPose().getY() < (specimenHPlayer1.getY() + 1)) {
                    follower.followPath(specimenHPlayerGet, true);
                    timerReseted = false;
                    setPathState(13); 
                }
                break;
            case 13:
                /* Urmatoarea miscare incepe in functie de timer*/
                if(!timerReseted){
                    pathTimer.resetTimer();
                    timerReseted = true;
                }
                if(pathTimer.getElapsedTimeSeconds() > 0.45)
                    claw.CloseOuttake();
                if(pathTimer.getElapsedTimeSeconds() > 0.65) {
                    specimenHPlayerFin.setX(follower.getPose().getX());
                    specimenHPlayerFin.setY(follower.getPose().getY());
                    buildDynamicPaths();
                    slider.MoveOuttake(RobotConstants.outtakeSliderHPlayerSpecimenPosition, 0.65);
                    follower.followPath(subScore, true);
                    setPathState(14);
                }
                break;
            case 14:
                /* Urmatoarea miscare incepe doar dupa ce robotul este la 1inch dinstanta de cealalta */
                if (follower.getPose().getX() > (scorePose.getX() - 1) && follower.getPose().getY() > (scorePose.getY() - 1)) {
                    slider.MoveOuttake(RobotConstants.outtakeSliderReleasePosition, 0.9);
                    if(slider.GetUpOuttakePosition() < RobotConstants.outtakeSliderReleasePosition + 10) {
                        follower.followPath(moveHangedSpecimen, true);
                        setPathState(15); 
                    }
                }
                break;
            case 15:
                /* Urmatoarea miscare incepe doar dupa ce robotul este la 1inch dinstanta de cealalta */
                if (follower.getPose().getX() > (moveHangedSpecimenPose.getX() - 1) && follower.getPose().getY() > (moveHangedSpecimenPose.getY() - 1)) {
                    claw.OpenOuttake();
                    slider.MoveOuttake(RobotConstants.outtakeSliderRetractPosition, 0.4);
                    follower.followPath(hPlayer);
                    setPathState(16); 
                }
                break;
            case 16:
                /* Urmatoarea miscare incepe doar dupa ce robotul este la 1inch dinstanta de cealalta */
                if (follower.getPose().getX() < (specimenHPlayer1.getX() + 1) && follower.getPose().getY() < (specimenHPlayer1.getY() + 1)) {
                    follower.followPath(specimenHPlayerGet, true);
                    timerReseted = false;
                    setPathState(17); 
                }
                break;
            case 17:
                /* Urmatoarea miscare incepe in functie de timer */
                if(!timerReseted){
                    pathTimer.resetTimer();
                    timerReseted = true;
                }
                if(pathTimer.getElapsedTimeSeconds() > 0.45)
                    claw.CloseOuttake();
                if(pathTimer.getElapsedTimeSeconds() > 0.65) {
                    specimenHPlayerFin.setX(follower.getPose().getX());
                    specimenHPlayerFin.setY(follower.getPose().getY());
                    scorePose.setX(scorePose.getX() + 0.5);
                    buildDynamicPaths();
                    slider.MoveOuttake(RobotConstants.outtakeSliderHPlayerSpecimenPosition, 0.65);
                    follower.followPath(subScore, true);
                    setPathState(18);
                }
                break;
            case 18:
                /* Urmatoarea miscare incepe doar dupa ce robotul este la 1inch dinstanta de cealalta */
                if (follower.getPose().getX() > (scorePose.getX() - 1) && follower.getPose().getY() > (scorePose.getY() - 1)) {
                    slider.MoveOuttake(RobotConstants.outtakeSliderReleasePosition, 0.9);
                    if(slider.GetUpOuttakePosition() < RobotConstants.outtakeSliderReleasePosition + 10) {
                        claw.OpenOuttake();
                        follower.followPath(park, true);
                        setPathState(19); 
                    }
                }
                break;
            case 19:
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
