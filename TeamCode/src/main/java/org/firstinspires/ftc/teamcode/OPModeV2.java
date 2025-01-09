package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Vector;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.config.subsystem.ClawSubsystem;
import org.firstinspires.ftc.teamcode.config.subsystem.SliderSubsystem;
import org.firstinspires.ftc.teamcode.config.subsystem.AxonSubsystem;
import org.firstinspires.ftc.teamcode.config.RobotConstants;

import java.util.Locale;

@TeleOp(name = "Basic: OPModeV2", group = "Linear OpMode")
public class OPModeV2 extends OpMode {
    public ElapsedTime runtime = new ElapsedTime();

    private Follower follower;
    private final Pose startPose = new Pose(0,0,0);

    // Servouri ghiare
    private ClawSubsystem claw;

    // Slidere
    private SliderSubsystem slider;

    // Axoane
    private AxonSubsystem axon;

    @Override
    public void init() {
        // Initializare Pedro
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        // Initializare servo ghiare
        claw = new ClawSubsystem(hardwareMap);

        // Initializare slidere
        slider = new SliderSubsystem(hardwareMap);

        // Initializare axoane
        axon = new AxonSubsystem(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        // Reset the runtime when the play button is pressed
        follower.startTeleopDrive();
        runtime.reset();
    }

    public void loop() {
        ToggleGhiaraRotireIntake();
        ToggleGhiaraIntake();
        ToggleGhiaraOuttake();
        OuttakeAxonMotion();
        IntakeAxonMotion();
        OuttakeSliderMotion();
        IntakeSliderMotion();
        Roti();
        HPlayerToggler();
        Telemetry();
    }

    public void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    boolean hPlayer = false;
    boolean lBmpPressed = false;

    public void HPlayerToggler() {
        if(gamepad2.left_bumper && !lBmpPressed) {
            lBmpPressed = true;
            hPlayer = !hPlayer;
        } else if(!gamepad2.left_bumper)
            lBmpPressed = false;
    }

    boolean outtakeAutoSlider = false;
    boolean intakeAutoSlider = false;

    public void IntakeToOuttake() {
        sleep(200);
        axon.SetIntakeAxonPosition(RobotConstants.intakeUpPos);
        claw.initialRotClaw();
        intakeMidAxonOn = false;
        intakeAxonOn = false;
        sleep(200);
        claw.openIntakeClaw();
        sleep(400);
        intakeAutoSlider = true;
        slider.MoveIntakeSlider(RobotConstants.intakeSliderRetractPosition, 1);
        axon.SetIntakeAxonPosition(RobotConstants.intakeMiddlePos);
        intakeMidAxonOn = true;
    }

    boolean changedROTINT = false;

    private void ToggleGhiaraRotireIntake() {
        if (gamepad1.b && !changedROTINT) {
            if(claw.getPivotIntakePosition() == 0.0)
                claw.rotatedClaw();
            else
                claw.initialRotClaw();
            changedROTINT = true;
        } else if(!gamepad1.b)
            changedROTINT = false;
    }

    boolean changedINT = false;

    private void ToggleGhiaraIntake() {
        if (gamepad1.a && !changedINT) {
            changedINT = true;
            if(claw.getGrabIntakePosition() == 0.0)
                claw.openIntakeClaw();
            else
                claw.closeIntakeClaw();
            if(intakeAxonOn && !hPlayer)
                IntakeToOuttake();
        } else if (!gamepad1.a)
            changedINT = false;
    }

    boolean changedOUT = false;

    private void ToggleGhiaraOuttake() {
        if ((gamepad2.a || gamepad2.right_bumper) && !changedOUT) {
            changedOUT = true;
            if(claw.getGrabOuttakePosition() == 0.0)
                claw.openOuttakeClaw();
            else
                claw.closeOuttakeClaw();
            if(slider.getSliderUpOuttakePosition() > RobotConstants.outtakeSliderExtendPosition - 100) {
                sleep(300);
                outtakeAxonVal = RobotConstants.outtakeMidPos;
                outtakeAutoSlider = true;
                slider.MoveOuttakeSlider(RobotConstants.outtakeSliderRetractPosition, 1);
            }
        } else if (!gamepad2.a)
            changedOUT = false;
    }

    private void OuttakeSliderMotion(){
        if (gamepad2.left_stick_y != 0.0) {
            outtakeAutoSlider = false;
            slider.MoveOuttakeSlider(gamepad2.left_stick_y < 0 ? RobotConstants.outtakeSliderExtendPosition : RobotConstants.outtakeSliderRetractPosition, Math.abs(gamepad2.left_stick_y));
        } else if (!outtakeAutoSlider)
            slider.StopOuttakeSlider();
    }

    private void IntakeSliderMotion(){
        if(gamepad1.right_trigger != 0 && gamepad1.left_trigger == 0) {
            intakeAutoSlider = false;
            slider.MoveIntakeSlider(RobotConstants.intakeSliderExtendPosition, gamepad1.right_trigger);
        } else if(gamepad1.left_trigger != 0) {
            intakeAutoSlider = false;
            slider.MoveIntakeSlider(RobotConstants.intakeSliderRetractPosition, gamepad1.left_trigger);
        } else if(!intakeAutoSlider)
            slider.StopIntakeSlider();
    }

    double outtakeAxonVal = 0.4;
    private void OuttakeAxonMotion(){
        if (gamepad2.right_stick_y < 0 && outtakeAxonVal < 1)
            outtakeAxonVal += 0.005;
        else if (gamepad2.right_stick_y > 0 && outtakeAxonVal > 0)
            outtakeAxonVal -= 0.005;
        else if (gamepad2.dpad_up) {
            outtakeAxonVal = RobotConstants.outtakeUpPos;
            if (claw.getGrabOuttakePosition() == RobotConstants.closePos) {
                outtakeAutoSlider = true;
                slider.MoveOuttakeSlider(RobotConstants.outtakeSliderExtendPosition, 1);
            }
        } else if (gamepad2.dpad_down)
            outtakeAxonVal = RobotConstants.outtakeMidPos;
        else if (gamepad2.dpad_left)
            outtakeAxonVal = RobotConstants.outtakeBehindPos;
        axon.SetOuttakeAxonPosition(outtakeAxonVal);
    }

    boolean intakeAxonOn = false;
    boolean intakeAxonMove = false;
    boolean intakeMidAxonMove = false;
    boolean intakeMidAxonOn = false;

    private void IntakeAxonMotion(){
        if(gamepad1.x && intakeMidAxonMove && !intakeAxonOn) {
            axon.SetIntakeAxonPosition(intakeMidAxonOn ? RobotConstants.intakeUpPos : RobotConstants.intakeMiddlePos);
            intakeMidAxonOn = !intakeMidAxonOn;
            intakeMidAxonMove = false;
        } else if(!gamepad1.x)
            intakeMidAxonMove = true;
        if(gamepad1.y && intakeAxonMove && intakeMidAxonOn) {
            axon.SetIntakeAxonPosition(intakeAxonOn ? RobotConstants.intakeMiddlePos : RobotConstants.intakeDownPos);
            intakeAxonOn = !intakeAxonOn;
            intakeAxonMove = false;
        } else if(!gamepad1.y)
            intakeAxonMove = true;
    }

    double speedLimit = 1.0;
    boolean isG1LBumberPressed = false;
    private void Roti() {
        if(gamepad1.left_bumper && !isG1LBumberPressed) {
            speedLimit = speedLimit == 1.0 ? 0.5 : 1.0;
            isG1LBumberPressed = true;
        } else if(!gamepad1.left_bumper)
            isG1LBumberPressed = false;
        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y / speedLimit, -gamepad1.left_stick_x / speedLimit, -gamepad1.right_stick_x / speedLimit, true);
        follower.update();
    }

    private void Telemetry() {
        telemetry.addData("Status", "Run Time: " + String.format(Locale.US, "%d:%.2f", (int)(runtime.seconds() / 60), runtime.seconds() % 60));
        // SpeedLimit telemetry
        if(speedLimit != 1.0)
            telemetry.addData("SpeedLimit", "On");
        // HPlayer telemetry
        if(hPlayer)
            telemetry.addData("HPlayerMode", "On");
        // Pedro telemetry
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));
        // Ghiara intake telemetry
        telemetry.addData("Intake Position", claw.getGrabIntakePosition());
        telemetry.addData("Intake Angle", claw.getGrabIntakePosition() * 180);
        // Ghiara outtake telemetry
        telemetry.addData("Outtake Position", claw.getGrabOuttakePosition());
        telemetry.addData("Outtake Angle", claw.getGrabOuttakePosition() * 180);
        // Rotire ghiara intake telemetry
        telemetry.addData("Intake Rotation Position", claw.getPivotIntakePosition());
        telemetry.addData("Intake Rotation Angle", claw.getPivotIntakePosition() * 180);
        // Intake axon telemetry
        telemetry.addData("AxonIntake Position", axon.getAxonIntakePosition());
        telemetry.addData("AxonIntake Rotation Angle", axon.getAxonIntakePosition() * 180);
        // Outtake axon telemetry
        telemetry.addData("AxonOuttake Position", axon.getAxonOuttakePosition());
        telemetry.addData("AxonOuttake Rotation Angle", axon.getAxonOuttakePosition() * 180);
        // Outtake slider telemetry
        telemetry.addData("OuttakeSliderPowerUp", slider.getSliderUpOuttakePower());
        telemetry.addData("OuttakeSliderPowerDown", slider.getSliderDownOuttakePower());
        telemetry.addData("OuttakeSliderPositionUp", slider.getSliderUpOuttakePosition());
        telemetry.addData("OuttakeSliderPositionDown", slider.getSliderDownOuttakePosition());
        // Intake slider telemetry
        telemetry.addData("IntakeSliderPower", slider.getSliderIntakePower());
        telemetry.addData("IntakeSliderPos", slider.getSliderIntakePosition());
        // Send the telemetry to the driver station
        telemetry.update();
    }
}