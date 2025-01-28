package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.config.subsystem.ClawSubsystem;
import org.firstinspires.ftc.teamcode.config.subsystem.SliderSubsystem;
import org.firstinspires.ftc.teamcode.config.subsystem.AxonSubsystem;
import org.firstinspires.ftc.teamcode.config.RobotConstants;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

import java.util.List;
import java.util.Locale;

@TeleOp(name = "Basic: OPModeV2", group = "Linear OpMode")
public class OPModeV2 extends OpMode {
    public ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime timer = new ElapsedTime();

    private List<LynxModule> allHubs;

    private Follower follower;
    private final Pose startPose = new Pose(0,0,0);

    // Servouri ghiare
    private ClawSubsystem claw;

    // Slidere
    private SliderSubsystem slider;

    // Axoane
    private AxonSubsystem axon;

    boolean resetOuttakeTriggered = false;

    @Override
    public void init() {
        // Get all Lynx modules
        allHubs = hardwareMap.getAll(LynxModule.class);

        // Enable bulk caching for all modules
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // Initializare Pedro
        Constants.setConstants(FConstants.class, LConstants.class);
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
        claw.InitIntake();
        claw.InitOuttake();
        claw.InitPivot();
        slider.InitIntake();
        slider.InitOuttake();
        axon.InitIntake();
        axon.InitOuttake(RobotConstants.outtakeMidPos);
        resetOuttakeTriggered = true;
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
        if(intakeToOuttakeTriggered)
            IntakeToOuttake();
        if(resetOuttakeTriggered){
            slider.ResetOuttake();
            if(slider.GetOuttakeAmperage() > RobotConstants.outtakeSliderStopAmperage) {
                resetOuttakeTriggered = false;
                slider.ResetOuttakeEncoder();
            }
        }
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
            if(!hPlayer) {
                claw.OpenOuttake();
                sleep(100);
                outtakeAxonVal = RobotConstants.outtakeBehindPos;
            }
            hPlayer = !hPlayer;
        } else if(!gamepad2.left_bumper)
            lBmpPressed = false;
    }

    boolean outtakeAutoSlider = false;
    boolean intakeAutoSlider = false;

    public void IntakeToOuttake() {
        if(timer.seconds() < 0.2)
            return;
        if(!hPlayer)
            axon.SetIntakePosition(RobotConstants.intakeUpPos);
        else
            axon.SetIntakePosition(RobotConstants.intakeHPlayerMiddlePos);
        claw.InitialRot();
        intakeMidAxonOn = false;
        intakeAxonOn = false;
        if(!hPlayer) {
            if(timer.seconds() < 0.8)
                return;
            claw.OpenIntake();
        }
        if(timer.seconds() > 1 && !hPlayer)
            return;
        intakeAutoSlider = true;
        slider.MoveIntake(!hPlayer ? RobotConstants.intakeSliderRetractPosition : RobotConstants.intakeSliderRetractPosition + 300, 1);
        if(hPlayer && slider.GetIntakePosition() > RobotConstants.intakeSliderRetractPosition + 400)
            return;
        if(!hPlayer)
            axon.SetIntakePosition(RobotConstants.intakeMiddlePos);
        else
            axon.SetIntakePosition(RobotConstants.intakeHPlayerMiddlePos);
        intakeMidAxonOn = true;
        intakeToOuttakeTriggered = false;
    }

    boolean changedROTINT = false;

    private void ToggleGhiaraRotireIntake() {
        if (gamepad1.b && !changedROTINT) {
            if(claw.GetPivotIntakePosition() == 0.0)
                claw.Rotated();
            else
                claw.InitialRot();
            changedROTINT = true;
        } else if(!gamepad1.b)
            changedROTINT = false;
    }

    boolean changedINT = false;
    boolean intakeToOuttakeTriggered = false;

    private void ToggleGhiaraIntake() {
        if (gamepad1.a && !changedINT) {
            changedINT = true;
            if(claw.GetGrabIntakePosition() == 0.0) {
                claw.OpenIntake();
                intakeMidAxonOn = true;
                axon.SetIntakePosition(RobotConstants.intakeMiddlePos);
                intakeAutoSlider = true;
                slider.MoveIntake(RobotConstants.intakeSliderRetractPosition, 1);
            }
            else
                claw.CloseIntake();
            if(intakeAxonOn) {
                intakeToOuttakeTriggered = true;
                follower.setMaxPower(1.0);
                speedLimit = false;
                timer.reset();
            }
        } else if (!gamepad1.a)
            changedINT = false;
    }

    boolean changedOUT = false;

    private void ToggleGhiaraOuttake() {
        if ((gamepad2.a || gamepad2.right_bumper) && !changedOUT) {
            changedOUT = true;
            if(claw.GetGrabOuttakePosition() == 0.0)
                claw.OpenOuttake();
            else
                claw.CloseOuttake();
            if(slider.GetUpOuttakePosition() > RobotConstants.outtakeSliderExtendPosition - 100) {
                sleep(300);
                outtakeAxonVal = RobotConstants.outtakeMidPos;
                outtakeAutoSlider = true;
                slider.MoveOuttake(RobotConstants.outtakeSliderRetractPosition, 1);
            }
        } else if (!gamepad2.a)
            changedOUT = false;
    }

    private void OuttakeSliderMotion(){
        if (gamepad2.left_stick_y != 0.0) {
            outtakeAutoSlider = false;
            if(hPlayer)
                slider.MoveOuttake(gamepad2.left_stick_y < 0 ? RobotConstants.outtakeSliderHPlayerSpecimenPosition : RobotConstants.outtakeSliderRetractPosition, Math.abs(gamepad2.left_stick_y));
            else
                slider.MoveOuttake(gamepad2.left_stick_y < 0 ? RobotConstants.outtakeSliderExtendPosition : RobotConstants.outtakeSliderRetractPosition, Math.abs(gamepad2.left_stick_y));
        } else if (!outtakeAutoSlider)
            slider.StopOuttake();
    }

    private void IntakeSliderMotion(){
        if(gamepad1.right_trigger != 0 && gamepad1.left_trigger == 0) {
            intakeAutoSlider = false;
            slider.MoveIntake(RobotConstants.intakeSliderExtendPosition, gamepad1.right_trigger);
        } else if(gamepad1.left_trigger != 0) {
            intakeAutoSlider = false;
            slider.MoveIntake(RobotConstants.intakeSliderRetractPosition, gamepad1.left_trigger);
        } else if(!intakeAutoSlider)
            slider.StopIntake();
    }

    double outtakeAxonVal = RobotConstants.outtakeMidPos;
    private void OuttakeAxonMotion(){
        if (gamepad2.right_stick_y < 0 && outtakeAxonVal < 1)
            outtakeAxonVal += 0.005;
        else if (gamepad2.right_stick_y > 0 && outtakeAxonVal > 0)
            outtakeAxonVal -= 0.005;
        else if (gamepad2.dpad_up) {
            outtakeAxonVal = RobotConstants.outtakeUpPos;
            if (claw.GetGrabOuttakePosition() == RobotConstants.closePos) {
                outtakeAutoSlider = true;
                slider.MoveOuttake(RobotConstants.outtakeSliderExtendPosition, 1);
            }
        } else if (gamepad2.dpad_down)
            outtakeAxonVal = RobotConstants.outtakeMidPos;
        else if (gamepad2.dpad_left) {
            outtakeAxonVal = RobotConstants.outtakeBehindPos;
            claw.OpenOuttake();
        }
        axon.SetOuttakePosition(outtakeAxonVal);
    }

    boolean intakeAxonOn = false;
    boolean intakeAxonMove = false;
    boolean intakeMidAxonMove = false;
    boolean intakeMidAxonOn = false;

    private void IntakeAxonMotion(){
        if(gamepad1.x && intakeMidAxonMove && !intakeAxonOn) {
            axon.SetIntakePosition(intakeMidAxonOn ? RobotConstants.intakeUpPos : RobotConstants.intakeMiddlePos);
            if(!intakeMidAxonOn)
                claw.OpenIntake();
            intakeMidAxonOn = !intakeMidAxonOn;
            intakeMidAxonMove = false;
        } else if(!gamepad1.x)
            intakeMidAxonMove = true;
        if(gamepad1.y && intakeAxonMove && intakeMidAxonOn) {
            axon.SetIntakePosition(intakeAxonOn ? RobotConstants.intakeMiddlePos : RobotConstants.intakeDownPos);
            intakeAxonOn = !intakeAxonOn;
            intakeAxonMove = false;
        } else if(!gamepad1.y)
            intakeAxonMove = true;
    }

    boolean speedLimit = false;
    boolean isG1LBumberPressed = false;
    private void Roti() {
        if(gamepad1.left_bumper && !isG1LBumberPressed) {
            isG1LBumberPressed = true;
            follower.setMaxPower(speedLimit ? 1.0 : 0.2);
            speedLimit = !speedLimit;
        } else if(!gamepad1.left_bumper)
            isG1LBumberPressed = false;
        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
        follower.update();
    }

    private void Telemetry() {
        telemetry.addData("Status", "Run Time: " + String.format(Locale.US, "%d:%.2f", (int)(runtime.seconds() / 60), runtime.seconds() % 60));
        // SpeedLimit telemetry
        if(speedLimit)
            telemetry.addData("SpeedLimit", "On");
        // HPlayer telemetry
        if(hPlayer)
            telemetry.addData("HPlayerMode", "On");
        // Pedro telemetry
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));
        // Ghiara intake telemetry
        telemetry.addData("Intake Position", claw.GetGrabIntakePosition());
        telemetry.addData("Intake Angle", claw.GetGrabIntakePosition() * 180);
        // Ghiara outtake telemetry
        telemetry.addData("Outtake Position", claw.GetGrabOuttakePosition());
        telemetry.addData("Outtake Angle", claw.GetGrabOuttakePosition() * 180);
        // Rotire ghiara intake telemetry
        telemetry.addData("Intake Rotation Position", claw.GetPivotIntakePosition());
        telemetry.addData("Intake Rotation Angle", claw.GetPivotIntakePosition() * 180);
        // Intake axon telemetry
        telemetry.addData("AxonIntake Position", axon.GetIntakePosition());
        telemetry.addData("AxonIntake Rotation Angle", axon.GetIntakePosition() * 180);
        // Outtake axon telemetry
        telemetry.addData("AxonOuttake Position", axon.GetOuttakePosition());
        telemetry.addData("AxonOuttake Rotation Angle", axon.GetOuttakePosition() * 180);
        // Outtake slider telemetry
        telemetry.addData("OuttakeSliderPowerUp", slider.GetUpOuttakePower());
        telemetry.addData("OuttakeSliderPowerDown", slider.GetDownOuttakePower());
        telemetry.addData("OuttakeSliderPositionUp", slider.GetUpOuttakePosition());
        telemetry.addData("OuttakeSliderPositionDown", slider.GetDownOuttakePosition());
        telemetry.addData("OuttakeSliderAmperage", slider.GetOuttakeAmperage());
        // Intake slider telemetry
        telemetry.addData("IntakeSliderPower", slider.GetIntakePower());
        telemetry.addData("IntakeSliderPos", slider.GetIntakePosition());
        telemetry.addData("IntakeSliderAmperage", slider.GetIntakeAmperage());
        // Send the telemetry to the driver station
        telemetry.update();
    }
}