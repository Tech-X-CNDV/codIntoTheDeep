package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.List;
@TeleOp(name = "Basic: testBulkMode", group = "Linear OpMode")
public class testBulkMode extends OpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    private DcMotorEx  motor1;
    private DcMotorEx  motor2;
    private List<LynxModule> allHubs;

    @Override
    public void init() {
        motor1 = hardwareMap.get(DcMotorEx.class, "intakeSlider");
        motor2 = hardwareMap.get(DcMotorEx.class, "outtakeSliderUp");

        // Get all Lynx modules
        allHubs = hardwareMap.getAll(LynxModule.class);

        // Enable bulk caching for all modules
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        // Initialize data series for motor 1 amperage
        //dashboard.getTelemetry().addDataSeries("Motor 1 Amperage", "AMPS");
// Initialize data series for motor 2 amperage
        //dashboard.getTelemetry().addDataSeries("Motor 2 Amperage", "AMPS");
    }

    @Override


    public void loop() {
        motor2.setPower(gamepad2.right_stick_y);
        // Clear cache at the beginning of each loop if necessary
        // Read motor amperage
        double motor1Amperage = motor1.getCurrent(CurrentUnit.AMPS);
        double motor2Amperage = motor2.getCurrent(CurrentUnit.AMPS);

        dashboardTelemetry.addData("Motor 1 Amperage", motor1Amperage);
        dashboardTelemetry.addData("Motor 2 Amperage", motor2Amperage);

        telemetry.addData("Motor 1 Amperage", motor1Amperage);
        telemetry.addData("Motor 2 Amperage", motor2Amperage);
        dashboard.getTelemetry().update();
        telemetry.update();
        dashboardTelemetry.update();
    }
}