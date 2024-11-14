package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp

public class autoTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        // ... (Initializing encoders and variables)

        // Initializing the angle of the robot
        double robotAngle = 0;

        // Definirea constantelor
        double TICKS_PER_REV = 2000; // For example, for an encoder with 2000 ticks per revolution
        double WHEEL_CIRCUMFERENCE = 48 * Math.PI; // For example, for a wheel with a diameter of 48mm

        // Reading the values ​​from the encoders
        int encoder1Ticks = encoder1.getCurrentPosition();
        int encoder2Ticks = encoder2.getCurrentPosition();
        int encoder3Ticks = encoder3.getCurrentPosition();

        // Calculation of distances traveled by each wheel
        double distance1 = (encoder1Ticks * WHEEL_CIRCUMFERENCE) / TICKS_PER_REV;
        double distance2 = (encoder2Ticks * WHEEL_CIRCUMFERENCE) / TICKS_PER_REV;
        double distance3 = (encoder3Ticks * WHEEL_CIRCUMFERENCE) / TICKS_PER_REV;

        double deltaX1 = distance1 * Math.cos(robotAngle); // Wheel 1: front-left (0 degrees)
        double deltaY1 = distance1 * Math.sin(robotAngle);
        double deltaX2 = distance2 * Math.cos(robotAngle + Math.toRadians(90)); // Wheel 2: front-right (90 degrees)
        double deltaY2 = distance2 * Math.sin(robotAngle + Math.toRadians(90));
        double deltaX3 = distance3 * Math.cos(robotAngle + Math.toRadians(180)); // Wheel 3: back-middle (180 degrees)
        double deltaY3 = distance3 * Math.sin(robotAngle + Math.toRadians(180));

        // Combination of displacements
        double deltaX = (deltaX1 + deltaX2 + deltaX3) / 3; // Average of displacements
        double deltaY = (deltaY1 + deltaY2 + deltaY3) / 3;

        double x = 0;
        double y = 0;

        // Position Update
        x += deltaX;
        y += deltaY;

        // Angles calculation
        double robotAngle1 = Math.atan2(deltaY1, deltaX1);
        double robotAngle2 = Math.atan2(deltaY2, deltaX2);
        double robotAngle3 = Math.atan2(deltaY3, deltaX3);

        // Combining Angles
        robotAngle = (robotAngle1 + robotAngle2 + robotAngle3) / 3;

        // Target position and heading
        double targetX = 100;
        double targetY = 50;
        double targetAngle = Math.toRadians(90); // Target heading in radians

        // Calculate movement vectors
        double distance = Math.hypot(targetX - x, targetY - y);
        double angleToTarget = Math.atan2(targetY - y, targetX - x);
        double headingError = targetAngle - robotAngle;

        double kP_linear = 0.5; // Proportional gain for linear movement
        double linearSpeed = kP_linear * distance;

        double vx = linearSpeed * Math.cos(angleToTarget);
        double vy = linearSpeed * Math.sin(angleToTarget);

        double kP_angular = 1.0; // Proportional gain for angular movement
        double vω = kP_angular * headingError;

        double scaledVx = vx / maxVx;
        double scaledVy = vy / maxVy;
        double scaledVω = vω / maxVω;

        scaledVx = Math.max(-1.0, Math.min(1.0, scaledVx));
        scaledVy = Math.max(-1.0, Math.min(1.0, scaledVy));
        scaledVω = Math.max(-1.0, Math.min(1.0, scaledVω));

        // Calculate wheel velocities (adjust signs based on your mecanum configuration)
        double v1 = +scaledVx + scaledVy + scaledVω;
        double v2 = +scaledVx - scaledVy + scaledVω;
        double v3 = -scaledVx - scaledVy + scaledVω;
        double v4 = -scaledVx + scaledVy + scaledVω;

        motor1.setPower(v1);
        motor2.setPower(v2);
        motor3.setPower(v3);
        motor4.setPower(v4);

    }
}