package pedroPathing.constants;

import com.pedropathing.localization.*;
import com.pedropathing.localization.constants.*;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public class LConstants {
    static {
        ThreeWheelIMUConstants.forwardTicksToInches = .002;
        ThreeWheelIMUConstants.strafeTicksToInches = .0019;
        ThreeWheelIMUConstants.turnTicksToInches = .0019;
        ThreeWheelIMUConstants.leftY = 4.5;
        ThreeWheelIMUConstants.rightY = -4.5;
        ThreeWheelIMUConstants.strafeX = -3.5;
        ThreeWheelIMUConstants.leftEncoder_HardwareMapName = "leftFrontMotor";
        ThreeWheelIMUConstants.rightEncoder_HardwareMapName = "rightFrontMotor";
        ThreeWheelIMUConstants.strafeEncoder_HardwareMapName = "leftRearMotor";
        ThreeWheelIMUConstants.IMU_HardwareMapName = "imu";
        ThreeWheelIMUConstants.IMU_Orientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP);
        ThreeWheelIMUConstants.leftEncoderDirection = Encoder.FORWARD;
        ThreeWheelIMUConstants.rightEncoderDirection = Encoder.FORWARD;
        ThreeWheelIMUConstants.strafeEncoderDirection = Encoder.FORWARD;
    }
}




