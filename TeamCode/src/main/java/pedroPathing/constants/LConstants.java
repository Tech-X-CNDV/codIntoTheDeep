package pedroPathing.constants;

import com.pedropathing.localization.*;
import com.pedropathing.localization.constants.*;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public class LConstants {
    static {
        ThreeWheelConstants.forwardTicksToInches = .002;
        ThreeWheelConstants.strafeTicksToInches = .002;
        ThreeWheelConstants.turnTicksToInches = .0019;
        ThreeWheelConstants.leftY = 4.5;
        ThreeWheelConstants.rightY = -4.5;
        ThreeWheelConstants.strafeX = -3.5;
        ThreeWheelConstants.leftEncoder_HardwareMapName = "leftFrontMotor";
        ThreeWheelConstants.rightEncoder_HardwareMapName = "rightFrontMotor";
        ThreeWheelConstants.strafeEncoder_HardwareMapName = "leftRearMotor";
        ThreeWheelConstants.leftEncoderDirection = Encoder.FORWARD;
        ThreeWheelConstants.rightEncoderDirection = Encoder.FORWARD;
        ThreeWheelConstants.strafeEncoderDirection = Encoder.FORWARD;
    }
}




