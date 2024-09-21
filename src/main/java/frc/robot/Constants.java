package frc.robot;
public class Constants {
    public static final int kEncoderChannelA = 2;
    public static final int kEncoderChannelB = 3;
    public static final int kDutyCycleEncoderChannel = 0;
    public static final double kDutyCycleEncoderDistancePerRot = 360.0;
    public static final int kMotorChannel = 0;
    public static final double kP = 1.6;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kArmDistancePerPulse = 2.0 * Math.PI / 4096.0;
    public static final double kArmDistanceTolerance = Math.PI / 64;
    public static final double kMaxVelocity = Math.PI / 4.0;
    public static final double kJoystickTolerance = 0.05;
    public static final double kCalibrationSpeed = Math.PI / 18.0;
    public static final double kCalibrationTolerance = Math.PI / 365.0;
    public static final double kCalibrationDelay = 1.0;

}
