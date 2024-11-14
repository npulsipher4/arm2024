package frc.robot;
public class Constants {
    public static final int kEncoderChannelA = 2;
    public static final int kEncoderChannelB = 3;
    public static final int kDutyCycleEncoderChannel = 0;
    public static final double kDutyCycleEncoderDistancePerRotRadians = Math.PI / 2.0;
    public static final int kMotorChannel = 0;
    public static final double kP = 1.6;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kArmDistancePerPulseRadians = 2.0 * Math.PI / 4096.0;
    public static final double kArmDistanceToleranceRadiansPerSecond = Math.PI / 64;
    public static final double kMaxVelocityRadiansPerSecond = Math.PI / 4.0;
    public static final double kJoystickTolerance = 0.05;
    public static final double kCalibrationSpeedRadiansPerSecond = Math.PI / 18.0;
    public static final double kCalibrationToleranceRadiansPerSecond = Math.PI / 365.0;
    public static final double kCalibrationSecondsDelay = 1.0;
    public static final double kAngle1Radians = Math.PI / 4.0;

}
