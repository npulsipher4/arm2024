package frc.robot;
public class Constants {
    public static final int kArmPWMid = 1;
    public static final int kEncoderChannel = 0;
    public static final double kP = 0.5;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kArmDistancePerRot = 1.0; //TODO: get in rad
    public static final double kTargetDistance = Math.PI / 3.0;
    public static final double kDecelThresholdRel = Math.PI / 18;
    public static final double kMaxVelocity = Math.PI / 4.0;
    public static final double kFinishedAccelerationPeriod = 1.0;
    public static final double kDeltaT = 0.02;
    public static final double kAccelerationTolerance = 0.1;
    public static final double kMaxAcceleration = Math.PI / 4.0;
}
