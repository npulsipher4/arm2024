// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class arm extends SubsystemBase {
  PWM m_armMotor = new PWM(Constants.kArmPWMid);
  DutyCycleEncoder m_armEncoder = new DutyCycleEncoder(Constants.kEncoderChannel);
  PIDController m_armPIDController = new PIDController(Constants.kP, Constants.kI, Constants.kD);
  double m_relativePositionOffset = 0.0;
  double m_startVelocity = findCurrentVelocity();
  boolean m_prevCycleAcceleration = false;
  double prevDistance = 0;

  public arm() {
    m_armEncoder.setDistancePerRotation(Constants.kArmDistancePerRot);
    prevDistance = m_armEncoder.getAbsolutePosition();
  }


  private double findDecelThreshold(double speed) {
    if (speed > 0) {
      return Constants.kTargetDistance - Constants.kDecelThresholdRel;
    }
    return Constants.kTargetDistance + Constants.kDecelThresholdRel;
  }

  public double findCurrentVelocity() {
    double vel = (m_armEncoder.getAbsolutePosition() - prevDistance) / Constants.kDeltaT;
    prevDistance = m_armEncoder.getAbsolutePosition();
    return vel;

  }

  private double getRelativeEncoderPos() {
    return m_armEncoder.getAbsolutePosition() - m_relativePositionOffset;
  }

  private boolean shouldBeAccelerating(double targetVelocity) {
    if (targetVelocity + (targetVelocity * Constants.kAccelerationTolerance) > findCurrentVelocity() &&
      targetVelocity - (targetVelocity * Constants.kAccelerationTolerance) < findCurrentVelocity()) {
        return false;
    }
    return true;
  }

  private double calculateVelocity(double joystickSpeed) {
    double targetVelocity = joystickSpeed * Constants.kMaxVelocity;
    if ((findCurrentVelocity() > targetVelocity && targetVelocity >= 0.0) ||
      (findCurrentVelocity() < targetVelocity && targetVelocity <= 0.0) ||
      (!m_prevCycleAcceleration && shouldBeAccelerating(targetVelocity))) {
        m_relativePositionOffset = m_armEncoder.getAbsolutePosition();
        m_startVelocity = findCurrentVelocity();
    }
    double deltaV = targetVelocity - m_startVelocity;
    double distanceScalingFactor = (Constants.kFinishedAccelerationPeriod * (deltaV/Constants.kMaxVelocity));

    m_prevCycleAcceleration = shouldBeAccelerating(targetVelocity);

    return (
      3 * deltaV * Math.pow(
        getRelativeEncoderPos()/distanceScalingFactor,
        2) + 
      -2 * deltaV * Math.pow( 
        getRelativeEncoderPos()/distanceScalingFactor,
        3) + 
      m_startVelocity
    );
  }

  public void moveArm(double joystickSpeed) {
      calculateVelocity(joystickSpeed);
  }

  @Override
  public void periodic() {
    
  }
}
