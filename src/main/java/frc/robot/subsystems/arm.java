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
  PWM armMotor = new PWM(Constants.kArmPWMid);
  DutyCycleEncoder armEncoder = new DutyCycleEncoder(Constants.kEncoderChannel);
  PIDController armPIDController = new PIDController(Constants.kP, Constants.kI, Constants.kD);
  double prevDistance = 0;

  public arm() {
    armEncoder.setDistancePerRotation(Constants.kArmDistancePerRot);
    prevDistance = armEncoder.getAbsolutePosition();
  }


  private double findDecelThreshold(double speed) {
    if (speed > 0) {
      return Constants.kTargetDistance - Constants.kDecelThresholdRel;
    }
    return Constants.kTargetDistance + Constants.kDecelThresholdRel;
  }

  public double findCurrentVelocity() {
    double vel = (armEncoder.getAbsolutePosition() - prevDistance) / Constants.kDeltaT;
    prevDistance = armEncoder.getAbsolutePosition();
    return vel;

  }

  private double calculateVelocity(double joystickSpeed, double distance) {
    double distanceOffset = distance - armEncoder.getAbsolutePosition();
    double deltaV = (joystickSpeed * Constants.kMaxVelocity) - findCurrentVelocity();
    double distanceScalingFactor = (Constants.kDeltaVelocityPeriod * (deltaV/Constants.kMaxVelocity));
    return (
      3 * deltaV * Math.pow(
        distanceOffset/distanceScalingFactor,
        2) + 
      -2 * deltaV * Math.pow( 
        distanceOffset/distanceScalingFactor,
        3) + 
      findCurrentVelocity()
    );
  }

  public void moveArm(double joystickSpeed) {
      calculateVelocity(joystickSpeed , armEncoder.getAbsolutePosition());
  }

  @Override
  public void periodic() {
    
  }
}
