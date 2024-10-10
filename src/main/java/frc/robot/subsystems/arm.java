// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class arm extends SubsystemBase {
  PWM m_armMotor = new PWM(Constants.kArmPWMid);
  DutyCycleEncoder m_armEncoder = new DutyCycleEncoder(Constants.kEncoderChannel);
  Spark m_armMotorSpark = new Spark(Constants.kEncoderChannel);
  PIDController m_armPIDController = new PIDController(Constants.kP, Constants.kI, Constants.kD);

  public arm() {
    m_armEncoder.setDistancePerRotation(Constants.kArmDistancePerRot);
  }

  public void moveArm(double joystickSpeed) {
      
  }

  @Override
  public void periodic() {
    
  }
}
