// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ExternalEncoderConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class arm extends SubsystemBase {
  SparkFlex m_armMotor = new SparkFlex(Constants.kMotorChannel, MotorType.kBrushless);
  SparkFlexConfig m_armMotorConfig = new SparkFlexConfig();
  ExternalEncoderConfig m_armEncoderConfig = new ExternalEncoderConfig();

  PIDController m_armPIDController = new PIDController(Constants.kP, Constants.kI, Constants.kD);
  RelativeEncoder m_armEncoder = m_armMotor.getEncoder();

  double m_startTimeSeconds = 0.00;
  double m_velocityRadiansPerSecond = 0.0;

  private enum State {
    CALIBRATING,
    OPERATING,
    AT_RIGHT_BOUND
  }

  private State m_state = State.CALIBRATING;

  public arm() {
    // Motor / Encoder config
    m_armEncoderConfig.positionConversionFactor(Constants.kArmPositionConversionFactor); //TODO: change to conversion factor to get radians 
    m_armEncoderConfig.velocityConversionFactor(Constants.kArmVelocityConversionFactor);
    m_armMotorConfig.apply(m_armEncoderConfig);
    m_armMotor.configure(m_armMotorConfig, null, null);

    m_armPIDController.setTolerance(Constants.kArmDistanceToleranceRadiansPerSecond);

    m_armEncoder.setPosition(0.0);
  }

  public void setArmSpeed(double speed) {
    m_velocityRadiansPerSecond = speed;
  }
  
  public void setOperatingState() {
    m_state = State.OPERATING;
  }

  private void moveArmVel(double speed) {
    m_armMotor.set(m_armPIDController.calculate(m_armEncoder.getVelocity(), speed));
  }

  public void indexArm() {
    m_state = State.CALIBRATING;
    m_startTimeSeconds = (double)RobotController.getFPGATime()/1000000;
    moveArmVel(Constants.kCalibrationSpeedRadiansPerSecond);
  }

  /* private boolean inTolerance(double input, double setPoint, double percentTolerance) {
    if (input >= setPoint - setPoint * percentTolerance &&
      input <= setPoint + setPoint * percentTolerance) {
      return true;
    }
    return false;
  } */

  @Override
  public void periodic() {
    SmartDashboard.putString("State:", m_state.toString());
    SmartDashboard.putNumber("Arm Speed:", m_armEncoder.getVelocity());
    SmartDashboard.putNumber("Arm Pos:", m_armEncoder.getPosition());
    SmartDashboard.putNumber("Time:", (double)RobotController.getFPGATime()/1000000 - m_startTimeSeconds);
    SmartDashboard.putNumber("Target Speed:", m_velocityRadiansPerSecond);
    SmartDashboard.putNumber("Speed difference:", m_velocityRadiansPerSecond - m_armEncoder.getVelocity());

    switch (m_state) {
      case CALIBRATING:
      {
        double armSpeedRadiansPerSecond = m_armEncoder.getVelocity();
        double currentTimeSeconds = (double)RobotController.getFPGATime()/1000000;
        if ((Math.abs(armSpeedRadiansPerSecond) <= Constants.kCalibrationToleranceRadiansPerSecond) && 
        (currentTimeSeconds - m_startTimeSeconds >= Constants.kCalibrationSecondsDelay)) {
          moveArmVel(0.0);
          m_armEncoder.setPosition(0.0);
          m_state = State.OPERATING;
        }
        break;
      }
    
      case OPERATING:
      {
        double armPosRadians = m_armEncoder.getPosition();
        moveArmVel(m_velocityRadiansPerSecond);
        if (armPosRadians >= Constants.kAngleCutoffRadians && m_velocityRadiansPerSecond > 0.0) {
          moveArmVel(0.0);
          m_state = State.AT_RIGHT_BOUND;
        }
        break;
      }

      case AT_RIGHT_BOUND:
      if (m_velocityRadiansPerSecond < 0) {
        moveArmVel(m_velocityRadiansPerSecond);
      } else {
        moveArmVel(0.0);
      }
      if (m_armEncoder.getPosition() < Constants.kAngleCutoffRadians) {
        m_state = State.OPERATING;
      }
        
    }
  }
}
