// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.opencv.core.Mat;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class arm extends SubsystemBase {
  Encoder m_armEncoder = new Encoder(Constants.kEncoderChannelA, Constants.kEncoderChannelB, true,  EncodingType.k4X);
  Spark m_armMotor = new Spark(Constants.kMotorChannel);
  PIDController m_armPIDController = new PIDController(Constants.kP, Constants.kI, Constants.kD);
  DutyCycleEncoder m_armEncoderPos = new DutyCycleEncoder(Constants.kDutyCycleEncoderChannel);

  double m_startTimeSeconds = 0.00;
  double m_velocityRadiansPerSecond = 0.0;

  private enum State {
    CALIBRATING,
    OPERATING,
    OUT_OF_SCOPE
  }

  private State m_state = State.CALIBRATING;

  public arm() {
    m_armEncoder.setDistancePerPulse(Constants.kArmDistancePerPulseRadians);
    m_armEncoder.reset();

    m_armEncoderPos.setDistancePerRotation(Constants.kDutyCycleEncoderDistancePerRotRadians);
    m_armPIDController.setTolerance(Constants.kArmDistanceToleranceRadiansPerSecond);

    m_armMotor.setInverted(true);
  }

  public void setArmSpeed(double speed) {
    m_velocityRadiansPerSecond = speed;
  }
  
  public void setOperatingState() {
    m_state = State.OPERATING;
  }

  private void moveArmVel(double speed) {
    m_armMotor.set(m_armPIDController.calculate(m_armEncoder.getRate(), speed));
  }

  public void indexArm() {
    m_state = State.CALIBRATING;
    m_startTimeSeconds = (double)RobotController.getFPGATime()/1000000;
    moveArmVel(Constants.kCalibrationSpeedRadiansPerSecond);
  }

  private boolean inTolerance(double input, double setPoint, double percentTolerance) {
    if (input >= setPoint - setPoint * percentTolerance &&
      input <= setPoint + setPoint * percentTolerance) {
      return true;
    }
    return false;
  }

  @Override
  public void periodic() {
    SmartDashboard.putString("State:", m_state.toString());
    SmartDashboard.putNumber("Arm Speed:", m_armEncoder.getRate());
    SmartDashboard.putNumber("Arm Pos:", m_armEncoder.getDistance());
    SmartDashboard.putNumber("Time:", (double)RobotController.getFPGATime()/1000000 - m_startTimeSeconds);
    SmartDashboard.putNumber("Target Speed:", m_velocityRadiansPerSecond);
    SmartDashboard.putNumber("Speed difference:", m_velocityRadiansPerSecond - m_armEncoder.getRate());
    SmartDashboard.putNumber("Current Encoder Count: ", m_armEncoder.get());

    switch (m_state) {
      case CALIBRATING:
      {
        double armSpeedRadiansPerSecond = m_armEncoder.getRate();
        double currentTimeSeconds = (double)RobotController.getFPGATime()/1000000;
        if ((Math.abs(armSpeedRadiansPerSecond) <= Constants.kCalibrationToleranceRadiansPerSecond) && 
        (currentTimeSeconds - m_startTimeSeconds >= Constants.kCalibrationSecondsDelay)) {
          moveArmVel(0.0);
          m_armEncoder.reset();
          m_state = State.OPERATING;
        }
        break;
      }
    
      case OPERATING:
      {
        double armPosRadians = m_armEncoder.getDistance();
        moveArmVel(m_velocityRadiansPerSecond);
        if (armPosRadians >= Constants.kAngleCutoffRadians && m_velocityRadiansPerSecond < 0.0) {
          moveArmVel(0.0);
        }
        break;
      }
        
    }
  }
}
