// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class arm extends SubsystemBase {
  Encoder m_armEncoder = new Encoder(Constants.kEncoderChannelA, Constants.kEncoderChannelB, false,  EncodingType.k4X);
  Spark m_armMotor = new Spark(Constants.kMotorChannel);
  PIDController m_armPIDController = new PIDController(Constants.kP, Constants.kI, Constants.kD);
  
  double m_startTime = 0.00;
  double m_speed = 0.0;

  private enum State {
    CALIBRATING,
    OPERATING
  }

  private State m_state = State.CALIBRATING;

  public arm() {
    m_armEncoder.setDistancePerPulse(Constants.kArmDistancePerPulse);
    m_armEncoder.reset();
    m_armPIDController.setTolerance(Constants.kArmDistanceTolerance);
  }

  public void setArmSpeed(double speed) {
    m_speed = speed;
  }
  
  public void setOperatingState() {
    m_state = State.OPERATING;
  }

  private void moveArm(double speed) {
    m_armMotor.set(m_armPIDController.calculate(m_armEncoder.getRate(), speed));
  }

  public void indexArm() {
    m_state = State.CALIBRATING;
    m_startTime = (double)RobotController.getFPGATime()/1000000;
    moveArm(Constants.kCalibrationSpeed);
  }

  @Override
  public void periodic() {
    SmartDashboard.putString("State:", m_state.toString());
    SmartDashboard.putNumber("Arm Speed:", m_armEncoder.getRate());
    SmartDashboard.putNumber("Arm Pos:", m_armEncoder.getDistance());
    SmartDashboard.putNumber("Time:", (double)RobotController.getFPGATime()/1000000 - m_startTime);
    SmartDashboard.putNumber("Target Speed:", m_speed);
    SmartDashboard.putNumber("Speed difference:", m_speed - m_armEncoder.getRate());

    switch (m_state) {
      case CALIBRATING:
        if ((m_armEncoder.getRate() <= 0.0 && m_armEncoder.getRate() >= -Constants.kCalibrationTolerance ||
          m_armEncoder.getRate() >= 0.0 && m_armEncoder.getRate() <= Constants.kCalibrationTolerance) &&
          (double)RobotController.getFPGATime()/1000000 - m_startTime >= Constants.kCalibrationDelay) {
          moveArm(0.0);
          m_armEncoder.reset();
          m_state = State.OPERATING;
        }
        break;
    
      case OPERATING:
        moveArm(m_speed);
        break;
    }
  }
}
