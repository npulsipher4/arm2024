// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.arm;

public class RobotContainer {

  public final arm m_arm = new arm();
  public static final XboxController m_controller = new XboxController(0);

  public double clampJoystick(double jSpeed) {
    if (jSpeed < 0.0 && jSpeed > -Constants.kJoystickTolerance ||
    jSpeed > 0.0 && jSpeed < Constants.kJoystickTolerance) {
      return 0.0;
    }
    return jSpeed;
  }

  public RobotContainer() {
    configureBindings();
    m_arm.setDefaultCommand(
      new RunCommand(
        () -> {
          m_arm.setArmSpeed(clampJoystick(m_controller.getRightX()) * Constants.kMaxVelocity);
        }, m_arm)
    );
  }

  private void configureBindings() {
    new Trigger(() -> m_controller.getRightBumperPressed())
    .onTrue(Commands.runOnce(() -> m_arm.indexArm(), m_arm));

    new Trigger(() -> m_controller.getXButtonPressed())
    .onTrue(Commands.runOnce(() -> m_arm.setOperatingState()));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
