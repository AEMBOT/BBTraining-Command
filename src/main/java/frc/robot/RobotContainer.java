// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.DriveSubsystem;
import static frc.robot.constants.*;


public class RobotContainer {
  private DriveSubsystem drive = new DriveSubsystem();

  private final CommandXboxController m_controller = new CommandXboxController(xBoxControllerPort);

  public RobotContainer() {
    configureBindings();

    drive.setDefaultCommand(
      new RunCommand(
        () -> {
          drive.arcadeDrive(m_controller.getLeftY(), m_controller.getRightX());
        },
        drive)
    );
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
