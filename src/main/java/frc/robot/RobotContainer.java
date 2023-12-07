// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

import static frc.robot.Constants.*;

import com.pathplanner.lib.path.PathPlannerPath;


public class RobotContainer {
  private DriveSubsystem drive = new DriveSubsystem();
  private IntakeSubsystem intake = new IntakeSubsystem();
  SlewRateLimiter filterx = new SlewRateLimiter(slewrate);
  SlewRateLimiter filtery = new SlewRateLimiter(slewrate);

  private final CommandXboxController m_controller = new CommandXboxController(xBoxControllerPort);
  private final PathPlannerPath pathTrajectory = PathPlannerPath.fromPathFile("Start Path");

  public RobotContainer() {
    configureBindings();

    drive.setDefaultCommand(
      new RunCommand(
        () -> drive.arcadeDrive(
          filtery.calculate(m_controller.getLeftY()), 
          filterx.calculate(m_controller.getRightX()) * turnSensitivity
        ), 
        drive
      )
    );

    m_controller.rightBumper().whileTrue(intake.enableIntake());
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return drive.followPathCommand(pathTrajectory);
  }

  public void onEnabled() {

  }

  public void onInit() {

  }
}
