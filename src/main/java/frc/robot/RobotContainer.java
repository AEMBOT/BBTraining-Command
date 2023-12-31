// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import static frc.robot.Constants.*;


public class RobotContainer {
    private final DriveSubsystem drive = new DriveSubsystem();
    private final IntakeSubsystem intake = new IntakeSubsystem();
    private final ShooterSubsystem shooter = new ShooterSubsystem();
    SlewRateLimiter filterx = new SlewRateLimiter(slewRate);
    SlewRateLimiter filtery = new SlewRateLimiter(slewRate);

    private final CommandXboxController controller1 = new CommandXboxController(xBoxControllerPort1);
    private final CommandXboxController controller2 = new CommandXboxController(xBoxControllerPort2);
    private final PathPlannerPath pathTrajectory = PathPlannerPath.fromPathFile("Start Path");

    public RobotContainer() {
        configureBindings();

        drive.setDefaultCommand(
                new RunCommand(
                        () -> drive.arcadeDrive(
                                filtery.calculate(controller1.getLeftY()),
                                filterx.calculate(controller1.getRightX()) * turnSensitivity
                        ),
                        drive
                )
        );


        controller1.rightBumper().whileTrue(intake.enableIntake());

        controller1.b().whileTrue(intake.reverseIntake());

        controller2.y().onTrue(shooter.runIndexerCommand()); // Possibly temporary binding
        controller2.a().whileTrue(shooter.reverseIndexerCommand()); // Possibly temporary binding
        controller1.a().whileTrue(shooter.reverseIndexerCommand()); // Possibly temporary binding

        controller2.x().whileTrue(shooter.shootBackwardsCommand());

        controller2.leftBumper().onTrue(shooter.shootCommand());
    }

    private void configureBindings() {
    }

    public Command getAutonomousCommand() {
        return drive.randomAuto();
    }

    public void onEnabled() {

    }

    public void onInit() {

    }
}
