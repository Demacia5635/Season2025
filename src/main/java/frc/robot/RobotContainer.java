package frc.robot;

import frc.robot.slug.subsystems.Slug;
import frc.robot.utils.LogManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
  LogManager logManager;

  CommandXboxController controller;
  Slug slug;

  public RobotContainer() {
    logManager = new LogManager();

    controller = new CommandXboxController(0);

    configureBindings();
  }

  private void configureBindings() {
    controller.povDown().onTrue(slug.getFeedCommand());
    controller.povUp().onTrue(slug.getExcreteCommand());
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
