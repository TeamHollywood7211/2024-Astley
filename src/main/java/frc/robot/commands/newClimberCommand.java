// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.newClimberSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class newClimberCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final newClimberSubsystem m_subsystem;
  private final CommandXboxController m_controller;
  public newClimberCommand(newClimberSubsystem subsystem, CommandXboxController controller) {
    m_subsystem = subsystem;
    m_controller = controller;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if(m_controller.b().getAsBoolean())
    {
      m_subsystem.toggleArm();
    }

    if(m_controller.povUp().getAsBoolean())
    {
      m_subsystem.manArm(1);
    }
    if(m_controller.povDown().getAsBoolean())
    {
      m_subsystem.manArm(-1);
    }
    if(m_controller.x().getAsBoolean())
    {
      m_subsystem.trap();
    }
    if(m_controller.povLeft().getAsBoolean())
    {
      m_subsystem.manArm(0);
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
