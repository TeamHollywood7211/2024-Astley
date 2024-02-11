// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/** An example command that uses an example subsystem. */
public class ShooterCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ShooterSubsystem m_subsystem;
  private final CommandXboxController m_controller;

  public ShooterCommand(ShooterSubsystem subsystem, CommandXboxController controller) {
    m_subsystem = subsystem;
    this.m_controller = controller;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if(m_controller.leftBumper().getAsBoolean())
    {
      m_subsystem.runGripSpeed(.15, .15); //max speed
    }
   else if(m_controller.getLeftTriggerAxis() > 0.05)
   {
    // If you want FULL SEND
    m_subsystem.runGripSpeed(-1, -1 );
    // If you want adjustable speed
    //m_subsystem.runGripSpeed(-1* m_controller.getLeftTriggerAxis(), -1 * m_controller.getLeftTriggerAxis());
  }
  else if(m_controller.getRightTriggerAxis() > 0.05)
  {
    m_subsystem.runRearSpeed(1);
  }
  else if (m_controller.rightBumper().getAsBoolean())
  {
    m_subsystem.runRearSpeed(-1);
  }
  else
  {
      //m_gripSubsystem.setGripOut();
      m_subsystem.runGripSpeed(0, 0);
      m_subsystem.runRearSpeed(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public void outTake() {

   // m_subsystem.runGripSpeed(Constants.outakeSpeed);
  }
  public void inTake() {

   // m_subsystem.runGripSpeed(Constants.intakeSpeed);
  }
}
