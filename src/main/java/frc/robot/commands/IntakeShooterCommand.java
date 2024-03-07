// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/** An example command that uses an example subsystem. */
public class IntakeShooterCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final IntakeSubsystem m_intakeSubsystem;
  private final ShooterSubsystem m_shooterSubsystem;
  private final CommandXboxController m_controller;


  public IntakeShooterCommand(IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem, CommandXboxController controller) {
    m_intakeSubsystem = intakeSubsystem;
    m_shooterSubsystem = shooterSubsystem;
    m_controller = controller;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {



    if(m_intakeSubsystem.readShooterRingSensor() == true) //If we have a ring in the system, spool the shooter
   {
    m_shooterSubsystem.setShooterSpeed(RobotContainer.shooterSpeed/2);
   }
    //Shooter

   if(m_controller.rightTrigger().getAsBoolean())
   {
      m_shooterSubsystem.setShooterSpeed(RobotContainer.shooterSpeed); //when in doubt, -0.6
      if(m_intakeSubsystem.readShooterRingSensor())
      {
        m_intakeSubsystem.setFeeder(-0.1);
      }
      //m_intakeSubsystem.setFeeder(-1);
   } 
   if(m_controller.rightBumper().getAsBoolean())
   {
      m_shooterSubsystem.setShooterSpeed(-0.15);
      //m_intakeSubsystem.setFeeder(1);
   }

   if((!m_controller.rightTrigger().getAsBoolean()) && (!m_controller.rightBumper().getAsBoolean()) && (!m_intakeSubsystem.readShooterRingSensor()))
   {
      m_shooterSubsystem.setShooterSpeed(0);
      
   }

   //Intake

   if(m_controller.leftTrigger().getAsBoolean())
   {
    if(m_intakeSubsystem.readShooterRingSensor() == false) //Only  intake till there is a ring near the shooter
    {
      m_intakeSubsystem.setIntake(-0.10);
      m_intakeSubsystem.setFeeder(-0.10);
    }
    else
    {
      m_intakeSubsystem.setIntake(0);
      m_intakeSubsystem.setFeeder(0);
    }
  }
   if(m_controller.leftBumper().getAsBoolean())
   {
    m_intakeSubsystem.setIntake(0.15);
    m_intakeSubsystem.setFeeder(0.15);
   }
   if((!m_controller.leftTrigger().getAsBoolean()) && (!m_controller.leftBumper().getAsBoolean()) && (!m_controller.rightTrigger().getAsBoolean()))
   {
    m_intakeSubsystem.setIntake(0);
    m_intakeSubsystem.setFeeder(0);
   }





/* 
  if((!m_controller.leftTrigger().getAsBoolean()) && (!m_controller.leftBumper().getAsBoolean()))
   {
      if((!m_controller.rightTrigger().getAsBoolean()) && (!m_controller.rightBumper().getAsBoolean()))
    {
      m_intakeSubsystem.setFeeder(0);
        
    }
   }*/





  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
