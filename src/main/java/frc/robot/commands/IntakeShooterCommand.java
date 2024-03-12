// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/** An example command that uses an example subsystem. */
public class IntakeShooterCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final IntakeSubsystem m_intakeSubsystem;
  private final ShooterSubsystem m_shooterSubsystem;
  private final LEDSubsystem m_led;
  private final CommandXboxController m_controller;


  public IntakeShooterCommand(IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem, LEDSubsystem ledSubsystem, CommandXboxController controller) {
    m_intakeSubsystem = intakeSubsystem;
    m_shooterSubsystem = shooterSubsystem;
    m_controller = controller;
    m_led = ledSubsystem;
    addRequirements(intakeSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {

    //CONTROLS -
    // (I take controls like a FPS)
    // Shooter = right 
    // Intake  = left
    //
    // Triggers are the actual pulling in and shooting
    // Bumpers are "We need to backup" / retraction
    //
    //----

    if(m_intakeSubsystem.readShooterRingSensor() == true) //If we have a ring in the system, spool the shooter
   {
   // m_shooterSubsystem.setShooterSpeed(RobotContainer.shooterSpeed*0.75);
   }
    //Shooter\
   if(m_controller.x().getAsBoolean()) //If we have a ring in the system, spool the shooter
   {
    m_shooterSubsystem.setShooterSpeed(RobotContainer.shooterSpeed);
   }


   if(m_controller.rightTrigger().getAsBoolean()) //On standard shoot
   {
      m_shooterSubsystem.setShooterSpeed(RobotContainer.shooterSpeed);
      if(m_intakeSubsystem.readShooterRingSensor())
      {
        m_intakeSubsystem.setFeeder(-0.1);
      }
   } 
   if(m_controller.rightBumper().getAsBoolean()) //Retracing from shooter
   {
      m_shooterSubsystem.setShooterSpeed(-0.15);
   }

   //Stopping shooter under any circumstance that it isnt on
   if((!m_controller.rightTrigger().getAsBoolean()) && (!m_controller.rightBumper().getAsBoolean()) && (!m_intakeSubsystem.readShooterRingSensor()) && (!m_controller.x().getAsBoolean()))
   {
      m_shooterSubsystem.setShooterSpeed(0);
      
   }

   //Intake

   if(m_controller.leftTrigger().getAsBoolean()) //Intaking in a piece
   {
    if(m_intakeSubsystem.readShooterRingSensor() == false) //Only  intake till there isnt a ring near the shooter
    {
      m_intakeSubsystem.setIntake(-0.1);
      m_intakeSubsystem.setFeeder(-0.1);
    }
    else
    {
      m_intakeSubsystem.setIntake(0);
      m_intakeSubsystem.setFeeder(0);
    }
  }

   if(m_controller.leftBumper().getAsBoolean()) //Pulling a ring out
   {
    m_intakeSubsystem.setIntake(0.30);
    m_intakeSubsystem.setFeeder(0.15);
   }
   //Stop intake under any circumstance that it shouldnt be running
   if((!m_controller.leftTrigger().getAsBoolean()) && (!m_controller.leftBumper().getAsBoolean()) && (!m_controller.rightTrigger().getAsBoolean()))
   {
    m_intakeSubsystem.setIntake(0);
    m_intakeSubsystem.setFeeder(0);
   }
   if(m_intakeSubsystem.readShooterRingSensor() == true)
   {
      m_led.setOrange();
   }
   else
   {
    m_led.setTeam();
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
}
