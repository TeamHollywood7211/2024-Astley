// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import frc.robot.Constants.LED;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class Auto_intake_safe extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final IntakeSubsystem m_intake;
  private final LEDSubsystem m_led;
  private final ShooterSubsystem m_shooter;

  Double startTime; //TL;DR timers suck. (They are useful just not here)
  boolean finished = false;
  boolean resetTimer = true;
  int timesRan = 1;
  double timer = 0;
  double timeToKill = 4; //Time until to give up on the intake
  //Timer time;
 // double timer = 0;

  public Auto_intake_safe(IntakeSubsystem subsystem, LEDSubsystem led, ShooterSubsystem shooter) {
    m_intake = subsystem;
    m_led = led;
    m_shooter = shooter;

    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    System.out.println("AUTO INTAKE: START THE INTAKE!!");
    //time.reset();
    startTime = DriverStation.getMatchTime(); //Reset the timer
  }

  @Override
  public void execute() {
    
    System.out.println("AUTO INTAKE: TIME: " + (startTime - DriverStation.getMatchTime()) + "  ");
    System.out.println("AUTO INTAKE: RING? :" + m_intake.readShooterRingSensor() + "\n");
    


    if(resetTimer)
    {
      startTime = DriverStation.getMatchTime(); //sets the timer the timer started to now
      resetTimer = false; //says "yo we resetted"
      System.out.println("AUTO INTAKE: RESETING TIMER");
      timer = 0; //sets the actual timer variable to 0
    }

    if(timer < timeToKill/2)
    {
      m_shooter.setShooterSpeed(-0.2);
    }
    else
    {
      m_shooter.setShooterSpeed(0);
    }
    
    timer = (startTime - DriverStation.getMatchTime()); //calculate time between now and last time we reset timer
    //time.start();
    //timer = time.get();
    if((m_intake.readShooterRingSensor() == false) && (timer < timeToKill)) //if timer aint done, or we dont have a piece
    {
      m_intake.setIntake(-0.2); //Run that intake
      m_intake.setFeeder(-0.05);
    }
    else //else we done :3
    {
      m_intake.setIntake(0);
      m_intake.setFeeder(0);
    }
    if(timer > timeToKill) //if we dont grab and we pass our time to kill
    {
      m_intake.setIntake(0); //stop it
      m_intake.setFeeder(0);
    }
    if((timer > timeToKill + 0.1) || (m_intake.readShooterRingSensor() == true)) //if we grab a piece (or we pass our time to kill by a weeee bit)
    {
      System.out.println("AUTO INTAKE: MAY OR MAY NOT HAVE THE RING, IDC WE MOVIN' "); 
      System.out.println("AUTO INTAKE: TIME RAN: " + timesRan);                          
      finished = true; //say we done :3
    }
    else
    {
      finished = false; //if not done, say we aint done
    }

    if(m_intake.readIntakeRingSensor() == true) //LED stuff so that we look cool and stuff
    {
       m_led.setPurple();
    }
    if(m_intake.readShooterRingSensor() == true)
    {
     m_led.setGreen();
    }
    if((!m_intake.readIntakeRingSensor()) && (!m_intake.readShooterRingSensor()))
    {
     m_led.setTeam();
    }

  }

  @Override
  public void end(boolean interrupted) {
    resetTimer = true;
    timesRan++; //Just a bit of debug info for me 
  }

  @Override
  public boolean isFinished() {
    return finished; 
  }
}
