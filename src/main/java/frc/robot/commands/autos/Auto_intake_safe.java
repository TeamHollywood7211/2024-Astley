// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class Auto_intake_safe extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final IntakeSubsystem m_intake;

  Double startTime; //TL;DR timers suck. (They are useful just not here)
  boolean finished = false;
  boolean resetTimer = true;
  int timesRan = 1;
  double timer = 0;
  double timeToKill = 15;
  //Timer time;
 // double timer = 0;

  public Auto_intake_safe(IntakeSubsystem subsystem) {
    m_intake = subsystem;

    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    System.out.println("AUTO INTAKE: START THE INTAKE!!");
    //time.reset();
    startTime = DriverStation.getMatchTime();
  }

  @Override
  public void execute() {
    System.out.println("AUTO INTAKE: TIME: " + (startTime - DriverStation.getMatchTime()) + "  ");
    System.out.println("AUTO INTAKE: RING? :" + m_intake.readShooterRingSensor() + "\n");



    if(resetTimer)
    {
      startTime = DriverStation.getMatchTime();
      resetTimer = false;
      System.out.println("AUTO INTAKE: RESETING TIMER");
      timer = 0;
    }

    timer = (startTime - DriverStation.getMatchTime());
    //time.start();
    //timer = time.get();
    if((m_intake.readShooterRingSensor() == false) && (timer < timeToKill))
    {
      m_intake.setIntake(-0.2); //0.15 is the original speed
      m_intake.setFeeder(-0.15);
    }
    else
    {
      m_intake.setIntake(0);
      m_intake.setFeeder(0);
    }
    if(timer > timeToKill)
    {
      m_intake.setIntake(0);
      m_intake.setFeeder(0);
    }
    if((timer > timeToKill + 0.1) || (m_intake.readShooterRingSensor() == true))
    {
      System.out.println("AUTO INTAKE: MAY OR MAY NOT HAVE THE RING, IDC WE MOVIN' "); 
      System.out.println("AUTO INTAKE: TIME RAN: " + timesRan);                          
      finished = true;
    }
    else
    {
      finished = false;
    }

  }

  @Override
  public void end(boolean interrupted) {
    resetTimer = true;
    timesRan++;
    //time.stop();
    //time.reset();
  }

  @Override
  public boolean isFinished() {
    //startTime = DriverStation.getMatchTime();

    return finished; 
  }
}
