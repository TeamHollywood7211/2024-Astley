// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


import frc.robot.Constants.ShooterConstants;



public class ShooterSubsystem extends SubsystemBase {
  PIDController pid = new PIDController(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD);
  //remember, P is like where you wanna get, I is smth and D is like some slow down thingy.
  CANSparkMax shooterMotor1 = new CANSparkMax(ShooterConstants.shooterMotor1ID, MotorType.kBrushless);
  CANSparkMax shooterMotor2 = new CANSparkMax(ShooterConstants.shooterMotor2ID, MotorType.kBrushless);
  
  //CANSparkMax armMotor = new CANSparkMax(ShooterConstants.armMotorID, MotorType.kBrushless);

  

  public ShooterSubsystem() {
    shooterMotor1.restoreFactoryDefaults();
    shooterMotor2.restoreFactoryDefaults();

    shooterMotor1.setSmartCurrentLimit(40);
    shooterMotor2.setSmartCurrentLimit(40);

    //shooterMotor2.follow(shooterMotor1); 

    



  }

  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          
        });
  }


  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    
  /*
  
  when wrist was here
  SmartDashboard.putNumber("Shooter Angle: ", shooterAngleEncoder.getPosition());
    //setpoint = MathUtil.clamp(setpoint, 0, 60);

    shooterAngleMotor.set(MathUtil.clamp(pid.calculate(shooterAngleEncoder.getPosition(), setpoint), -0.75, 0.75));
*/
    //mostly built on the cool pid.calculate() function. Clamps the speed AND the setpoints so
    //I, the operator, and the code doesnt smash itself into the robot, ultimately destroying
    //the fragile Neo Vortex (which can catch on fire :3)
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }


  public void setShooterSpeed(double speed)  //sets motor speed based on var
  {
    speed = MathUtil.clamp(speed, -1,1); //Those things are effing powerful, dont wanna overshoot their range
    shooterMotor1.set(-speed); 
    shooterMotor2.set(speed);
  }

  public boolean auto_shooterOn() //auton shooter for pathplanner
  {
    shooterMotor1.set(-1);
    shooterMotor2.set(1);
    return true;
  }

  public boolean auto_shooterOff()
  {
    shooterMotor1.set(0.0);
    shooterMotor2.set(0.0);
    return true;
  }
  
}
