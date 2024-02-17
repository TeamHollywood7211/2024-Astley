// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ShooterConstants;



public class ShooterSubsystem extends SubsystemBase {
  PIDController pid = new PIDController(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD);
  //remember, P is like where you wanna get, I is smth and D is like some slow down thingy.
  CANSparkMax shooterMotor1 = new CANSparkMax(ShooterConstants.shooterMotor1ID, MotorType.kBrushless);
  CANSparkMax shooterMotor2 = new CANSparkMax(ShooterConstants.shooterMotor2ID, MotorType.kBrushless);

  CANSparkMax shooterAngleMotor = new CANSparkMax(ShooterConstants.ShooterMoverMotorID, MotorType.kBrushless);
  public RelativeEncoder shooterAngleEncoder = shooterAngleMotor.getEncoder();
  
  double setpoint = 0;

  public ShooterSubsystem() {
    shooterMotor1.restoreFactoryDefaults();
    shooterMotor2.restoreFactoryDefaults();

    shooterMotor2.follow(shooterMotor1);




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
    
    SmartDashboard.putNumber("Shooter Angle: ", shooterAngleEncoder.getPosition());
    setpoint = MathUtil.clamp(setpoint, 0, 60);

    shooterAngleMotor.set(MathUtil.clamp(pid.calculate(shooterAngleEncoder.getPosition(), setpoint), -0.75, 0.75));

    //mostly built on the cool pid.calculate() function. Clamps the speed AND the setpoints so
    //I, the operator, and the code doesnt smash itself into the robot, ultimately destroying
    //the fragile Neo Vortex (which can catch on fire :3)
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
  public void intake() //useless, reuse for pathplanner
  {
    shooterMotor1.set(0.25);
    shooterMotor2.set(0.25);
  }
  public void outake()
  {
    shooterMotor1.set(-0.15);
    shooterMotor2.set(-0.15);
  }
  public void stopShooter()
  {
    shooterMotor1.set(0);
    shooterMotor2.set(0);

  }



  public void moveShooter(double speed)
  {
    setpoint += speed;
  }

  public void shooterPosLow() 
  {
    setpoint = 0;
    
  }

  public void shooterPosMid()
  {
    setpoint = 30;
  }
  public void shooterPosHigh()
  {
    setpoint = 47;
  }


  public void shooterPosRingPickup()
  {
    setpoint = 38;
    RobotContainer.shooterSpeed = 0.60;

  }
  public void shooterPosSpeaker()
  {
    setpoint = 56;
    RobotContainer.shooterSpeed = 0.50;
  }
  public void shooterPosLongShot()
  {
    setpoint = 23;
    RobotContainer.shooterSpeed = 1;
  }

  public void shooterPosSuperLongShot()
  {
    setpoint = 20;
    RobotContainer.shooterSpeed = 1;
  }
  public void setShooterSpeed(double speed)
  {
    speed = MathUtil.clamp(speed, -1,1);
    shooterMotor1.set(-speed);
    shooterMotor2.set(-speed);
  }

}
