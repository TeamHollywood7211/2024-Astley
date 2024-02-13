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
import frc.robot.Constants.ShooterConstants;



public class ShooterSubsystem extends SubsystemBase {
  PIDController pid = new PIDController(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD);
  CANSparkMax shooterMotor1 = new CANSparkMax(ShooterConstants.shooterMotor1ID, MotorType.kBrushless);
  CANSparkMax shooterMotor2 = new CANSparkMax(ShooterConstants.shooterMotor2ID, MotorType.kBrushless);

  CANSparkMax shooterAngleMotor = new CANSparkMax(ShooterConstants.ShooterMoverMotorID, MotorType.kBrushed);
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
    
    shooterAngleMotor.set(MathUtil.clamp(pid.calculate(shooterAngleEncoder.getPosition(), setpoint), -0.5, 0.5));
    // This method will be called once per scheduler run
  
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
  public void intake()
  {
    shooterMotor1.set(1);
    shooterMotor2.set(1);
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

  public void shooterPosA() //fill in with the actual robot pos later plz :3
  {
    setpoint = 0;
  }

  public void shooterPosB()
  {
    setpoint = 0;
  }
  


}
