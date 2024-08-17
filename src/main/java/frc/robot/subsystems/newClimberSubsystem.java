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
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.newClimberConstants;

public class newClimberSubsystem extends SubsystemBase {


  static boolean armDown = false;
  
  int posUp = 0;
  int posDown = 0;
  CANSparkMax ArmMotorL = new CANSparkMax(newClimberConstants.ClimberMotorLeft, MotorType.kBrushless);
  CANSparkMax ArmMotorR = new CANSparkMax(newClimberConstants.ClimberMotorRight, MotorType.kBrushless);
  PIDController armPID = new PIDController(newClimberConstants.kP, newClimberConstants.kI, newClimberConstants.kD);


  public RelativeEncoder ArmEncoder = ArmMotorR.getEncoder();

  double ArmSetpoint = ArmEncoder.getPosition();


  public newClimberSubsystem() {
    ArmMotorR.restoreFactoryDefaults();
    ArmMotorL.restoreFactoryDefaults();
    ArmMotorR.setSmartCurrentLimit(40);
    ArmMotorL.setSmartCurrentLimit(40);
    //ArmMotorR.setInverted(false);
    //ArmMotorR.follow(ArmMotorL);
    
  }

  

 Command exampleMethodCommand() {
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }


  public boolean exampleCondition() {
    return false;
  }

  @Override
  public void periodic() {
    double currentPos = ArmEncoder.getPosition();
    SmartDashboard.putNumber("New Arm Position", currentPos);
    ArmMotorL.set(MathUtil.clamp(armPID.calculate(-currentPos, ArmSetpoint), -0.5, 0.5));
    ArmMotorR.set(MathUtil.clamp(armPID.calculate(currentPos, -ArmSetpoint), -0.5, 0.5));
    SmartDashboard.putNumber("Percentage to Pos: ", currentPos / ArmSetpoint);
  }

  @Override
  public void simulationPeriodic() {
  }

  public void toggleArm()
  {
    if(armDown)
    {
      ArmSetpoint = posUp;
    }
    else
    {
      ArmSetpoint = posDown;
    }
    armDown = !armDown;
  }

  public void trap()
  {
    ArmSubsystem.armSetpoint = newClimberConstants.armPos;      
    ArmSubsystem.wristSetpoint = newClimberConstants.wirstPos;  
  }
  
  public void manArm(int pos)
  {
    ArmSetpoint += pos;
  }

}
