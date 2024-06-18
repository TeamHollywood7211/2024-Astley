// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
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

public class ClimberSubsystem extends SubsystemBase {

  PIDController climberPID = new PIDController(ClimberConstants.kP, ClimberConstants.kI, ClimberConstants.kD);

  CANSparkMax climber1 = new CANSparkMax(ClimberConstants.arm1ID, MotorType.kBrushless);

  RelativeEncoder climber1Encoder = climber1.getEncoder();

  double climberSetpoint = 0;

 

  public ClimberSubsystem() {
     SmartDashboard.putNumber("Climber POS", 0);
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
    //climberSetpoint = SmartDashboard.getNumber("Climber POS", 0);
    climber1.set(MathUtil.clamp(climberPID.calculate(climber1Encoder.getPosition(), climberSetpoint), -1, 1));  
  }

  @Override
  public void simulationPeriodic() {
  }
  
  public void manual()
  {
    climberSetpoint = SmartDashboard.getNumber("Climber POS", 0);
  }
  public void manUp()
  {
    climberSetpoint += 4;
  }
  public void manDown()
  {
    climberSetpoint -= 4;
  }
  public void prepClimb()
  {
    climberSetpoint = -150;
  }
  public void lowerClimb()
  {
    climberSetpoint = 0;
  }

}
