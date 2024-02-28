// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.util.PIDConstants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
    CANSparkMax ClimberMotor1 = new CANSparkMax(ClimberConstants.arm1ID, MotorType.kBrushless);
    CANSparkMax ClimberMotor2 = new CANSparkMax(ClimberConstants.arm2ID, MotorType.kBrushless);
    PIDController pid = new PIDController(ClimberConstants.kP, ClimberConstants.kI, ClimberConstants.kD);
    RelativeEncoder ClimberEncoder = ClimberMotor1.getEncoder();
    double setpoint = 0;

  public ClimberSubsystem() {
    ClimberMotor1.restoreFactoryDefaults();
    ClimberMotor2.restoreFactoryDefaults();
    //ClimberMotor2.follow(ClimberMotor1);
    ClimberMotor2.setInverted(false);
    ClimberMotor1.setInverted(false);
    
    

  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Climber Pos", ClimberEncoder.getPosition());
    //ClimberMotor1.set(pid.calculate(ClimberEncoder.getPosition(), setpoint));
    //ClimberMotor2.set(pid.calculate(ClimberEncoder.getPosition(),-setpoint));
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }


  public void lowerClimbers()
  {
    setpoint = 0;
  }
  public void raiseClimbers()
  {
    setpoint = 20;
  }


  public void manuClimberUp()
  {
    //setpoint ++;
  }
  public void manuClimberDown()
  {
    //setpoint --;
  }

  public void climberSet(double speed)
  {
    ClimberMotor1.set(speed);
    ClimberMotor2.set(-speed);
  }

  public void resetClimberZero()
  {
    ClimberEncoder.setPosition(0);
  }
}
