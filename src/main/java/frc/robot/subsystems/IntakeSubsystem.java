// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;


import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

public class IntakeSubsystem extends SubsystemBase {
  
  CANSparkMax IntakeMotor1 = new CANSparkMax(IntakeConstants.IntakeMotor1ID, MotorType.kBrushless);
  CANSparkMax IntakeMotor2 = new CANSparkMax(IntakeConstants.IntakeMotor2ID, MotorType.kBrushless);
  CANSparkMax feederMotor = new CANSparkMax(IntakeConstants.feederMotor1ID, MotorType.kBrushless);
  CANSparkMax feederMotor2 = new CANSparkMax(IntakeConstants.feederMotor2ID, MotorType.kBrushless);
  DigitalInput ringIntakeSensor = new DigitalInput(IntakeConstants.IRSensorIntakeSignalID); //The IR sensor for the rings
  DigitalInput ringShooterSensor = new DigitalInput(IntakeConstants.IRSensorShooterSignalID);
  /** Creates a new ExampleSubsystem. */
  public IntakeSubsystem() {
    IntakeMotor1.restoreFactoryDefaults();
    IntakeMotor2.restoreFactoryDefaults();
    feederMotor.restoreFactoryDefaults();
    feederMotor2.restoreFactoryDefaults();
    
    feederMotor2.setInverted(false);
    //feederMotor2.follow(feederMotor);
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
    SmartDashboard.putBoolean("Ring In Shooter", !(readIntakeRingSensor())); //tells drivers if there is a ring detected by top IR sensor
    SmartDashboard.putBoolean("Ring In Intake", !(readShooterRingSensor()));
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }



  public void setIntake(double speed) //For when we want in code to have an intake set by a variable
  {
    IntakeMotor1.set(-speed);
    IntakeMotor2.set(-speed);
  }

  public void setFeeder(double speed) //Same thing but feeder
  {
    feederMotor.set(speed);
    feederMotor2.set(-speed);
  }

  public boolean auto_intakeOn() //These two are for manually turning intake on/off for pathplanner
  {
    IntakeMotor1.set(0.15);
    IntakeMotor2.set(0.15);
    feederMotor.set(-0.15);
    feederMotor2.set(0.15);
    return true;
  }
  public boolean auto_intakeOff()
  {
    IntakeMotor1.set(0.01);
    IntakeMotor2.set(0.01);
     feederMotor.set(0.01);
     feederMotor2.set(0.01);
    return true;
  }

  public boolean readIntakeRingSensor()
  {
    return !ringShooterSensor.get(); //IR sensors reads true if beam isnt broken. We invert this since we want to say "FALSE, WE SEE NO RING"
  }
  public boolean readShooterRingSensor()
  {
    return !ringIntakeSensor.get();
  }

}
