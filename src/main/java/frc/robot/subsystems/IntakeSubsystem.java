// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

public class IntakeSubsystem extends SubsystemBase {
  
  CANSparkMax IntakeMotor1 = new CANSparkMax(IntakeConstants.IntakeMotor1ID, MotorType.kBrushed);
  CANSparkMax IntakeMotor2 = new CANSparkMax(IntakeConstants.IntakeMotor2ID, MotorType.kBrushed);
  DigitalInput IR = new DigitalInput(0);
  /** Creates a new ExampleSubsystem. */
  public IntakeSubsystem() {
    IntakeMotor1.restoreFactoryDefaults();
    IntakeMotor2.restoreFactoryDefaults();
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
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void intake()
  {
    IntakeMotor1.set(1);
    IntakeMotor2.set(-1);
  }
  public void outake()
  {
    IntakeMotor1.set(-1);
    IntakeMotor1.set(1);
  }
  public void stopIntake()
  {
    IntakeMotor1.set(0);
    IntakeMotor2.set(0);
  }

}
