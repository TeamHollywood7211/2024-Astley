// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTableInstance;

public class VisionSubsystem extends SubsystemBase {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    boolean runLimeLight = false;
    
    // how many degrees back is your limelight rotated from perfectly vertical?
      
    double limelightMountAngleDegrees = 25.0; 

        // distance from the center of the Limelight lens to the floor
    double limelightLensHeightInches = 20.0; 
  /** Creates a new ExampleSubsystem. */
  public VisionSubsystem() {


  }


  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void autoTarget()
  {
    runLimeLight = true;
  }

  public void stopLimeLight()
  {
    runLimeLight = false;
  }
}
