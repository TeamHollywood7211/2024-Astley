package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public final class Constants {
    public static final double stickDeadband = 0.1;;

    public static final double driveSpeed = 0.99; //Set to 1 for max speed 
    //
    //  This is 0 -> 1 (0% through 100%)         
    //  For safety, I recommend either 40% or 60%
    //
    //  !!DO NOT!! SET IT TO ANYTHING BEYOND 1!!!
    //

    public static final int bot = 1; //0 = Practice, 1 = Main 



    public static class ArmConstants
    {
      public static final int armMotorID = 45; //fill in with actual thing
      public static final int wristMotorID = 47;
    
      public static final double armP = 0.075;  //Speed
      public static final double armI = 0;      //Ignore(?)
      public static final double armD = 0.0005; //Rate of Change (Multiplies the current I (which changes) by this?)

      //wrist
      public static final double wristP = 0.02;
      public static final double wristI = 0;
      public static final double wristD = 0.003;


      public static final double cli_wristP = 0.005;
      public static final double cli_wristI = 0;
      public static final double cli_wristD = 0.003;
      /*
       * 
        In case you are looking for info on how to tune these bad boys, then take a look at this crazy guide I stole online.
        P (proportional) - this is a term that when multiplied by a constant (Kp) will generate a motor speed that will help move the motor in the correct direction and speed.

        I (integral) - this term is the sum of successive errors. The longer the error exists the larger the integral contribution will be. It is simply a sum of all the errors over time. If the wrist isn’t quite getting to the setpoint because of a large load it is trying to move, the integral term will continue to increase (sum of the errors) until it contributes enough to the motor speed to get it to move to the setpoint. The sum of the errors is multiplied by a constant (Ki) to scale the integral term for the system.

        D (differential) - this value is the rate of change of the errors. It is used to slow down the motor speed if it’s moving too fast. It’s computed by taking the difference between the current error value and the previous error value. It is also multiplied by a constant (kd) to scale it to match the rest of the system.

       */
  
    }
    public static class IntakeConstants
    {
      public static final int IntakeMotor1ID = 41;
      public static final int IntakeMotor2ID = 42;
      public static final int feederMotor1ID = 43;
      public static final int feederMotor2ID = 44;
      public static final int IRSensorIntakePowerID = 0;
      public static final int IRSensorIntakeSignalID = 1;

      public static final int IRSensorShooterSignal = 2;
      public static final int IRSensorShooterSignalID = 3;

    }
    
    public static class ShooterConstants
    {
      public static final int ShooterMoverMotorID = 45;
      public static final double kP = 0.01;
      public static final double kI = 0;
      public static final double kD = 0.001;
      public static final int shooterMotor1ID = 50;
      public static final int shooterMotor2ID = 51;
      

      public static final int armMotorID = 45;
    }


    public static final int cameraResolution = 240;

    public static class ClimberConstants //Remove climber next time if need be 
    {
      public static final double kP = 0.01;;
      public static final double kI = 0;
      public static final double kD = 0.001;

      public static final int arm1ID = 60;
      public static final int arm2ID = 61;
    }

    public static class LED
    {
      public static final int CANdleID = 51;
      public static final int numLED = 308;

      public static final int teamR = 255;
      public static final int teamG = 0;
      public static final int teamB = 0;


    }
    public static class newClimberConstants
    {
      public static final int ClimberMotorLeft = 54;
      public static final int ClimberMotorRight = 55;
      public static final double kP = 0.01;
      public static final double kI = 0;
      public static final double kD = 0.0005;

      public static final double armPos = -199.15;
      public static final double wirstPos = 46.5;
    }


}
