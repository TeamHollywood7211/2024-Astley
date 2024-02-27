// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.IntakeShooterCommand;
import frc.robot.commands.moveArmCommand;
import frc.robot.commands.autos.Auto_intake;
import frc.robot.commands.autos.Auto_shoot;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RobotContainer {
  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity
  //private final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();
  public static double shooterSpeed = 0.60;
  private final SendableChooser<Command> autoChooser;

  private static boolean doTelemetry = true;



  //Subsystems
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final ArmSubsystem armSubsystem = new ArmSubsystem();
    


  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController m_driver = new CommandXboxController(0); // My joystick
  private final CommandXboxController m_operator = new CommandXboxController(1);
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain


  //Commands


  private final moveArmCommand m_moveArm = new moveArmCommand(armSubsystem, m_operator);
  private final IntakeShooterCommand m_intakeShooterCommand = new IntakeShooterCommand(intakeSubsystem, shooterSubsystem, m_operator);


  //AUTO COMMANDS
  private final Auto_shoot au_shoot = new Auto_shoot(shooterSubsystem, intakeSubsystem);
  private final Auto_intake au_intake = new Auto_intake(intakeSubsystem);









  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();



  public void createFrontUsbCamera(){
    //UsbCamera frontUsbCamera = new UsbCamera("frontUsbCamObject", 0 );
    //frontUsbCamera.setResolution(160, 120);
    CameraServer.startAutomaticCapture();
    CvSink cvSink = CameraServer.getVideo();

    CvSource outputStream = CameraServer.putVideo("Blur", 640, 480);
  }



  /* Path follower */
  //private Command runAuto = drivetrain.getAutoPath("Tests");

  //private final Telemetry logger = new Telemetry(MaxSpeed);
  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-m_driver.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-m_driver.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-m_driver.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ).ignoringDisable(true));

    m_driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
    m_driver.b().whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-m_driver.getLeftY(), -m_driver.getLeftX()))));

    // reset the field-centric heading on left bumper press (reset gyro?)
    m_driver.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    //drivetrain.registerTelemetry(logger::telemeterize);

    m_driver.pov(0).whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0.5).withVelocityY(0)));
    m_driver.pov(180).whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(-0.5).withVelocityY(0)));



    /* Bindings for drivetrain characterization */
    /* These bindings require multiple buttons pushed to swap between quastatic and dynamic */
    /* Back/Start select dynamic/quasistatic, Y/X select forward/reverse direction */
    m_driver.back().and(m_driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    m_driver.back().and(m_driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    m_driver.start().and(m_driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    m_driver.start().and(m_driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
    
    //SHOOTER//
    new Trigger(m_operator.rightTrigger()).onTrue(m_intakeShooterCommand);
    new Trigger(m_operator.rightBumper()).onTrue(m_intakeShooterCommand);
    
    new Trigger(m_operator.rightTrigger()).and(m_operator.rightBumper()).onFalse(m_intakeShooterCommand);



      //INTAKE//
    new Trigger(m_operator.leftTrigger()).onTrue(m_intakeShooterCommand);
    new Trigger(m_operator.leftBumper()).onTrue(m_intakeShooterCommand);
    
    new Trigger(m_operator.leftTrigger()).or(m_operator.leftBumper()).onFalse(m_intakeShooterCommand);

    //Move Arm
    new Trigger(m_operator.rightStick()).whileTrue(m_moveArm);
    new Trigger(m_operator.leftStick()).whileTrue(m_moveArm);

    new Trigger(m_operator.povUp()).onTrue(new InstantCommand(armSubsystem::posAmp));
    new Trigger(m_operator.povDown()).onTrue(new InstantCommand(armSubsystem::posExLong));
    new Trigger(m_operator.povRight()).onTrue(new InstantCommand(armSubsystem::posMid));
    new Trigger(m_operator.povLeft()).onTrue(new InstantCommand(armSubsystem::posLong));

    new Trigger(m_operator.b()).onTrue(new InstantCommand(armSubsystem::posZero));

    new Trigger(m_operator.y()).onTrue(new InstantCommand(armSubsystem::posClimb));

  }



  public RobotContainer() {


    ////Auto Commands////
    //Auto Actions
    NamedCommands.registerCommand("act_shoot", au_shoot); //shoot that thang 
    NamedCommands.registerCommand("act_intake", au_intake); //does sick auto intake stuff
    NamedCommands.registerCommand("act_intake_on", new InstantCommand(intakeSubsystem::auto_intakeOn));
    NamedCommands.registerCommand("act_intake_off", new InstantCommand(intakeSubsystem::auto_intakeOff));
    NamedCommands.registerCommand("act_shooter_on", new InstantCommand(shooterSubsystem::auto_shooterOn));
    NamedCommands.registerCommand("act_shooter_off", new InstantCommand(shooterSubsystem::auto_shooterOff));



    //Auto Positions
    NamedCommands.registerCommand("pos_amp", new InstantCommand(armSubsystem::posAmp)); //set position of that thangs//
    NamedCommands.registerCommand("pos_exlong", new InstantCommand(armSubsystem::posExLong)); 
    NamedCommands.registerCommand("pos_mid", new InstantCommand(armSubsystem::posMid));
    NamedCommands.registerCommand("pos_long", new InstantCommand(armSubsystem::posLong));
    NamedCommands.registerCommand("pos_zero", new InstantCommand(armSubsystem::posZero)); 

    

    //Work on auto targetting to add the following commands
    /*
     * aim_speaker = aiming at speaker
     * 
     */
    //NamedCommands.registerCommand("aim_speaker", new InstantCommand(drivetrain::aimSpeaker));




    //turns out you have to put autochooser AFTER the named commands thingy
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    createFrontUsbCamera();
    configureBindings();
  }

  public Command getAutonomousCommand() {
    /* First put the drivetrain into auto run mode, then run the auto */
    return autoChooser.getSelected();
    //return runAuto;
  }
}
