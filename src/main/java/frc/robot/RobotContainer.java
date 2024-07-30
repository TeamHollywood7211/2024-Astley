// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.IntakeShooterCommand;
import frc.robot.commands.moveArmCommand;
//import frc.robot.commands.moveClimberCommand;
import frc.robot.commands.autos.Auto_intake;
import frc.robot.commands.autos.Auto_intake_safe;
import frc.robot.commands.autos.Auto_intake_unsafe;
import frc.robot.commands.autos.Auto_shoot;
import frc.robot.commands.autos.Auto_shoot_safe;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
//import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
//import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


import edu.wpi.first.wpilibj.DataLogManager;


/*
 * 
 *  Something that needs to be acknowledged is that
 *  this code is built on the holy prayers of the semicolon;
 *  ALL HAIL THE SEMICOLON;
 *  ALL HAIL THE SEMICOLON;
 *  ;  ;  ;  ;  ;  ;  ;  ;
 * 
 */


public class RobotContainer {
  static double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity
  static Rotation2d cachedAngle = Rotation2d.fromDegrees(0);
  public static double shooterSpeed = 0.65;
  private final SendableChooser<Command> autoChooser;

  // Subsystems
  public final ShooterSubsystem shooterSubsystem = new ShooterSubsystem(); //This is a bad naming scheme, dont name like this
  public final IntakeSubsystem intakeSubsystem = new IntakeSubsystem(); //These are our subsystems
  private final ArmSubsystem armSubsystem = new ArmSubsystem();
  public final LEDSubsystem ledSubsystem = new LEDSubsystem();
  //public final ClimberSubsystem climberSubsystem = new ClimberSubsystem();

  private final CommandXboxController m_driver = new CommandXboxController(0); //Driver joystick
  private final CommandXboxController m_operator = new CommandXboxController(1); //Operator joystick

  //private final CommandXboxController m_dev = new CommandXboxController(2);

  //BUTTON BOX BABY!!!
  //private final CommandXboxController m_buttonLeft = new CommandXboxController(2);
  //private final CommandXboxController m_buttonRight = new CommandXboxController(3);

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  // Commands

  private final moveArmCommand m_moveArm = new moveArmCommand(armSubsystem, m_operator);
  private final IntakeShooterCommand m_intakeShooterCommand = new IntakeShooterCommand(intakeSubsystem,
      shooterSubsystem, ledSubsystem, m_operator);

  // AUTO COMMANDS
  private final Auto_shoot au_shoot = new Auto_shoot(shooterSubsystem, intakeSubsystem);
  private final Auto_shoot_safe au_shoot_safe = new Auto_shoot_safe(shooterSubsystem, intakeSubsystem); //heh heh, I hope auto_shoot gets used... NOT!!
  private final Auto_intake au_intake = new Auto_intake(intakeSubsystem);
  private final Auto_intake_safe au_intake_safe = new Auto_intake_safe(intakeSubsystem, ledSubsystem, shooterSubsystem, 0.15, 0.4);
  private final Auto_intake_safe au_intake_safe_slow = new Auto_intake_safe(intakeSubsystem, ledSubsystem, shooterSubsystem, 0.01, 0.1);
  private final Auto_intake_unsafe au_intake_unsafe = new Auto_intake_unsafe(intakeSubsystem, ledSubsystem, shooterSubsystem, 0.01, 0.1);



  // Auto Aim Shenanigans
  private final SwerveRequest.FieldCentricFacingAngle autoAim = new SwerveRequest.FieldCentricFacingAngle();
  private final PhoenixPIDController autoTurnPID = new PhoenixPIDController(3.2, 0, 0.2);

  public final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric() //Creates the freaking swerve dude!!!

      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  public void createFrontUsbCamera() {
    CameraServer.startAutomaticCapture(); //Camera stuff :3
  }

  /* Path follower */
  // private Command runAuto = drivetrain.getAutoPath("Tests");

  // private final Telemetry logger = new Telemetry(MaxSpeed);

  private void configureBindings() {

    autoAim.HeadingController = autoTurnPID;
    autoAim.HeadingController.enableContinuousInput(-180, 180);

    drivetrain.setDefaultCommand(
        // #region standard drivetrain
        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-m_driver.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-m_driver.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-m_driver.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ).ignoringDisable(true));

    m_driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
    m_driver.b().whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-m_driver.getLeftY(), -m_driver.getLeftX()))));

    //Resets gyro
    m_driver.button(7).onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    // drivetrain.registerTelemetry(logger::telemeterize);

    // The slow mode type things we implemented
    m_driver.pov(0).whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0.5).withVelocityY(0)));
    m_driver.pov(180).whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(-0.5).withVelocityY(0)));
    m_driver.pov(270).whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0).withVelocityY(0.5)));
    m_driver.pov(90).whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0).withVelocityY(-0.5)));

    /*m_driver.rightBumper().whileTrue( // Auto target for driver
        drivetrain.applyRequest(() -> autoAim.withTargetDirection(drivetrain.directionToGoal())
            .withVelocityX(-m_driver.getLeftY() * MaxSpeed)
            .withVelocityY(-m_driver.getLeftX() * MaxSpeed)

        )
    );*/

    // m_driver.rightBumper().whileTrue(drivetrain.run(() ->
    // drivetrain.drive_autoAim(0)));

    /* Bindings for drivetrain characterization */
    /*
     * These bindings require multiple buttons pushed to swap between quastatic and
     * dynamic
     */
    /*
     * Back/Start select dynamic/quasistatic, Y/X select forward/reverse direction
     */
    m_driver.back().and(m_driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    m_driver.back().and(m_driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    m_driver.start().and(m_driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    m_driver.start().and(m_driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    m_driver.rightBumper().whileTrue(new InstantCommand(drivetrain::slowRobot)); // slow/fast mode system (this doesnt work)
    m_driver.rightBumper().whileFalse(new InstantCommand(drivetrain::fastRobot));

    // SHOOTER // GOD I WISH I GOT A BUTTON BOX // GOD I REALLY WISH I HAD A BUTTON BOX
    new Trigger(m_operator.rightTrigger()).onTrue(m_intakeShooterCommand);
    new Trigger(m_operator.rightBumper()).onTrue(m_intakeShooterCommand);

    new Trigger(m_operator.rightTrigger()).and(m_operator.rightBumper()).onFalse(m_intakeShooterCommand);

    // INTAKE //
    new Trigger(m_operator.leftTrigger()).onTrue(m_intakeShooterCommand)  ;
    new Trigger(m_operator.leftBumper()).onTrue(m_intakeShooterCommand)   ;

    new Trigger(m_operator.leftTrigger()).or(m_operator.leftBumper()).onFalse(m_intakeShooterCommand);
 
    // Move Arm //
    new Trigger(m_operator.rightStick()).whileTrue(m_moveArm);
    new Trigger(m_operator.leftStick()).whileTrue(m_moveArm) ;
    new Trigger(m_operator.povUp()).onTrue(new InstantCommand(armSubsystem::posAmp));
    new Trigger(m_operator.povDown()).onTrue(new InstantCommand(armSubsystem::posExLong));
    new Trigger(m_operator.povRight()).onTrue(new InstantCommand(armSubsystem::posMid));
    new Trigger(m_operator.povLeft()).onTrue(new InstantCommand(armSubsystem::posLong));

    new Trigger(m_operator.b()).onTrue(new InstantCommand(armSubsystem::posZero ));
    new Trigger(m_operator.y()).onTrue(new InstantCommand(armSubsystem::posClimb));

    new Trigger(m_operator.x()).onTrue(m_intakeShooterCommand); //revs up shooter GOD I WISH I HAD A BUTTON BOX (hey I got a button box) (the button box is ok, no disrespect to Scott but I wish it was smaller ))
  //
    //new Trigger(m_operator.x()).onTrue(new InstantCommand(armSubsystem::calcAngle)); // Run this for auto aim
  //
    new Trigger(m_operator.a()).onTrue(new InstantCommand(armSubsystem::posCross));
  //
    new Trigger(m_operator.start()).onTrue(new InstantCommand(shooterSubsystem::auto_shooterOff)); //force that stuff off 
    new Trigger(m_operator.start()).onTrue(new InstantCommand(intakeSubsystem ::auto_intakeOff));
  //
    new Trigger(m_operator.button(7)).onTrue(new InstantCommand(armSubsystem::calcAngle)); //auto climb
    
    //new Trigger(m_dev.povRight()).onTrue(new InstantCommand(climberSubsystem::manual));



    //new Trigger(m_driver.a()).onTrue(new InstantCommand(climberSubsystem::prepClimb));
    //new Trigger(m_driver.b()).onTrue(new InstantCommand(climberSubsystem::lowerClimb));

    // new Trigger(m_operator.x()).onTrue(new
    // InstantCommand(limelightSubsystem::findDistanceToTarget));

    // new Trigger(m_operator.x()).onTrue(new
    // InstantCommand(climberSubsystem::raiseClimbers));
    // new Trigger(m_operator.a()).onTrue(new
    // InstantCommand(climberSubsystem::lowerClimbers));

    // manual 3rd controller

    // new Trigger(m_dev.a()).onTrue(m_climber);
    // new Trigger(m_dev.y()).whileTrue(m_climber);
    // new Trigger(m_dev.b()).whileTrue(new
    // InstantCommand(climberSubsystem::resetClimberZero));


    //button box stuff
/* *
    new Trigger(m_buttonLeft.button(5)).onTrue(new InstantCommand(armSubsystem::posZero));
    new Trigger(m_buttonLeft.button(4)).onTrue(new InstantCommand(armSubsystem::posMid));
    new Trigger(m_buttonLeft.button(3)).onTrue(new InstantCommand(armSubsystem::posAmp));
    new Trigger(m_buttonLeft.button(2)).onTrue(new InstantCommand(armSubsystem::posClimb));

    
    new Trigger(m_buttonLeft.button(9)).onTrue(new InstantCommand(armSubsystem::posLong));
    new Trigger(m_buttonLeft.button(7)).onTrue(new InstantCommand(armSubsystem::posExLong));
    new Trigger(m_buttonLeft.button(8)).onTrue(new InstantCommand(armSubsystem::posHPS));
    new Trigger(m_buttonLeft.button(6)).onTrue(new InstantCommand(armSubsystem::posCross));

  
    //manual stuff
    new Trigger(m_buttonRight.button(1)).onTrue(new InstantCommand(shooterSubsystem::auto_shooterOff));
    new Trigger(m_buttonRight.button(1)).onTrue(new InstantCommand(intakeSubsystem::auto_intakeOff));

    //new Trigger(m_buttonLeft.button(6)).onTrue(new InstantCommand(armSubsystem::pos));

    //Shooter/Intake

    new Trigger(m_buttonRight.button(7)).onTrue(m_intakeShooterCommand);
    new Trigger(m_buttonRight.button(8)).onTrue(m_intakeShooterCommand);
    new Trigger(m_buttonRight.button(9)).onTrue(m_intakeShooterCommand);
    new Trigger(m_buttonRight.button(10)).onTrue(m_intakeShooterCommand); */

  }


  //everyday for someone to look busy you gotta
  //hide in your code and do comment refactors


  public RobotContainer() {

    //// Auto Commands////
    // Auto Actions

    NamedCommands.registerCommand("act_shoot", au_shoot); // shoot that thang
    NamedCommands.registerCommand("act_shoot_s", au_shoot_safe);

    NamedCommands.registerCommand("act_intakeIR", au_intake); // does sick auto intake stuff w/ IR
    NamedCommands.registerCommand("act_intakeIR_s", au_intake_safe); //USE THIS ONE INSTEAD, SAFE IS SAFER!!!! (What idiot chose "augh we didnt need the safe one"... wait...)
    NamedCommands.registerCommand("act_intakeIR_s_slow", au_intake_safe_slow);
    NamedCommands.registerCommand("act_intakeIR_us", au_intake_unsafe);

    NamedCommands.registerCommand("act_intake_on", new InstantCommand(intakeSubsystem::auto_intakeOn));
    NamedCommands.registerCommand("act_intake_off", new InstantCommand(intakeSubsystem::auto_intakeOff));
    NamedCommands.registerCommand("act_shooter_on", new InstantCommand(shooterSubsystem::auto_shooterOn));
    NamedCommands.registerCommand("act_shooter_off", new InstantCommand(shooterSubsystem::auto_shooterOff));

    /*
     * Arm Positions
     * 
     */
    NamedCommands.registerCommand("pos_amp", new InstantCommand(armSubsystem::posAmp)); // set position of that thangs//
    NamedCommands.registerCommand("pos_exlong", new InstantCommand(armSubsystem::posExLong));
    NamedCommands.registerCommand("pos_mid", new InstantCommand(armSubsystem::posMid));
    NamedCommands.registerCommand("pos_long", new InstantCommand(armSubsystem::posLong));
    NamedCommands.registerCommand("pos_zero", new InstantCommand(armSubsystem::posZero));
    NamedCommands.registerCommand("pos_offside", new InstantCommand(armSubsystem::posOff));
    NamedCommands.registerCommand("pos_ctf", new InstantCommand(armSubsystem::posCross));

    //Weird effed up hard coded dumb stupid positions
    NamedCommands.registerCommand("pos_specFC3", new InstantCommand(armSubsystem::specFC3));
    NamedCommands.registerCommand("pos_note2", new InstantCommand(armSubsystem::posNote2));
    NamedCommands.registerCommand("pos_auto", new InstantCommand(armSubsystem::calcAngle));


    

    NamedCommands.registerCommand("reset_gyro", drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));
    //Driver stuff (remove if crashing)
    NamedCommands.registerCommand("act_xstance", drivetrain.applyRequest(() -> brake));
    NamedCommands.registerCommand("act_dr_aim", drivetrain.applyRequest(() -> autoAim.withTargetDirection(drivetrain.directionToGoal())
            .withVelocityX(-m_driver.getLeftY() * MaxSpeed)
            .withVelocityY(-m_driver.getLeftX() * MaxSpeed)));
    NamedCommands.registerCommand("act_op_aim", new InstantCommand(armSubsystem::calcAngle));

    // Work on auto targetting to add the following commands
    /*
     * aim_speaker = aiming at speaker
     * 
     */
    // NamedCommands.registerCommand("aim_speaker", new
    // InstantCommand(drivetrain::aimSpeaker));

    // turns out you have to put autochooser AFTER the named commands thingy
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    createFrontUsbCamera();
    configureBindings();
    //RobotController.setBrownoutVoltage(5);
  }

  public Command getAutonomousCommand() {
    /* First put the drivetrain into auto run mode, then run the auto */
    return autoChooser.getSelected();
    // return runAuto;
  }
}
