// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.Scalar;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.cscore.UsbCamera;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import org.opencv.core.Mat;
import org.opencv.core.Point;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve/maxSwerve"));
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  private final ArmSubsystem m_arm = new ArmSubsystem();
  private final PnumaticSubsystem  m_climb = new PnumaticSubsystem();
  private final VisionSubsystem m_vision = new VisionSubsystem();
   private final TargetSwerveSubsystem m_TargetSwerveSubsystem = new TargetSwerveSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final CommandPS5Controller driverXbox = new CommandPS5Controller(0);
   final CommandPS5Controller testPS5 = new CommandPS5Controller(1);
   private RobotContainer m_robotContainer;
   
     private Command TargetCoommand;

  CvSink cvSink;
  UsbCamera usbCamera;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
 
    // Configure the trigger bindings
    configureBindings();
    
   
    
          CvSource outputStream = CameraServer.putVideo("Rectangle", 100, 100);
    Command driveFieldOrientedAngularVelocity = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> driverXbox.getRightX()
    );
     Command TargetToSpeaker = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> m_TargetSwerveSubsystem.TargetToSpeaker()
    );

    Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> driverXbox.getRawAxis(2));

    drivebase.setDefaultCommand(
      //  !RobotBase.isSimulation() ? driveFieldOrientedAngularVelocity : driveFieldOrientedDirectAngleSim
      m_TargetSwerveSubsystem.TargetToSpeaker()==-1?driveFieldOrientedAngularVelocity:TargetToSpeaker);
        
  }

  private void configureBindings()
  {
      
    driverXbox.cross().onTrue((Commands.runOnce(drivebase::zeroGyro)));
    // driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());

    driverXbox.povUp().onTrue(m_arm.armUpperReset());
    driverXbox.povDown().onTrue(m_arm.armLowerReset());

    // driverXbox.button(10).whileTrue(m_intake.stopFeederCommand());

    
    // Üçgene basıldığında otomatik kol ayarlamayı aç
    driverXbox.triangle().toggleOnTrue(
      m_arm.setArmAngleCommand(() -> {
        m_arm.isPIDActive = true;
        SmartDashboard.putBoolean("Is auto arm on", true);
        return m_arm.calculateArmAngle(m_vision.getDistanceToSpeaker());
      })
    );

    // Modun kapandığını SmartDashboarda haber ver
    driverXbox.triangle().toggleOnFalse(
      new InstantCommand(() -> {
        m_arm.isPIDActive = false;
        SmartDashboard.putBoolean("Is auto arm on", false);
      })
    );
    
    driverXbox.circle().toggleOnTrue(
      new InstantCommand(() -> {
      m_intake.runIntakeOut();
      m_intake.runFeederOut();
        SmartDashboard.putBoolean("Is feeder out", false);
      })
    );
      
    driverXbox.circle().toggleOnFalse(
      new InstantCommand(() -> {
        m_intake.stopIntake();
        m_intake.stopFeeder();
        SmartDashboard.putBoolean("Is feeder out stop ", false);
      })
    );

      driverXbox.povLeft().onTrue(
      new InstantCommand(() -> {
         m_climb.ClimbOn();
        System.out.println("Is pnumutic open ");
      })
    );

     driverXbox.povRight().onTrue(
      new InstantCommand(() -> {
         m_climb.climof();
               System.out.println("Is pnumutic off ");
      
      })
      
    );
    
  driverXbox.circle().toggleOnTrue(
      new InstantCommand(() -> {
        m_TargetSwerveSubsystem.TargetAccept(true);
       
        SmartDashboard.putBoolean("Target Speaker ", true);
      })
    );
      driverXbox.circle().toggleOnFalse(
      new InstantCommand(() -> {
        m_TargetSwerveSubsystem.TargetAccept(false);
       
        SmartDashboard.putBoolean("Target Speaker ", false);
      })
    );
    
    

    driverXbox.L1().whileTrue(
      new StartEndCommand(() -> m_arm.setVoltage(4), () -> m_arm.setVoltage(ArmConstants.kArmStatic), m_arm)
    );
    driverXbox.L2().whileTrue(
      new StartEndCommand(() -> m_arm.setVoltage(-4), () -> m_arm.setVoltage(ArmConstants.kArmStatic), m_arm)
    );

    driverXbox.R1().whileTrue(new ShootCommand(m_intake, m_shooter));
    driverXbox.R2().whileTrue(new StartEndCommand(
      () -> { m_intake.runIntakeIn(); },
      () -> { m_intake.stopIntake(); }
    ));
  }

  public void updateSmartDashboard() {
    SmartDashboard.putNumber("Arm Current Angle", m_arm.getAngle());
    SmartDashboard.putNumber("Arm Angle Setpoint", m_arm.m_armAngleSetpoint);
    SmartDashboard.putNumber("Arm Written Voltage", m_arm.m_lastVoltage);

    SmartDashboard.putBoolean("is arm at upper limit", !m_arm.m_armUpperLimit.get());
    SmartDashboard.putBoolean("is arm at lower limit", !m_arm.m_armLowerLimit.get());
    SmartDashboard.putBoolean("has object", !m_intake.getSensorReading());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
   //return new InstantCommand(() -> drivebase.resetOdometry(new Pose2d(0,0, new Rotation2d(0)))); 
   return new SequentialCommandGroup(
      m_arm.armLowerReset(),
      m_shooter.runShooterOnceCommand(),
      m_intake.runFeederInOnceCommand(),
      m_intake.runIntakeInOnceCommand(),

      // drivebase.driveCommand(
      //   () -> 0.75,
      //   () -> 0,
      //   () -> 0
      // ).withTimeout(1),
      // new WaitCommand(0.75),
      new WaitCommand(1),

      // drivebase.driveCommand(
      //   () -> 1,
      //   () -> 0,
      //   () -> 0
      // ).withTimeout(1),
      // new WaitCommand(0.4),

      // drivebase.driveCommand(
      //   () -> 0,
      //   () -> 0,
      //   () -> 0
      // ).withTimeout(1),

      // new WaitCommand(2),
      m_shooter.stopShooterOnceCommand(),
      m_intake.stopIntakeOnceCommand(),
      m_intake.stopFeederOnceCommand(),
      new InstantCommand(() -> { System.out.println("tuna debug autonomous"); })
    );
  }

  public void setDriveMode()
  {
    //drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
