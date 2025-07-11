// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.SwerveSS;

import com.ctre.phoenix6.signals.NeutralModeValue;

import au.grapplerobotics.CanBridge;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static final CTREConfigs ctreConfigs = new CTREConfigs();

  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  private SwerveSS s_Swerve;  
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    CanBridge.runTCP();

    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    this.s_Swerve = RobotContainer.s_Swerve;
  }


  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();




    // if(s_Swerve.m_frontRightLL.getTV()){
    //   s_Swerve.m_poseEstimator.resetPosition(s_Swerve.getGyroYaw(), s_Swerve.getModulePositions(), LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-fr").pose);
    //   s_Swerve.swerveOdometry.resetPosition(s_Swerve.getGyroYaw(), s_Swerve.getModulePositions(), s_Swerve.m_poseEstimator.getEstimatedPosition());
    // }
    // else if(s_Swerve.m_backLeftLL.getTV()){
    //   s_Swerve.m_poseEstimator.resetPosition(s_Swerve.getGyroYaw(), s_Swerve.getModulePositions(), LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-fl").pose);
    //   s_Swerve.swerveOdometry.resetPosition(s_Swerve.getGyroYaw(), s_Swerve.getModulePositions(), s_Swerve.m_poseEstimator.getEstimatedPosition());
    // }
    // else{      
    // s_Swerve.m_poseEstimator.resetPosition(s_Swerve.getGyroYaw(), s_Swerve.getModulePositions(), new Pose2d());
    // s_Swerve.swerveOdometry.resetPosition(s_Swerve.getGyroYaw(), s_Swerve.getModulePositions(), new Pose2d());
    // }

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    s_Swerve.setNeutralMode(NeutralModeValue.Brake);



    // if(DriverStation.getAlliance().get() == DriverStation.Alliance.Red){
    //   new Rotation2d();
    //   s_Swerve.m_poseEstimator.resetPosition(s_Swerve.getGyroYaw().plus(Rotation2d.fromDegrees(180)), s_Swerve.getModulePositions(), LimelightHelpers.getBotPose2d_wpiBlue("limelight"));
    //   new Rotation2d();
    //   s_Swerve.swerveOdometry.resetPosition(s_Swerve.getGyroYaw().plus(Rotation2d.fromDegrees(180)), s_Swerve.getModulePositions(), s_Swerve.m_poseEstimator.getEstimatedPosition());
    //   SmartDashboard.putBoolean("Alliance", false);
    // }
    // else{


      // if(s_Swerve.m_frontRightLL.getTV()){
      //   s_Swerve.m_poseEstimator.resetPosition(s_Swerve.getGyroYaw(), s_Swerve.getModulePositions(), LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-fr").pose);
      //   s_Swerve.swerveOdometry.resetPosition(s_Swerve.getGyroYaw(), s_Swerve.getModulePositions(), s_Swerve.m_poseEstimator.getEstimatedPosition());
      // }
      // else if(s_Swerve.m_backLeftLL.getTV()){
      //   s_Swerve.m_poseEstimator.resetPosition(s_Swerve.getGyroYaw(), s_Swerve.getModulePositions(), LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-fl").pose);
      //   s_Swerve.swerveOdometry.resetPosition(s_Swerve.getGyroYaw(), s_Swerve.getModulePositions(), s_Swerve.m_poseEstimator.getEstimatedPosition());
      // }
      // else{      
      //   s_Swerve.m_poseEstimator.resetPosition(s_Swerve.getGyroYaw(), s_Swerve.getModulePositions(), new Pose2d());
      //   s_Swerve.swerveOdometry.resetPosition(s_Swerve.getGyroYaw(), s_Swerve.getModulePositions(), new Pose2d());
      // }
    
    
      // }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();

  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
