// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import frc.robot.commands.AimToTarget;
import frc.robot.subsystems.DriveSubsystem;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;


public class RobotContainer {
  private final Joystick arm_stick = new Joystick(0);
  private DriveSubsystem driveSubsystem;

 

  public RobotContainer() {
    driveSubsystem = new DriveSubsystem();
    configureBindings();
  }

  
  private void configureBindings() {
    // new JoystickButton(arm_stick, 5).whileTrue(new AimToTarget(driveSubsystem));    
  }


  public Command getAutonomousCommand() {
     // Create a voltage constraint to ensure we don't accelerate too fast
     var autoVoltageConstraint =
     new DifferentialDriveVoltageConstraint(
         new SimpleMotorFeedforward(
             Constants.ksVolts,
             Constants.kvVoltSecondsPerMeter,
             Constants.kaVoltSecondsSquaredPerMeter),
         Constants.kDriveKinematics,
         10);

 // Create config for trajectory
 TrajectoryConfig config =
     new TrajectoryConfig(
             Constants.kMaxSpeedMetersPerSecond,
             Constants.kMaxAccelerationMetersPerSecondSquared)
         // Add kinematics to ensure max speed is actually obeyed
         .setKinematics(Constants.kDriveKinematics)
         // Apply the voltage constraint
         .addConstraint(autoVoltageConstraint);

 // An example trajectory to follow.  All units in meters.
 Trajectory exampleTrajectory =
     TrajectoryGenerator.generateTrajectory(
         // Start at the origin facing the +X direction
         new Pose2d(0, 0, new Rotation2d(0)),
         // Pass through these two interior waypoints, making an 's' curve path
         List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
         // End 3 meters straight ahead of where we started, facing forward
         new Pose2d(3, 0, new Rotation2d(0)),
         // Pass config
         config);

 RamseteCommand ramseteCommand =
     new RamseteCommand(
         exampleTrajectory,
         driveSubsystem::getPose,
         new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
         new SimpleMotorFeedforward(
             Constants.ksVolts,
             Constants.kvVoltSecondsPerMeter,
             Constants.kaVoltSecondsSquaredPerMeter),
         Constants.kDriveKinematics,
         driveSubsystem::getWheelSpeeds, 
         new PIDController(Constants.kPDriveVel, 0, 0),
         new PIDController(Constants.kPDriveVel, 0, 0),
         // RamseteCommand passes volts to the callback
         driveSubsystem::tankDriveVolts,
         driveSubsystem);

 // Reset odometry to the starting pose of the trajectory.
 driveSubsystem.resetOdometry(exampleTrajectory.getInitialPose());

 // Run path following command, then stop at the end.
 return ramseteCommand.andThen(() -> driveSubsystem.tankDriveVolts(0, 0));
  }
}
