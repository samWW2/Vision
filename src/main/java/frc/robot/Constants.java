// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;


public final class Constants {
  public static final double ksVolts = 0.22;
  public static final double kvVoltSecondsPerMeter = 1.98;
  public static final double kaVoltSecondsSquaredPerMeter =  0.2;
  public static final double kTrackwidthMeters = 52;
  public static final DifferentialDriveKinematics kDriveKinematics =
  new DifferentialDriveKinematics(kTrackwidthMeters);
  public static final double kMaxSpeedMetersPerSecond = 3;
  public static final double kMaxAccelerationMetersPerSecondSquared = 1; 
  public static final double kRamseteB = 2;
  public static final double kPDriveVel = 8.5;
  public static final double kRamseteZeta = 0.7;




  
  



}
