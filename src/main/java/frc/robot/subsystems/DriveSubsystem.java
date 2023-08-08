// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.SPI; // this import





//change the name of the class to "DriveSubsystem"
//pay attention that the class extends SubsystemBase
public class DriveSubsystem extends SubsystemBase {
  
  private final CANSparkMax leftMotor1 = new CANSparkMax(1, MotorType.kBrushless);
  private final CANSparkMax leftMotor2 = new CANSparkMax(2, MotorType.kBrushless);
  private final CANSparkMax rightMotor1 = new CANSparkMax(3, MotorType.kBrushless);
  private final CANSparkMax rightMotor2 = new CANSparkMax(4, MotorType.kBrushless);
  private final double kDegToMeters = 0.076 * Math.PI ;
  private final MotorControllerGroup m_left = new MotorControllerGroup(leftMotor1, leftMotor2);
  private final MotorControllerGroup m_right = new MotorControllerGroup(rightMotor1, rightMotor2);
  private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_left, m_right);
  private RelativeEncoder rightEncoder = rightMotor1.getEncoder();
  private RelativeEncoder leftEncoder = rightMotor1.getEncoder();
  private double factor = (((8.45 * 15.24)/10000))*2.9;
  private final AHRS gyro = new AHRS(SPI.Port.kMXP);
  private final double start_angel = 148;
  private DifferentialDriveOdometry m_odometry;




  //similar to the init call
  public DriveSubsystem() {
    final Pose2d startPos = new Pose2d();
    m_odometry = new DifferentialDriveOdometry(
      gyro.getRotation2d(),
      leftEncoder.getPosition(),
      rightEncoder.getPosition(),
      startPos);
    rightMotor1 .setInverted(true);
    rightMotor2.setInverted(true);
    rightEncoder.setPositionConversionFactor(factor);
    leftEncoder.setPositionConversionFactor(factor);
   
  }

  public void setIdleMode(IdleMode mode) {
    leftMotor1.setIdleMode(mode);
  }


  @Override
  public void periodic() {
    SmartDashboard.putNumber(" pitch " , getPitch());
    m_odometry.update(
        gyro.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition());


  
  }
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftEncoder.getPositionConversionFactor(), rightEncoder.getPositionConversionFactor()); //change this to get rate
  }
  public void setMotors(double leftSpeed, double rightSpeed) {
    m_left.set(leftSpeed);
    m_right.set(rightSpeed);
    
}
public void resetOdometry(Pose2d pose) {
  resetEncoders();
  m_odometry.resetPosition(
      gyro.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition(), pose);
}
public void tankDriveVolts(double leftVolts, double rightVolts) {
  m_left.setVoltage(leftVolts);
  m_right.setVoltage(rightVolts);
  m_robotDrive.feed();
}
public Pose2d getPose() {
  return m_odometry.getPoseMeters();
}

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
  public CANSparkMax getRightMotor1(){
    return this.rightMotor1;
  }
  public CANSparkMax getLeftMotor1(){
    return this.leftMotor1;
  }
  
  public void arcadeDrive(double str, double turn){
    m_robotDrive.arcadeDrive(str, turn);
  }
  public void tankDrive(double left, double right){
    m_robotDrive.tankDrive(left, right);
  }
  public void resetEncoders(){
    rightEncoder.setPosition(0);
    leftEncoder.setPosition(0);
  }
public boolean isPitchChanged(){
  if(Math.abs(getPitch()) > 15){    //change the number
     return true;
   }
   return false;
}
  public double getPitch(){
    double angle = gyro.getRoll() + start_angel;
    if(angle > 180){
      angle -= 360;
    }
    return angle;
  }
  public double getRightEncoder(){
    return rightEncoder.getPosition();
  }
  public double getleftEncoder(){
    return leftEncoder.getPosition();
  }
  public double getBothEncoders(){
    return (getRightEncoder() + getleftEncoder())/2;
  }

  
}
