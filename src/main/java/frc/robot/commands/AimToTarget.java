// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.subsystems.DriveSubsystem;
// import frc.robot.subsystems.Limelight;

// public class AimToTarget extends CommandBase {
//   /** Creates a new AimToTarget. */
//   private Limelight limelight;
//   private DriveSubsystem drive;
//   private final float Kp = -0.1f;
//   private float min_command = 0.05f;
//   double right_command=0;
//   double left_command=0;
//   private double tx = limelight.getTx();
//   public AimToTarget(DriveSubsystem drive) {
//     limelight = new Limelight();
//     drive = new DriveSubsystem();
//     addRequirements(drive);
//   }

//   @Override
//   public void initialize() {}

//   @Override
//   public void execute() {
//     double heading_error = -tx;
//     double steering_adjust = 0.0;
    

//     if (Math.abs(heading_error) > 1.0) {
//         if (heading_error < 0) {
//             steering_adjust = Kp * heading_error + min_command;
//         } else {
//             steering_adjust = Kp * heading_error - min_command;
//         }
//     }
//     left_command += steering_adjust;
//     right_command -= steering_adjust;
//     drive.tankDrive(left_command, right_command);
//   }

//   @Override
//   public void end(boolean interrupted) {}

//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
