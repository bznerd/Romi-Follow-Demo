// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.sensors.RomiGyro;
import frc.robot.Constants.Drivetrain;
import frc.robot.Constants.AutoConstants;
import com.pathplanner.lib.commands.PPRamseteCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.controller.RamseteController;

public class RomiDrivetrain extends SubsystemBase {
  // The Romi has the left and right motors set to
  // PWM channels 0 and 1 respectively
  private final Spark leftMotor = new Spark(0);
  private final Spark rightMotor = new Spark(1);

  // The Romi has onboard encoders that are hardcoded
  // to use DIO pins 4/5 and 6/7 for the left and right
  private final Encoder leftEncoder = new Encoder(4, 5);
  private final Encoder rightEncoder = new Encoder(6, 7);

  private final RomiGyro gyro = new RomiGyro();

  // Set up the differential drive controller
  private final DifferentialDrive diffDrive = new DifferentialDrive(leftMotor, rightMotor);

  private final Field2d field = new Field2d();

  private final PIDController leftPID = new PIDController(Drivetrain.kP, 0, 0);
  private final PIDController rightPID = new PIDController(Drivetrain.kP, 0, 0);
  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Drivetrain.kS, Drivetrain.kV, Drivetrain.kA);
  private final RamseteController controller = new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta);

  private final DifferentialDrivePoseEstimator poseEstimator = new DifferentialDrivePoseEstimator(Drivetrain.kDriveKinematics, getGyro(), getLeftDistance(), getRightDistance(), new Pose2d());

  /** Creates a new RomiDrivetrain. */
  public RomiDrivetrain() {
    // Use inches as unit for encoder distances
    leftEncoder.setDistancePerPulse((Math.PI * Drivetrain.kWheelDiameterMeters) / Drivetrain.kCountsPerRevolution);
    rightEncoder.setDistancePerPulse((Math.PI * Drivetrain.kWheelDiameterMeters) / Drivetrain.kCountsPerRevolution);
    resetEncoders();

    // Invert right side since motor is flipped
    rightMotor.setInverted(true);
    controller.setTolerance(new Pose2d());
  }

  public void arcadeDrive(double xaxisSpeed, double zaxisRotate) {
    diffDrive.arcadeDrive(xaxisSpeed, zaxisRotate);
  }

  public void driveVoltages(double left, double right) {
    leftMotor.setVoltage(left);
    rightMotor.setVoltage(right);
  }

  public void resetEncoders() {
    leftEncoder.reset();
    rightEncoder.reset();
  }

  public double getLeftDistance() {
    return leftEncoder.getDistance();
  }

  public double getRightDistance() {
    return rightEncoder.getDistance();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate());
  }

  public void resetGyro() {
    gyro.reset();
  }

  public Rotation2d getGyro() {
    return Rotation2d.fromDegrees(-gyro.getAngleZ());
  }

  public void resetPose() {
    resetPose(new Pose2d());
  }

  public void resetPose(Pose2d pose) {
    resetGyro();
    poseEstimator.resetPosition(getGyro(), getLeftDistance(), getRightDistance(), pose);
  }

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public void driveFF(ChassisSpeeds chassis) {
    var wheels = Drivetrain.kDriveKinematics.toWheelSpeeds(chassis);
    leftMotor.setVoltage(feedforward.calculate(wheels.leftMetersPerSecond));
    rightMotor.setVoltage(feedforward.calculate(wheels.rightMetersPerSecond));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    poseEstimator.update(getGyro(), getLeftDistance(), getRightDistance());
    diffDrive.feed();
    field.setRobotPose(getPose());
    SmartDashboard.putData("Pose", field);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
    return this.runOnce(() -> {
      // Reset odometry for the first path you run during auto
        if(isFirstPath) this.resetPose(traj.getInitialPose());
      }).andThen(
        new PPRamseteCommand(
            traj, 
            this::getPose, // Pose supplier
            controller,
            feedforward,
            Drivetrain.kDriveKinematics, // DifferentialDriveKinematics
            this::getWheelSpeeds, // DifferentialDriveWheelSpeeds supplier
            leftPID, // Left controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
            rightPID, // Right controller (usually the same values as left controller)
            this::driveVoltages,
            this
        )).andThen(new PrintCommand("Finished"));
  }
}
