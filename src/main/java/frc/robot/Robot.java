// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// CODE is based on this tutorial: https://www.youtube.com/watch?v=ihO-mw_4Qpo&ab_channel=FRC0toAutonomous

package frc.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.fasterxml.jackson.databind.ser.std.RawSerializer;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

  // variable to store the start time
  private double startTime; // IMPORTANT

   // Driving MOTORS
  private CANSparkMax leftFrontMotor = new CANSparkMax(3, CANSparkMaxLowLevel.MotorType.kBrushless);
  private CANSparkMax leftBackMotor = new CANSparkMax(4, CANSparkMaxLowLevel.MotorType.kBrushless);
  private CANSparkMax rightFrontMotor = new CANSparkMax(2, CANSparkMaxLowLevel.MotorType.kBrushless);
  private CANSparkMax rightBackMotor = new CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless);
  

  MotorControllerGroup leftControllerGroup = new MotorControllerGroup(leftFrontMotor, leftBackMotor);
  MotorControllerGroup rightControllerGroup = new MotorControllerGroup(rightFrontMotor, rightBackMotor);

  DifferentialDrive drive = new DifferentialDrive(leftControllerGroup, rightControllerGroup);

  private DifferentialDriveOdometry m_Odometry;
  // Intake Motors 
  private WPI_VictorSPX rollerMotor = new WPI_VictorSPX(5);
  private WPI_VictorSPX raisingMotor = new WPI_VictorSPX(6);

// Encoder Methods:
  // Encoder reset:
  public void resetEncoders(){
    rightEncoder.setPosition(0);
    leftEncoder.setPosition(0);
  }
// Wheel Speed Method 
public DifferentialDriveWheelSpeeds getWheelSpeeds(){
  return new DifferentialDriveWheelSpeeds(getLeftEncoderVelocity(), getRightEncoderVelocity());
}

// Average encoder distance Method
public double getAverageEncoderDistance(){
  return ((getLeftEncoderPosition() + getRightEncoderPosition()) / 2.0) ;
}

// Position of the right encoder after the conversion factor is applied Method --- had to negate it cuz it was outputing results in negative nums
public double getRightEncoderPosition(){
  return -rightEncoder.getPosition();
}


// Position of the left encoder after the conversion factor is applied Method
public double getLeftEncoderPosition(){
  return leftEncoder.getPosition();
}

// Velocity of the right encoder after the conversion factor is applied Method
public double getRightEncoderVelocity(){
  return (rightEncoder.getVelocity());
}

// Velocity of the left encoder after the conversion factor is applied  Method
public double getLeftEncoderVelocity(){
  return leftEncoder.getVelocity();
}

// Methods to return the Encoders:
public RelativeEncoder getLefEncoder(){
  return leftEncoder;
}

public RelativeEncoder getRightEncoder(){
  return rightEncoder;

}
// ______________________________________________________________________
//IMU Methods:
  // Odometry Reset
  public void resetOdometry(Pose2d poss){
    resetEncoders();
    m_Odometry.resetPosition(new Rotation2d(Units.degreesToRadians(getHeading())), getLeftEncoderPosition(), getRightEncoderPosition(), poss);
  }

  // Heading Reset
  public void zeroHeading(){
    m_IMU.calibrate();
    m_IMU.reset();
  }

  public ADIS16470_IMU getImu(){
    return getImu();
  }

 // Heading Output Method:
  public double getHeading() {
      return m_IMU.getAngle();
  }

  // Turning Rate Output Method:
  public double getTurnRate(){
    return m_IMU.getRate();
  }
  public Pose2d getPose(){
    return m_Odometry.getPoseMeters();
  }
// -------------------------------------------------------------------------------------------------------------------

  // IMU
  private ADIS16470_IMU m_IMU = new ADIS16470_IMU();


  // Controllers
  private XboxController driveController = new XboxController(0);
  private XboxController intakeController = new XboxController(1);

  // Unit Conversion
  private final double kDriveTick2Feet = 1.0/4096*6*Math.PI/12;
  private final double kRaisingTick2Feet = 360.0 / 512 * 26 / 42 * 18 / 60 * 18 / 84;


  // Encoders 
    RelativeEncoder leftEncoder = leftFrontMotor.getEncoder();
    RelativeEncoder rightEncoder = rightFrontMotor.getEncoder();


  @Override
  public void robotInit() {
    startTime = Timer.getFPGATimestamp();
  // inverted settings
    rightControllerGroup.setInverted(true);
    leftControllerGroup.setInverted(false);
    rollerMotor.setInverted(false);
    raisingMotor.setInverted(false);

    //init Encoders
    raisingMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);

    // slave setup
    rightBackMotor.follow(rightFrontMotor);
    leftBackMotor.follow(leftFrontMotor);


    // reset encoders to zero
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);

    // set encoder boundreis for intake
    // raisingMotor.configReverseSoftLimitThreshold(0/k)

    // deadBand
    drive.setDeadband(0.05);
  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Left encoder value in meters", getLeftEncoderPosition());
    SmartDashboard.putNumber("Right encoder value in meters", getRightEncoderPosition());
    SmartDashboard.putNumber("left Encoder Velocity", getLeftEncoderVelocity());
    SmartDashboard.putNumber("right Encoder velocity", getRightEncoderVelocity());
    SmartDashboard.putNumber("Average Encoder Distance", getAverageEncoderDistance());
    SmartDashboard.putNumber("Imu heading", getHeading());
    SmartDashboard.putNumber("Imu Turn Rate", getTurnRate());

  }

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {
    double time = Timer.getFPGATimestamp(); // gets the time in seconds
    if(time - startTime < 3){
      leftFrontMotor.set(0.6);
      leftBackMotor.set(0.6);
      rightFrontMotor.set(-0.6);
      rightBackMotor.set(-0.6);
    }else{
      leftFrontMotor.set(0);
      leftBackMotor.set(0);
      rightFrontMotor.set(0);
      rightBackMotor.set(0);
    }
  }

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    double power = -driveController.getRawAxis(1) * 0.8;  // for this axis: up is negative, down is positive
    double turn = driveController.getRawAxis(4) * 0.3;
     // slow speed down to 60% and turning speed to 30% for better controllability
    // // deadband
    // if(Math.abs(power) < 0.05){
    //   power = 0;
    // }
    // if(Math.abs(turn) < 0.05){
    //   turn = 0;
    // }
    drive.arcadeDrive(power, turn);

    // intake control
    // double intakeRas
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
