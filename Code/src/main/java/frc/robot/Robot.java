/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.*;
import com.ctre.phoenix.*;

import edu.wpi.cscore.CameraServerJNI;
import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot implements PIDOutput {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private Joystick controller;
  private MecanumDrive robotDrive;
  private Spark frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor;
  private Gyro gyro;
  private Accelerometer accel;
  private PIDController pid; // Boilerplate for later
  private double kP, kI, kD;
  private AHRS ahrs;
  private UsbCamera cam0;
  private CameraServer camserv;
  private MjpegServer mjpegserv;
  private NetworkTable table;

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    m_period = .02;

    controller = new Joystick(0);
    rearLeftMotor = new Spark(0);
    rearRightMotor = new Spark(1);
    frontRightMotor = new Spark(2);
    frontLeftMotor = new Spark(3);
    robotDrive = new MecanumDrive(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor);
    gyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS0); // Gyro doesn't seem to work; returns 0 reading regardless of rotation
    accel = new BuiltInAccelerometer(); // Needs to be calibrated
    ahrs = new AHRS(Port.kMXP);
    camserv = CameraServer.getInstance();

    camserv.startAutomaticCapture();

  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // System.out.println("Y: " + controller.getYChannel() + " X: " + controller.getXChannel() + " Z: " + controller.getZChannel());
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  @Override
  public void teleopInit() {
    super.teleopInit();
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    controller.setZChannel(3); // 1: Right joystick up
    controller.setYChannel(2); // 0: Left joystick up
    controller.setXChannel(0); // 2: Left joystick side-to-side
    robotDrive.driveCartesian(controller.getY(), controller.getX(), controller.getZ());
    // Default: 1 0 2
    //System.out.println("Xaccel = " + accel.getX());
    System.out.println("Angle = " + ahrs.getYaw());
  }

  @Override
  public void testInit() {
    super.testInit();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  @Override
  public void disabledInit() {
    super.disabledInit();
  }

  @Override
  public void disabledPeriodic() {
    super.disabledPeriodic();
  }

  @Override
  public void pidWrite(double output) {
  }

}
