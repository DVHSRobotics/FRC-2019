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
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
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
  private Spark scissorMotor1;
  private Gyro gyro;
  private Accelerometer accel;
  private PIDController pid; // Boilerplate for later
  private double kP, kI, kD;
  private AHRS ahrs;
  private UsbCamera cam0;
  private CameraServer camserv;
  private MjpegServer mjpegserv;
  private NetworkTableInstance tableinst;
  private DigitalInput scissorMotor1In;
  private DriverStation dsinst;
  private Servo servo1;
  private CANEncoder encoder;
  private CANSparkMax sparkmax;
  private PowerDistributionPanel pdp;

  //private Joy2 Joystick; //HIGBY joystick

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
    scissorMotor1 = new Spark(4);
    robotDrive = new MecanumDrive(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor);
    //gyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS0); // Gyro doesn't seem to work; returns 0 reading regardless of rotation
    accel = new BuiltInAccelerometer(); // Needs to be calibrated
    //ahrs = new AHRS(Port.kMXP);
    camserv = CameraServer.getInstance();
    tableinst = NetworkTableInstance.getDefault();
    scissorMotor1In = new DigitalInput(0);
    dsinst = DriverStation.getInstance();
    servo1 = new Servo(5);
    pdp = new PowerDistributionPanel();

    camserv.startAutomaticCapture();
    SmartDashboard.putNumber("PDP Voltage", pdp.getVoltage());
    SmartDashboard.putNumber("Scissor Motor PWM", scissorMotor1.getRaw());
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
    /*System.out.println("The Cake is What You Think it is For");
    System.out.println("To LIE");*/
    //rearRightMotor.setSpeed(.08);
  }

  /**
   * This function is called periodically during operator control.
   */
  
  //ROHAN'S CODE

  @Override
  public void teleopPeriodic() {
    controller.setZChannel(3); // 1: Right joystick up-and-down
    controller.setYChannel(2); // 0: Left joystick up-and-down
    controller.setXChannel(0); // 2: Left joystick side-to-side
    robotDrive.driveCartesian(controller.getY(), controller.getX(), controller.getZ());
    scissorMotor1.set(controller.getRawAxis(4) * 2);
    //System.out.println("ScissorLift = " + scissorMotor1In.get());
    servo1.set(controller.getRawAxis(5)); 
    // Default: 1 0 2
    //System.out.println("Xaccel = " + accel.getX());
    //System.out.println("Angle = " + ahrs.getYaw());
  }

  @Override
  public void testInit() {
    rearRightMotor.set(-.08);
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void pidWrite(double output) {
  }

  //HIGBY'S CODE

  /*@Override
  public void teleopPeriodic() {
  
    controller.setXChannel(0);
    controller.setYChannel(2);
    controller.setZChannel(3);
    robotDrive.driveCartesian(controller.getY(), controller.getX(), controller.getZ());

  }*/

}

/*

  WHAT THE CONTROLLER IS 

CH1   THR   SB UP
 +=   ELE   SB NUT
CH3   RUD   SB UP
 +=   AIL   SB NUT
CH4   AIL   SB UP
 +=   RUD   SB NUT
CH5   sci BLANK
CH6   ser BLANK
CH9   SA DOWN
CH10  SA UP
CH11  SB DOWN
CH12  SB UP
CH13  SC DOWN
CH14  SC NUT
CH15  SC UP
CH16  SD DOWN
CH17  SD NUT
CH18  SD UP
CH19  SF UP
CH20  SF DOWN
CH21  SH NONE

*/