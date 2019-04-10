/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

/*
  PRIMARY REFERENCES:
  FULL FRC ROBOT CONTROL SYSTEM DOCUMENTATION: https://wpilib.screenstepslive.com/s/4485
  FULL JAVA API DOCUMENTATION: https://wpilib.screenstepslive.com/s/currentCS/m/java/l/272787-frc-java-wpilib-api-documentation
  NEXGEN ROBOTICS GITHUB: https://github.com/DVHSRobotics
  WPILIB SPREENSTEPS WEBSITE: https://wpilib.screenstepslive.com/s
  PID: https://wpilib.screenstepslive.com/s/3120/m/7912/l/79828-operating-the-robot-with-feedback-from-sensors-pid-control
  
  PARTS:
  OFFICIAL FRC ROBOT PARTS OVERVIEW: http://wpilib.screenstepslive.com/s/currentCS/m/getting_started/l/599672-frc-control-system-hardware-overview
  REV NEO MOTORS: http://www.revrobotics.com/rev-21-1650/
  SPARK MAX MOTOR CONTROLLERS: http://www.revrobotics.com/rev-11-2158/
  POWER DISTRIBUTION PANEL (PDP): http://www.ctr-electronics.com/pdp.html#product_tabs_technical_resources

  Hover over method or class names or right click on them to get more information about them.
  Ask Mr. Palmer or Andrew if confused about anything. Or just do a Google search.
*/

package frc.robot;

// Third party imports (didn't come with WPIlib)
import com.kauailabs.navx.frc.AHRS; // Source: https://pdocs.kauailabs.com/navx-mxp/software/roborio-libraries/java/
import com.revrobotics.*;
import com.ctre.phoenix.*;

// Default imports
import edu.wpi.cscore.CameraServerJNI;
import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.hal.PWMConfigDataResult;
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
import edu.wpi.first.wpilibj.Sendable;
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

  // DECLARE CLASS VARIABLES
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private Joystick controller, controller2;
  private MecanumDrive robotDrive;
  private Spark frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor;
  private Spark scissorLiftMotor, hatchGrabMotor;
  private Servo rampServos, scissorLiftSpringServo, scissorLiftBlockServo, grabberServo;
  private Gyro gyro;
  private Accelerometer accel;
  private PIDController pid; // Currently not needed
  private double kP, kI, kD;
  private AHRS ahrs;
  private UsbCamera cam0;
  private CameraServer camserv;
  private MjpegServer mjpegserv;
  private NetworkTableInstance tableinst;
  private DigitalInput hatchGrabMotorIn;
  private DriverStation dsinst;
  private CANEncoder encoder;
  private CANSparkMax sparkmax;
  private PowerDistributionPanel pdp;
  private PWMConfigDataResult rslt;
  private int hatchMotorCount;
  private boolean lastHatchMotorIn;

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

    // INITIALIZE VARIABLES DECLARED EARLIER

    // Controllers
    controller = new Joystick(0);
    controller2 = new Joystick(1);

    // Wheel Motors
    rearLeftMotor = new Spark(0);
    rearRightMotor = new Spark(1);
    frontRightMotor = new Spark(2);
    frontLeftMotor = new Spark(3);

    // Other motors & servos
    scissorLiftMotor = new Spark(4);
    hatchGrabMotor = new Spark(5);
    rampServos = new Servo(6);
    scissorLiftSpringServo = new Servo(7);
    scissorLiftBlockServo = new Servo(8);

    // Input for Hall Effect Sensor to DIO attachment for Bosch Seat Motor
    // NEVER WORKED - CONTACT BOSCH IF INTERESTED
    hatchGrabMotorIn = new DigitalInput(0);

    // Mecanum Drive
    robotDrive = new MecanumDrive(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor);

    // Sensors - unneeded
    //gyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS0); // FRC Gyro doesn't seem to work; returns 0 reading regardless of rotation
    //accel = new BuiltInAccelerometer(); // Accelerometer built into rio - Needs to be calibrated
    //ahrs = new AHRS(Port.kMXP); // NavX (plugs into large center port of Rio)

    // Camera variables
    camserv = CameraServer.getInstance();
    tableinst = NetworkTableInstance.getDefault();
    dsinst = DriverStation.getInstance();
    pdp = new PowerDistributionPanel();
    rslt = scissorLiftMotor.getRawBounds();

    // Initilize camera feeds once
    camserv.startAutomaticCapture();
  }

  /**
   * This function is called repeatedly (every robot packet), no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable chooser
   * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
   * remove all of the chooser code and uncomment the getString line to get the
   * auto name from the text box below the Gyro //ROHAN IS STUPID
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure below with additional strings. If using the SendableChooser
   * make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /**
   * This function is called periodically (repeatedly) during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    /*switch (m_autoSelected) {
    case kCustomAuto:
      // Put custom auto code here
      break;
    case kDefaultAuto:
    default:
      // Put default auto code here
      break;
    }*/

    // Manually drive robot during autonomous mode
    controller.setZChannel(3); // 1: Right joystick up-and-down
    controller.setYChannel(2); // 0: Left joystick up-and-down
    controller.setXChannel(0); // 2: Left joystick side-to-side
    robotDrive.driveCartesian(controller.getY(), controller.getX(), controller.getZ());
  }

  @Override
  public void teleopInit() {
  }

  /**
   * This function is called periodically (repeatedly) during teleop mode.
   */
  @Override
  public void teleopPeriodic() {
    // Manual Driving
    controller.setZChannel(3); // 1: Right joystick up-and-down
    controller.setYChannel(2); // 0: Left joystick up-and-down
    controller.setXChannel(0); // 2: Left joystick side-to-side
    robotDrive.driveCartesian(controller.getY(), controller.getX(), controller.getZ());
    scissorLiftMotor.setSpeed(-.2);

    // Ramp Deploy
    if(controller.getRawButton(11))
      rampServos.set(.5);
    else
      rampServos.set(0.0);
    
    /*if (hatchGrabMotorIn.get() != lastHatchMotorIn && hatchMotorCount < 44) {
      hatchMotorCount++;
      lastHatchMotorIn = hatchGrabMotorIn.get();
      hatchGrabMotor.set(controller2.getRawButtonPressed(1) ? .5 : controller2.getRawButtonPressed(2) ? -.5 : 0.0);
    } else if (hatchMotorCount == 44) {
      hatchMotorCount = 0;
      lastHatchMotorIn = false;
      hatchGrabMotor.set(0.0);
    } else
      hatchGrabMotor.set(0.0);*/

    /*if(controller2.getRawButton(1))
      scissorLiftMotor.setPosition(.2);
    else if (controller2.getRawButton(2))
      scissorLiftMotor.setPosition(.8);*/
  }

  @Override
  public void testInit() {
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

  // Necessary for PID
  @Override
  public void pidWrite(double output) {
  }

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