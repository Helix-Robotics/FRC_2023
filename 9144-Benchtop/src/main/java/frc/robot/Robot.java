// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
//import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.IdleMode;

import javax.net.ssl.TrustManager;

import com.kauailabs.navx.frc.AHRS;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.filter.SlewRateLimiter;



/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {
  //drive drain
  static final int chang = 50;
  private final CANSparkMax m_frontLeft = new CANSparkMax(1, MotorType.kBrushed);
  private final CANSparkMax m_rearLeft = new CANSparkMax(2, MotorType.kBrushed);
  private final MotorControllerGroup m_leftDrive = new MotorControllerGroup(m_frontLeft, m_rearLeft);

  private final CANSparkMax m_frontRight = new CANSparkMax(3, MotorType.kBrushed);
  private final CANSparkMax m_rearRight = new CANSparkMax(4, MotorType.kBrushed);
  private final MotorControllerGroup m_rightDrive = new MotorControllerGroup(m_frontRight, m_rearRight);

  private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftDrive, m_rightDrive);
  
  //arm motor 
  private final CANSparkMax m_arm = new CANSparkMax(5, MotorType.kBrushless);
  
  //Intake motor
  private final CANSparkMax m_intake = new CANSparkMax(6, MotorType.kBrushed);

  //ground intake motors
  private final CANSparkMax m_gintake_left = new CANSparkMax(8, MotorType.kBrushed);
  private final CANSparkMax m_gintake_right = new CANSparkMax(7, MotorType.kBrushed);

  //controllers
  private final XboxController m_controller = new XboxController(0);
  private final Joystick m_stick = new Joystick(1);
  //private final XboxController m_buttonBox = new XboxController(2);


  private final int AUTO_MODE = 2; // 1 - mobility without bump, 2- mobility with bump, 3 - charging station

  private final Timer m_timer = new Timer();

  private final AHRS gyro = new AHRS(SPI.Port.kMXP);

  private static final double kOffBalanceAngleThresholdDegrees = 10;
  private static final double kOonBalanceAngleThresholdDegrees = 5;

  private double turningAdjust = 0;//-0.06;

  private double kP = 0.05;
  private boolean autoBalanceXMode;
  private boolean autoBalanceYMode;

  private boolean autoBalanceChargingStation;
  private double maxSpeed = 0.9;


  //intake part
  static final int INTAKE_CURRENT_LIMIT_A = 25;
  static final int INTAKE_HOLD_CURRENT_LIMIT_A = 10;
  static final double INTAKE_OUTPUT_POWER = 1.0;
  static final double INTAKE_HOLD_POWER = 0.3;
  static final double AUTO_THROW_TIME_S = 0.375;
  static final double AUTO_DRIVE_TIME = 7.0;
  static final double AUTO_DRIVE_SPEED = -0.25;

  static final int GINTAKE_CURRENT_LIMIT_A = 40;
  static final double GINTAKE_OUTPUT_POWER = 1.0;

  //arm part
  static final int ARM_CURRENT_LIMIT_A = 20;
  static final double ARM_OUTPUT_POWER = 0.4;
  static final double ARM_EXTEND_TIME_S = 2.0;

  static final double BACKWARDS_AUTO = 0.8;

  static final int NONE = 0;
  static final int CUBE = 1;
  static final int CONE = 2;
  static final int GROUND = 3;

  private double lastGamePiece = NONE;
  private double autonomousIntakePower;

  //pid control for arm motor //Testing only
  private SparkMaxPIDController m_pidController;
  private RelativeEncoder m_encoder;
  public double m_kP, m_kI, m_kD, m_kIz, m_kFF, m_kMaxOutput, m_kMinOutput;

  //Player assitant variables
  private int assist_started = 0; //1 started, 0 - stopped/cancelled
  private int assist_button_mode = 0; // 0 - unset, 1 - pickup, 2 - dropoff
  private int assist_obj_type = 0; // 0 - unknown, 1 - cone, 2 - cube
  private int assist_drop_row = 0; // 0 - unknown, 1 - bottom, 2 - mid, 3 - top
  private int assist_step = 0; // 0 - haven't started yet, >=1 - step #
  private double assist_action_start_time = 0.0; // 0 - not start, otherwise the started time.

  static final int ASSIST_STARTED = 1;
  static final int ASSIST_FINISHED = 0;

  static final int ASSIST_MODE_UNKNOWN = 0;
  static final int ASSIST_MODE_PICKUP = 1;
  static final int ASSIST_MODE_DROPOFF = 2;

  static final int ASSIST_OBJ_TYPE_UNKNOWN = 0;
  static final int ASSIST_OBJ_TYPE_CONE = 1;
  static final int ASSIST_OBJ_TYPE_CUBE = 2;

  static final int ASSIST_DROP_ROW_UNKNOWN = 0;
  static final int ASSIST_DROP_ROW_BOTTOM = 1;
  static final int ASSIST_DROP_ROW_MID = 2;
  static final int ASSIST_DROP_ROW_HIGH = 3;

  //cameras
  private NetworkTable ntTopCamera;
  private NetworkTable ntIntakeCamera;
  private float ntTy;

  static final int ASSIST_CAMERA_PIPELINE_PICKUP_CONE = 0;
  static final int ASSIST_CAMERA_PIPELINE_PICKUP_CUBE = 1;

  //balance parameters 
  private double prev_speed = 0.0;
  private double maxSpeedForBalance = 0.63;

  private SlewRateLimiter driveFilter = new SlewRateLimiter(0.7); 
  /**
   * Auto driving forward
   * 
   * @param seconds
   */
  private void autoDrivingFwdForSeconds(double seconds){
    /**
     * Configuration
    */
    double speedAutoDrivingFwd = 0.8;
    if (m_timer.get() < seconds){
      m_robotDrive.arcadeDrive(speedAutoDrivingFwd, 0.0+turningAdjust, false);
    }
  }

  /**
   * Auto balancing for charging station from the time
   * @param from
   */
  private void autoBalancingFromSeconds(double from){
    if (m_timer.get() > from){
      autoBalanceOnCharging();
    }
  }

  /**
   * Easiest algorithm for driving
   * Just a linear
   */
  private void linearDrive() {
    m_robotDrive.arcadeDrive(m_controller.getLeftY()*0.5, m_controller.getRightX()*0.9+turningAdjust);
  }

  /**
   * the calculation will make speed easier to controler at lower speed
   * but faster change at high speed
   * at highend, it will close 3 times change as lower end
   * make smoothK smaller if you want smooth operation at lower speed
   * specially design for charging station tasks and general driving
   */
  private double cubdedAlgorithm(double value, double smoothK){
    return (Math.pow(value, 3)+smoothK*value)/(1.0+smoothK);
  }

  //this is for speed
  private double getCubedSpeed(double y){
    double smoothK = 0.2;
    return cubdedAlgorithm(y, smoothK);
  }

  //this for turning 
  //we also want to fine control in turning at low end
  private double getCubedTurning(double x){
    double smoothK = 1;
    return cubdedAlgorithm(x, smoothK);
  }

  /**
   * 
   */
  private void cubedDrive() {
    //* times make cap speed */
    /**
     * Configuration
     */
    if (m_controller.getLeftTriggerAxis() > 0.1){
      maxSpeed = 0.6;
      turningAdjust = 0;
    }
    if (m_controller.getRightTriggerAxis() > 0.1){
      maxSpeed = 0.9;
      turningAdjust = 0;

    }
    /**
     * Calculation
     */
    
    double speed = getCubedSpeed(m_controller.getLeftY())*maxSpeed;
    double turning = getCubedTurning(m_controller.getRightX())*maxSpeed;

    /**
     * Debug
     */
    //display manual speed
    //manual speed will help other parts
    SmartDashboard.putNumber("TO_SPEED", speed);
    SmartDashboard.putNumber("XBOX_Y", m_controller.getLeftY());
    SmartDashboard.putNumber("TO_TURNING", turning);

    /**
     * Drive
     */
    //driving
    //add filter
    //speed = driveFilter.calculate(speed);
    m_robotDrive.arcadeDrive(speed, turning+turningAdjust);
  }

  /**
   * xbox controller left bumper will disable auto balance in teleop mode
   * sticker's button 2(the silwer button around the stick left hand side) to enable teleoop mode
   */
  private void updateAutoBalanceForChargingStation(){
  
    //we enable autocharging in teleop mode
    if(m_controller.getLeftBumper()){
      autoBalanceChargingStation = true;
      setDriveDrainMotorBrake();
    }

    //we disable autocharging in teleop mode
    if(m_controller.getRightBumper()){
      autoBalanceChargingStation = false;
    }
    

    if(m_controller.getYButton()){
      //autoBalanceChargingStation = false;
      setDriveDrainMotorCoast();
    }

    if(m_controller.getXButton()){
      setDriveDrainMotorBrake();
    }
  }

  /** 
   * have steep at low value, but wont have lot change at high end 
   * Similar to sine algorithm
   * but have slighly bigger change at lower end
   */
  private double cubedRootAlgorithm(double value, double smoothK){
    return (Math.pow(value, 1/3)+smoothK*value)/(1.0+smoothK);
  }

  /**
   *  use sin algorithm for algorithm 
   * this will similar effect as cubedRoot algorithm
   * but it will much smoother lower end
   */
  private double sineAlgorithm(double value, double smoothK){
    return (Math.sin(value*Math.PI/2)+smoothK*value)/(1.0+smoothK);
  }

  private void autoBalanceOnCharging(){
  
    //1)we need to convert Tilt to movement //forward or backward
    //2)we gona to assume robot will be in right direction when it come in right angle 
    //2.a) We may need still enable turning from one of controllers if required
    //3)assume that rebalance required if other robots going on
    //4) robot may entry to charge station in either direction, assume it will start from our side
    //5) Calculation of speed change is important

    /**
     * Configurations
     */
    // setArmMotor(-0.1);
    //double maxSpeedForBalance = 0.63; //replaced with class variable, max speed for balance , gradually increase this
    double minSpeed = 0.345; //0.35 minimum speed required for robot to move
    double maxAngle = 20; //although the document max degree is 35, 
                          //but in practice, 15 will be maximum, 
                          //this will impact how flat top will be 
                          //we want climbing in constant speed. 
                          //decreae this if you see too much speed change in climbing
    double startToBalanceAngle = 2.01; //1.99; //modify this to achiec

    /**
     * Read from devices
     */
    double pitch = gyro.getPitch(); //we only use pitch to balance for now
    double turning = m_stick.getX();

    /**
     * set four motors to brake mode
     */
    
    /**
     * Speed Calculation Part
     */
    //calculation starts
    double speed = pitch/maxAngle;

    if (prev_speed * speed < 0){
     //maxSpeedForBalance = maxSpeedForBalance * 0.98;
     //dont change max speed for now
    }

    //Prevent overloading, we will have only have max = 1
    if(speed>1){
      speed = 1; //safty 
    }
    if (speed < -1){
      speed = -1;
    }

    //cubed root algorithm
    //speed = cubedRootAlgorithm(speed, 1.5)*maxSpeedForBalance;
    //use sin algorithm 
    //speed = sineAlgorithm(speed, 0.1)*maxSpeedForBalance;
    speed = sineAlgorithm(speed, 0.5)*maxSpeedForBalance;

    //when less than minspeed, the robot wont move
    if(Math.abs(speed)<minSpeed){
      speed=minSpeed*speed/Math.abs(speed);
    }

    prev_speed = speed;

        
    /**
     * Turning Calculation
     */
    //calculate turning
    turning = getCubedTurning(turning);

    if (Math.abs(pitch) < startToBalanceAngle ){
      speed = 0;
      turning = 0;
    }
    /**
     * Display on smartdashboard for debugging purpose
     */
    SmartDashboard.putNumber("AC_SPEED", speed);
    SmartDashboard.putNumber("AC_TURN", turning);


    /**
     * Start Driving
     * When under 2 degree, we will try to stoop
     */
    m_robotDrive.arcadeDrive(speed, turning);
    /*
    if (Math.abs(pitch) > startToBalanceAngle){
      m_robotDrive.arcadeDrive(speed, turning); //+turningAdjust
    } else {
      setDriveMotors(0, 0);

      //m_robotDrive.arcadeDrive(0, turningAdjust);
    }
     */

  }

  //intake here
  public void setIntakeMotor(double percent, int amps) {
    m_intake.set(percent);
    m_intake.setSmartCurrentLimit(amps);
    SmartDashboard.putNumber("intake power (%)", percent);
    SmartDashboard.putNumber("intake motor current (amps)", m_intake.getOutputCurrent());
    SmartDashboard.putNumber("intake motor temperature (C)", m_intake.getMotorTemperature());
  }

  public void setGroundIntakeMotors(double percent, int amps){
    m_gintake_left.set(percent);
    m_gintake_right.set(-percent);
    m_gintake_left.setSmartCurrentLimit(amps);
    m_gintake_right.setSmartCurrentLimit(amps);
    SmartDashboard.putNumber("ground intake power (%)", percent);
    SmartDashboard.putNumber("ground left intake motor current (amps)", m_gintake_left.getOutputCurrent());
    SmartDashboard.putNumber("gounrd left intake motor temperature (C)", m_gintake_left.getMotorTemperature());   
    SmartDashboard.putNumber("ground right intake motor current (amps)", m_gintake_right.getOutputCurrent());
    SmartDashboard.putNumber("gounrd right intake motor temperature (C)", m_gintake_right.getMotorTemperature());   
  }

  //arm set here
  public void setArmMotor(double percent) {
    m_arm.set(percent);
    SmartDashboard.putNumber("arm power (%)", percent);    
    SmartDashboard.putNumber("arm motor current (amps)", m_arm.getOutputCurrent());    
    SmartDashboard.putNumber("arm motor temperature (C)", m_arm.getMotorTemperature());    
  }

  //set drive motors
  public void setDriveMotors(double forward, double turn) {
    SmartDashboard.putNumber("drive forward power (%)", forward);    
    SmartDashboard.putNumber("drive turn power (%)", turn);    

    double left = forward - turn;
    double right = forward + turn;

    m_robotDrive.arcadeDrive(forward, turn+0.0+turningAdjust, false);

    SmartDashboard.putNumber("drive left power (%)", left);    
    SmartDashboard.putNumber("drive right power (%)", right);        
  }

  public void moveArm() {
    double m_stick_y = m_stick.getY();
    SmartDashboard.putNumber("Stick Y Value:", m_stick_y);
    if (m_stick_y > 0.2){
      //move arm backward
      setArmMotor(m_stick_y*-0.4);
    } else if (m_stick_y < -0.15){
      //move arm forward
      setArmMotor(m_stick_y*-0.55);
      //moveArmPidDegrees(-20);
    } else {
      //stop arm motor
      setArmMotor(0);
      m_arm.stopMotor();
    }

  }


  private void intakeNow(){
    //Intake here
    double intakePower;
    int intakeAmps;
    if (m_stick.getRawButton(1)) {
      // cone in or cube out
      intakePower = INTAKE_OUTPUT_POWER;
      intakeAmps = INTAKE_CURRENT_LIMIT_A;
      lastGamePiece = CONE;
    } else if (m_stick.getRawButton(2)) {
      // cube in or cone out
      intakePower = -INTAKE_OUTPUT_POWER;
      intakeAmps = INTAKE_CURRENT_LIMIT_A;
      lastGamePiece = CUBE;
    } else if (m_stick.getRawButton(3) || m_stick.getRawButton(5)){
      lastGamePiece = NONE;
      intakePower = 0.0;
      intakeAmps = 0;      
    }else if (lastGamePiece == CUBE) {
      intakePower = -INTAKE_HOLD_POWER;
      intakeAmps = INTAKE_HOLD_CURRENT_LIMIT_A;
    } else if (lastGamePiece == CONE) {
      intakePower = INTAKE_HOLD_POWER;
      intakeAmps = INTAKE_HOLD_CURRENT_LIMIT_A;
    } else {
      intakePower = 0.0;
      intakeAmps = 0;
    }
    
    setIntakeMotor(intakePower, intakeAmps);
  }

  private void groundIntakeNow() {

    double intakePower;
    int intakeAmps;

    if (m_stick.getRawButton(6)){
      //ground intake in 
      intakePower = GINTAKE_OUTPUT_POWER;
      intakeAmps = GINTAKE_CURRENT_LIMIT_A;
      lastGamePiece = GROUND;
    }else if (m_stick.getRawButton(4)){
      //ground intake out
      intakePower = -GINTAKE_OUTPUT_POWER;
      intakeAmps = GINTAKE_CURRENT_LIMIT_A;  
      lastGamePiece = NONE;    
    } else if (m_stick.getRawButton(3) || m_stick.getRawButton(5)){
      lastGamePiece = NONE;
      intakePower = 0.0;
      intakeAmps = 0;
    }else{
      intakePower = 0.0;
      intakeAmps = 0;
    }

    setGroundIntakeMotors(intakePower, intakeAmps);
  }

  public void initizeArmMotorPid() {
    /**
     * In order to use PID functionality for a controller, a SparkMaxPIDController object
     * is constructed by calling the getPIDController() method on an existing
     * CANSparkMax object
     */
    m_pidController = m_arm.getPIDController();

    // Encoder object created to display position values
    m_encoder = m_arm.getEncoder();

     // PID coefficients
     m_kP = 0.1; 
     m_kI = 1e-4;
     m_kD = 1; 
     m_kIz = 0; 
     m_kFF = 0; 
     m_kMaxOutput = 1; 
     m_kMinOutput = -1;
 
     // set PID coefficients
     m_pidController.setP(m_kP);
     m_pidController.setI(m_kI);
     m_pidController.setD(m_kD);
     m_pidController.setIZone(m_kIz);
     m_pidController.setFF(m_kFF);
     m_pidController.setOutputRange(m_kMinOutput, m_kMaxOutput);
 
     // display PID coefficients on SmartDashboard
     SmartDashboard.putNumber("P Gain", m_kP);
     SmartDashboard.putNumber("I Gain", m_kI);
     SmartDashboard.putNumber("D Gain", m_kD);
     SmartDashboard.putNumber("I Zone", m_kIz);
     SmartDashboard.putNumber("Feed Forward", m_kFF);
     SmartDashboard.putNumber("Max Output", m_kMaxOutput);
     SmartDashboard.putNumber("Min Output", m_kMinOutput);
     SmartDashboard.putNumber("Set Rotations", 0);

  }

  public void moveArmPidDegrees(double degree){
    double rotation = degree/360.0;

    m_pidController.setReference(rotation, CANSparkMax.ControlType.kPosition);
    
    SmartDashboard.putNumber("SetPoint", rotation);
    SmartDashboard.putNumber("ProcessVariable", m_encoder.getPosition());
  }

  /*public void setButtonBox(){
    boolean b_mode_dropoff = m_buttonBox.getLeftBumper();
    boolean b_mode_pickup = m_buttonBox.getRightBumper();

    boolean b_type_cone = m_buttonBox.getLeftStickButton();
    boolean b_type_cube = m_buttonBox.getRightStickButton();

    boolean b_row_one = m_buttonBox.getXButton();
    boolean b_row_two = m_buttonBox.getBButton();
    boolean b_row_three = m_buttonBox.getAButton();

    boolean b_assist_cancelled = m_controller.getBButton();
    boolean b_assist_start = m_controller.getAButton();

    if(b_assist_cancelled){
      resetAssist();
      return;
    }

    if (b_assist_start){
      assist_started = ASSIST_STARTED;
    }

    //drop or pickup

    if(b_mode_dropoff){
      assist_button_mode = ASSIST_MODE_DROPOFF;
    }

    if(b_mode_pickup){
      assist_button_mode = ASSIST_MODE_PICKUP;
    }

    //obj type

    if(b_type_cone){
      assist_obj_type = ASSIST_OBJ_TYPE_CONE;
    }

    if(b_type_cube){
      assist_obj_type = ASSIST_OBJ_TYPE_CUBE;
    }

    //rows

    if(b_row_one){
      assist_drop_row = ASSIST_DROP_ROW_BOTTOM;
    }

    if(b_row_two){
      assist_drop_row = ASSIST_DROP_ROW_MID;
    }

    if(b_row_three){
      assist_drop_row = ASSIST_DROP_ROW_HIGH;
    }

  }*/

  public void buttonValues(){
    SmartDashboard.putNumber("Assist Start", assist_started);
    SmartDashboard.putNumber("Assist Mode", assist_button_mode);
    SmartDashboard.putNumber("Assist Object", assist_obj_type);
    SmartDashboard.putNumber("Assist Drop Row #", assist_drop_row);
    SmartDashboard.putNumber("Assist Step", assist_step);
  }

  public void resetAssist(){
    assist_started = ASSIST_FINISHED;
    assist_button_mode = ASSIST_MODE_UNKNOWN;
    assist_drop_row = ASSIST_DROP_ROW_UNKNOWN;
    assist_obj_type = ASSIST_OBJ_TYPE_UNKNOWN;
    assist_step = 0;
    assist_action_start_time = 0;
  }

   /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightDrive.setInverted(true);
    // Shuffleboard.getTab("Gyro").add(gyro);

    m_arm.setIdleMode(IdleMode.kBrake);
    m_arm.setSmartCurrentLimit(35);
    m_arm.setInverted(true);
    //initizeArmMotorPid();

    //intake here
    m_intake.setIdleMode(IdleMode.kBrake);

    //ground intake here
    //m_gintake_left.setIdleMode(IdleMode.kBrake);
    //m_gintake_right.setIdleMode(IdleMode.kBrake);

    //cameras
    ntTopCamera = NetworkTableInstance.getDefault().getTable("limelight-top");
    // ntTopCamera.getEntry("pipeline").setNumber(0);
    ntIntakeCamera = NetworkTableInstance.getDefault().getTable("limelight-intake");
  }

  double autonomousStartTime;

  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
    m_timer.reset();
    m_timer.start();
    autoBalanceChargingStation = true;
    autoBalanceXMode = false; //only for sample code, we wont use
    autoBalanceYMode = false; //only for sample code, we wont use

    autonomousStartTime = Timer.getFPGATimestamp();
    maxSpeedForBalance = 0.60;

    setDriveDrainMotorBrake();
  }

  public void autoMobilityMode2(double moveBackSeconds){
    double timeElapsed = Timer.getFPGATimestamp() - autonomousStartTime;
    autonomousIntakePower = INTAKE_OUTPUT_POWER;

    if (timeElapsed< ARM_EXTEND_TIME_S) {
      setArmMotor(ARM_OUTPUT_POWER + 0.3);
      setIntakeMotor(-INTAKE_HOLD_POWER, INTAKE_HOLD_CURRENT_LIMIT_A);
      setDriveMotors(0.0, 0.0);
    }
    else if (timeElapsed < ARM_EXTEND_TIME_S + ARM_EXTEND_TIME_S-0.3) {
       setArmMotor(0.0);
       setIntakeMotor(autonomousIntakePower, INTAKE_CURRENT_LIMIT_A);
       setDriveMotors(0.0, 0.0);
    } else if (timeElapsed < ARM_EXTEND_TIME_S + ARM_EXTEND_TIME_S + BACKWARDS_AUTO-0.3 ) {
       setArmMotor(-0.3);
       setIntakeMotor(0.0, INTAKE_CURRENT_LIMIT_A);
       setDriveMotors(0.8, 0.0);
    } else if (timeElapsed < ARM_EXTEND_TIME_S + AUTO_THROW_TIME_S + ARM_EXTEND_TIME_S + BACKWARDS_AUTO-0.3 ) {
       setArmMotor(-0.3);
       setIntakeMotor(0.0, INTAKE_CURRENT_LIMIT_A);
       setDriveMotors(0.0, 0.0);
    } else if (timeElapsed < ARM_EXTEND_TIME_S + AUTO_THROW_TIME_S + ARM_EXTEND_TIME_S + BACKWARDS_AUTO + moveBackSeconds) {
      setArmMotor(0.0);
      setIntakeMotor(0.0, INTAKE_CURRENT_LIMIT_A);
      setDriveMotors(0.3, 0.0);
    } else {
      setArmMotor(0.0);
      setIntakeMotor(0.0, INTAKE_CURRENT_LIMIT_A);
      setDriveMotors(0.0, 0.0);
    }
  }
  
  public void autoMobilityMode(double moveBackSeconds) {
    double timeElapsed = Timer.getFPGATimestamp() - autonomousStartTime;
    autonomousIntakePower = INTAKE_OUTPUT_POWER;

    if (timeElapsed< ARM_EXTEND_TIME_S) {
      setArmMotor(ARM_OUTPUT_POWER + 0.3);
      setIntakeMotor(-INTAKE_HOLD_POWER, INTAKE_HOLD_CURRENT_LIMIT_A);
      setDriveMotors(0.0, 0.0);
    } else if (timeElapsed < ARM_EXTEND_TIME_S + 0.3) {
      setArmMotor(0.0);
      setIntakeMotor(-INTAKE_HOLD_POWER, INTAKE_CURRENT_LIMIT_A);
      setDriveMotors(-0.6, 0.0);
    }
    else if (timeElapsed < ARM_EXTEND_TIME_S + ARM_EXTEND_TIME_S) {
       setArmMotor(0.0);
       setIntakeMotor(autonomousIntakePower, INTAKE_CURRENT_LIMIT_A);
       setDriveMotors(0.0, 0.0);
    } else if (timeElapsed < ARM_EXTEND_TIME_S + ARM_EXTEND_TIME_S + BACKWARDS_AUTO ) {
       setArmMotor(-0.3);
       setIntakeMotor(0.0, INTAKE_CURRENT_LIMIT_A);
       setDriveMotors(0.8, 0.0);
    } else if (timeElapsed < ARM_EXTEND_TIME_S + AUTO_THROW_TIME_S + ARM_EXTEND_TIME_S + BACKWARDS_AUTO ) {
       setArmMotor(-0.3);
       setIntakeMotor(0.0, INTAKE_CURRENT_LIMIT_A);
       setDriveMotors(0.0, 0.0);
    } else if (timeElapsed < ARM_EXTEND_TIME_S + AUTO_THROW_TIME_S + ARM_EXTEND_TIME_S + BACKWARDS_AUTO + moveBackSeconds) {
      setArmMotor(0.0);
      setIntakeMotor(0.0, INTAKE_CURRENT_LIMIT_A);
      setDriveMotors(0.2, 0.0);
    } else {
      setArmMotor(0.0);
      setIntakeMotor(0.0, INTAKE_CURRENT_LIMIT_A);
      setDriveMotors(0.0, 0.0);
    }
  }

  public void autoChargingModeCone(){
    double timeElapsed = Timer.getFPGATimestamp() - autonomousStartTime;
    autonomousIntakePower = INTAKE_OUTPUT_POWER;

    if (timeElapsed< ARM_EXTEND_TIME_S) {
      setArmMotor(ARM_OUTPUT_POWER + 0.3);
      setIntakeMotor(INTAKE_HOLD_POWER, ARM_CURRENT_LIMIT_A);
      setDriveMotors(0.0, 0.0);
    } else if (timeElapsed < ARM_EXTEND_TIME_S + 0.3) {
      setArmMotor(0.0);
      setIntakeMotor(INTAKE_HOLD_POWER, ARM_CURRENT_LIMIT_A);
      setDriveMotors(-0.5, 0.0);
    }
    else if (timeElapsed < ARM_EXTEND_TIME_S + ARM_EXTEND_TIME_S) {
       setArmMotor(0.0);
       setIntakeMotor(autonomousIntakePower, INTAKE_CURRENT_LIMIT_A);
       setDriveMotors(0.0, 0.0);
    } else if (timeElapsed < ARM_EXTEND_TIME_S + ARM_EXTEND_TIME_S + BACKWARDS_AUTO ) {
       setArmMotor(0.0);
       setIntakeMotor(0.0, INTAKE_CURRENT_LIMIT_A);
       setDriveMotors(0.5, 0.0);
    } else if (timeElapsed < ARM_EXTEND_TIME_S + AUTO_THROW_TIME_S + ARM_EXTEND_TIME_S + BACKWARDS_AUTO + 0.5) {
       setArmMotor(-0.2);
       setIntakeMotor(0.0, INTAKE_CURRENT_LIMIT_A);
       setDriveMotors(0.0, 0.0);
    } else if (timeElapsed < ARM_EXTEND_TIME_S + AUTO_THROW_TIME_S + ARM_EXTEND_TIME_S + BACKWARDS_AUTO + 0.5 + 1.32) {
      setArmMotor(-0.2);
      setIntakeMotor(0.0, INTAKE_CURRENT_LIMIT_A);
      setDriveMotors(0.9, 0.0);
    } else if (timeElapsed < ARM_EXTEND_TIME_S + AUTO_THROW_TIME_S + ARM_EXTEND_TIME_S  + BACKWARDS_AUTO + 0.5 + 1.32 + AUTO_DRIVE_TIME) {
      autoBalanceOnCharging();

      setArmMotor(0.0);
      setIntakeMotor(0.0, INTAKE_CURRENT_LIMIT_A);
      setDriveMotors(AUTO_DRIVE_SPEED, 0.0); 
    } else {
      setArmMotor(0.0);
      setIntakeMotor(0.0, INTAKE_CURRENT_LIMIT_A);
      setDriveMotors(0.0, 0.0);
    }


  }

  public void autoChargingModeCube(){
    double timeElapsed = Timer.getFPGATimestamp() - autonomousStartTime;
    autonomousIntakePower = -INTAKE_OUTPUT_POWER;

    if (timeElapsed< ARM_EXTEND_TIME_S) {
      setArmMotor(ARM_OUTPUT_POWER + 0.3);
      setIntakeMotor(-INTAKE_HOLD_POWER, ARM_CURRENT_LIMIT_A);
      setDriveMotors(0.0, 0.0);
    } else if (timeElapsed < ARM_EXTEND_TIME_S + 0.3) {
      setArmMotor(0.0);
      setIntakeMotor(-INTAKE_HOLD_POWER, ARM_CURRENT_LIMIT_A);
      setDriveMotors(-0.5, 0.0);
    }
    else if (timeElapsed < ARM_EXTEND_TIME_S + ARM_EXTEND_TIME_S) {
       setArmMotor(0.0);
       setIntakeMotor(autonomousIntakePower, INTAKE_CURRENT_LIMIT_A);
       setDriveMotors(0.0, 0.0);
    } else if (timeElapsed < ARM_EXTEND_TIME_S + ARM_EXTEND_TIME_S + BACKWARDS_AUTO ) {
       setArmMotor(0.0);
       setIntakeMotor(0.0, INTAKE_CURRENT_LIMIT_A);
       setDriveMotors(0.5, 0.1);
    } else if (timeElapsed < ARM_EXTEND_TIME_S + AUTO_THROW_TIME_S + ARM_EXTEND_TIME_S + BACKWARDS_AUTO + 0.5) {
       setArmMotor(-0.4);
       setIntakeMotor(0.0, INTAKE_CURRENT_LIMIT_A);
       setDriveMotors(0.0, 0.0);
    } else if (timeElapsed < ARM_EXTEND_TIME_S + AUTO_THROW_TIME_S + ARM_EXTEND_TIME_S + BACKWARDS_AUTO + 0.5 + 1.32) {
      setArmMotor(-0.2);
      setIntakeMotor(0.0, INTAKE_CURRENT_LIMIT_A);
      setDriveMotors(0.5, 0.0);
    } else if (timeElapsed < ARM_EXTEND_TIME_S + AUTO_THROW_TIME_S + ARM_EXTEND_TIME_S  + BACKWARDS_AUTO + 0.5 + 1.32 + AUTO_DRIVE_TIME) {
      autoBalanceOnCharging();

      setArmMotor(0.0);
      setIntakeMotor(0.0, INTAKE_CURRENT_LIMIT_A);
      setDriveMotors(AUTO_DRIVE_SPEED, 0.0); 
    } else {
      setArmMotor(0.0);
      setIntakeMotor(0.0, INTAKE_CURRENT_LIMIT_A);
      setDriveMotors(0.0, 0.0);
    }


  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    //logic will be 
    // 1) drive forward by certain seconds
    // 2) use NavX sensor to balance
    // 3) and then stop
    // double fwdSeconds = 1;
    // autoDrivingFwdForSeconds(fwdSeconds);
    // autoBalancingFromSeconds(fwdSeconds);

    if (AUTO_MODE == 1){
      autoMobilityMode2(0.5);
    } else if (AUTO_MODE == 2){
      autoMobilityMode2(1.0);
    } else {
      autoMobilityMode(CUBE);
      smartDashboardGyro();
    }    
  }

  /** This function is called once each time the robot enters teleoperated mode. */
  @Override
  public void teleopInit() {
    autoBalanceChargingStation = false;
    maxSpeedForBalance = 0.63;

    setDriveDrainMotorCoast();
  }

  public void manualMode() { 
    updateAutoBalanceForChargingStation();
    if (autoBalanceChargingStation){
      SmartDashboard.putBoolean("Start_Auto", true);
      autoBalanceOnCharging();
    } else {
      SmartDashboard.putBoolean("Start_Auto", false);
      cubedDrive();
    }

    //smartDashboardGyro(); //test gyro figure, please comment this out in real matches
    /* smartDashboardController(); //test controller figure, please comment this out in real matches
    */
    moveArm();
    intakeNow();
    groundIntakeNow();
  }

  public void pickupCone(){
    int last_step = 1;
    //double intake_duration = 0.5;
    //double extra_suck_duration = 1.0;

    //do step 0 - initialise some parameters
    if (assist_step == 0){
      ntTopCamera.getEntry("pipeline").setNumber(ASSIST_CAMERA_PIPELINE_PICKUP_CONE);
      ntIntakeCamera.getEntry("pipeline").setNumber(ASSIST_CAMERA_PIPELINE_PICKUP_CONE);
      //transist to step 1
      assist_step = 1;

    }

    /*
    //do step 1 - move forward to a position using top camera 
    if (assist_step == 1){
      float Kp = -1.0f;
      float ty = ntTopCamera.getEntry("ty").getFloat(0);

      //ignore some flucation
      if(ty<=-1.0){
        return;
      }

      float speed = Kp*ty;
      if (speed < -0.15){
        speed = -0.15f;
      }
      if (speed >  0.15){
        speed = 0.15f;
      }

      SmartDashboard.putNumber("Assist Top Camera Ty:", ty);
      SmartDashboard.putNumber("Assist Drive Speed:", speed);

      if (ty < 0 && ty > -0.1 ){
        assist_step = 2;
      }else {
        //keep arm and intake as it is 
        setArmMotor(0.0);
        setIntakeMotor(0.0, INTAKE_CURRENT_LIMIT_A);
        setDriveMotors(speed, 0.0);
      }
    }

    //do step 2 - intake and rotate 
    if (assist_step == 2){
      if (assist_action_start_time == 0.0){
        assist_action_start_time = Timer.getFPGATimestamp();
      }

      double timeElapsed = Timer.getFPGATimestamp() - assist_action_start_time;

      if (timeElapsed < intake_duration){
        setIntakeMotor(INTAKE_OUTPUT_POWER, INTAKE_CURRENT_LIMIT_A);
        setArmMotor(-0.2);
        setDriveMotors(0.0, 0.0);
      } else {
        setArmMotor(0.0);
        setIntakeMotor(INTAKE_OUTPUT_POWER, INTAKE_CURRENT_LIMIT_A);
        setDriveMotors(0.0, 0.0);    
        assist_action_start_time = 0;
        assist_step = 3;    
      }
    }

    //suck more 
    if (assist_step == 3){
      if (assist_action_start_time == 0.0){
        assist_action_start_time = Timer.getFPGATimestamp();
      }

      double timeElapsed = Timer.getFPGATimestamp() - assist_action_start_time;
      
      if (timeElapsed < extra_suck_duration){
        setArmMotor(0.0);
        setIntakeMotor(INTAKE_OUTPUT_POWER, INTAKE_CURRENT_LIMIT_A);
        setDriveMotors(0.0, 0.0);
      } else {
        setArmMotor(0.0);
        setIntakeMotor(INTAKE_HOLD_POWER, INTAKE_CURRENT_LIMIT_A);
        setDriveMotors(0.0, 0.0);

        assist_step = 4;
        assist_action_start_time = 0;
      }
    }   
     */

    if (assist_step >= last_step){
      resetAssist();
    }
  }



  public void pickupCube(){
    int last_step = 1;
    //double intake_duration = 0.5;
    //double extra_suck_duration = 1.0;

    //do step 0 - initialise some parameters
    if (assist_step == 0){
      ntTopCamera.getEntry("pipeline").setNumber(ASSIST_CAMERA_PIPELINE_PICKUP_CUBE);
      ntIntakeCamera.getEntry("pipeline").setNumber(ASSIST_CAMERA_PIPELINE_PICKUP_CUBE);
      //transist to step 1
      assist_step = 1;

    }
    
    if (assist_step >= last_step){
      resetAssist();
    }

  }
  


  public void dropToBottomRowWithCone(){
    int last_step = 2;
    double timeElapsed;
    double armDuration = 0.45;
    double dropoffDuration = 0.2;
    double intakeHoldPower = INTAKE_HOLD_POWER;

    //assist step 0 moves the arm up
    if(assist_step == 0){
      if (assist_action_start_time == 0.0){
        assist_action_start_time = Timer.getFPGATimestamp();
      }

      timeElapsed = Timer.getFPGATimestamp() - assist_action_start_time;
      
      if (timeElapsed < armDuration){
        setArmMotor(ARM_OUTPUT_POWER);
        setIntakeMotor(intakeHoldPower, INTAKE_HOLD_CURRENT_LIMIT_A);
        setDriveMotors(0.0, 0.0);  
      } else {
        assist_action_start_time = 0;
        // transit to step 1
        assist_step = 1;

      }

    }

    if(assist_step == 1){
      if (assist_action_start_time == 0.0){
        assist_action_start_time = Timer.getFPGATimestamp();
      }

      timeElapsed = Timer.getFPGATimestamp() - assist_action_start_time;

      if(timeElapsed < dropoffDuration){
        setIntakeMotor(-INTAKE_OUTPUT_POWER, INTAKE_CURRENT_LIMIT_A);
        setArmMotor(0);
        setDriveMotors(0.0, 0.0);  
      } else {
        //transit to step 2
        assist_step = 2;
        assist_action_start_time = 0;
      }
    }

    if(assist_step >= last_step){
      setArmMotor(0);
      setIntakeMotor(0, 0);
      setDriveMotors(0.0, 0.0);  
      
      resetAssist();
    }


  }

  public void dropToBottomRowWithCube(){
    int last_step = 2;
    double timeElapsed;
    double armDuration = 0.53;
    double dropoffDuration = 0.3;
    double intakeHoldPower = -INTAKE_HOLD_POWER;

    //assist step 0 moves the arm up
    if(assist_step == 0){
      if (assist_action_start_time == 0.0){
        assist_action_start_time = Timer.getFPGATimestamp();
      }

      timeElapsed = Timer.getFPGATimestamp() - assist_action_start_time;
      
      if (timeElapsed < armDuration){
        setArmMotor(ARM_OUTPUT_POWER);
        setIntakeMotor(intakeHoldPower, INTAKE_HOLD_CURRENT_LIMIT_A);
        setDriveMotors(0.0, 0.0);  
      } else {
        assist_action_start_time = 0;
        // transit to step 1
        assist_step = 1;

      }

    }

    if(assist_step == 1){
      if (assist_action_start_time == 0.0){
        assist_action_start_time = Timer.getFPGATimestamp();
      }

      timeElapsed = Timer.getFPGATimestamp() - assist_action_start_time;

      if(timeElapsed < dropoffDuration){
        setIntakeMotor(INTAKE_OUTPUT_POWER, INTAKE_CURRENT_LIMIT_A);
        setArmMotor(0);
        setDriveMotors(0.0, 0.0);  
      } else {
        //transit to step 2
        assist_step = 2;
        assist_action_start_time = 0;
      }
    }

    if(assist_step >= last_step){
      setArmMotor(0);
      setIntakeMotor(0, 0);
      setDriveMotors(0.0, 0.0);  

      resetAssist();
    }

  }

  public void dropToMidRowCube(){
    int last_step = 3;
    double timeElapsed;
    double backDuration = 0.15;
    double armDuration = 0.75;
    double dropoffDuration = 0.2;
    double intakeHoldPower = -INTAKE_HOLD_POWER;

    // move backward
    if(assist_step == 0){
      if (assist_action_start_time == 0.0){
        assist_action_start_time = Timer.getFPGATimestamp();
      }

      timeElapsed = Timer.getFPGATimestamp() - assist_action_start_time;

      if (timeElapsed < backDuration){
        setArmMotor(0);
        setIntakeMotor(intakeHoldPower, INTAKE_HOLD_CURRENT_LIMIT_A);
        setDriveMotors(0.25, 0.0); 
      } else {
        assist_action_start_time = 0;
        // transit to step 1

        setDriveMotors(0.0, 0.0);
        assist_step = 1;
      }
    }
    
    // lift arm
    if(assist_step == 1){
      if (assist_action_start_time == 0.0){
        assist_action_start_time = Timer.getFPGATimestamp();
      }

      timeElapsed = Timer.getFPGATimestamp() - assist_action_start_time;
      
      if (timeElapsed < armDuration){
        setArmMotor(0.5);
        setIntakeMotor(intakeHoldPower, INTAKE_HOLD_CURRENT_LIMIT_A);
        setDriveMotors(0.0, 0.0);  
      } else {
        assist_action_start_time = 0;
        // transit to step 1


        assist_step = 2;

      }

    }

    if(assist_step == 2){
      if (assist_action_start_time == 0.0){
        assist_action_start_time = Timer.getFPGATimestamp();
      }

      timeElapsed = Timer.getFPGATimestamp() - assist_action_start_time;

      if (timeElapsed < armDuration){
        setArmMotor(0);
        setIntakeMotor(intakeHoldPower, INTAKE_HOLD_CURRENT_LIMIT_A);
        setDriveMotors(-0.3, 0.0); 
      } else {
        assist_action_start_time = 0;
        
        // transit to step 1
        assist_step = 3;
      }

    }
    
    // if(assist_step == 4){
    //   if (assist_action_start_time == 0.0){
    //     assist_action_start_time = Timer.getFPGATimestamp();
    //   }

    //   timeElapsed = Timer.getFPGATimestamp() - assist_action_start_time;

    //   if(timeElapsed < dropoffDuration){
    //     setIntakeMotor(INTAKE_OUTPUT_POWER, INTAKE_CURRENT_LIMIT_A);
    //     setArmMotor(0);
    //     setDriveMotors(0.0, 0.0);  
    //   } else {
    //     //transit to last step
    //     assist_step = 5;
    //     assist_action_start_time = 0;
    //   }

    // }

    if(assist_step >= last_step){
      // setArmMotor(0);
      // setIntakeMotor(0, 0);
      // setDriveMotors(0.0, 0.0);  

      resetAssist();
    }

  }

  public void dropToMidRowCone(){
    int last_step = 2;
    double timeElapsed;
    double backDuration = 0.35;
    double armDuration = 0.75;
    double dropoffDuration = 0.2;
    double intakeHoldPower = INTAKE_HOLD_POWER; //*5 */

    lastGamePiece = CONE;
    // move backward
    if(assist_step == 0){
      if (assist_action_start_time == 0.0){
        assist_action_start_time = Timer.getFPGATimestamp();
      }

      timeElapsed = Timer.getFPGATimestamp() - assist_action_start_time;

      if (timeElapsed < backDuration){
        setArmMotor(0);
        setIntakeMotor(intakeHoldPower, INTAKE_HOLD_CURRENT_LIMIT_A);
        setDriveMotors(0.25, 0.0); 
      } else {
        assist_action_start_time = 0;
        // transit to step 1

        setDriveMotors(0.0, 0.0);
        assist_step = 1;
      }
    }

    // lift arm
    if(assist_step == 1){
      if (assist_action_start_time == 0.0){
        assist_action_start_time = Timer.getFPGATimestamp();
      }

      timeElapsed = Timer.getFPGATimestamp() - assist_action_start_time;
      
      if (timeElapsed < armDuration){
        setArmMotor(0.53);
        setIntakeMotor(intakeHoldPower, INTAKE_HOLD_CURRENT_LIMIT_A);
        setDriveMotors(0.0, 0.0);  
      } else {
        assist_action_start_time = 0;
        // transit to step 1
        assist_step = 2;

      }

    }

    //move forward//we dont need that too
    /* 
    if(assist_step == 3){
      if (assist_action_start_time == 0.0){
        assist_action_start_time = Timer.getFPGATimestamp();
      }

      timeElapsed = Timer.getFPGATimestamp() - assist_action_start_time;

      if (timeElapsed < armDuration){
        setArmMotor(0);
        setIntakeMotor(intakeHoldPower, INTAKE_HOLD_CURRENT_LIMIT_A);
        setDriveMotors(-0.17, 0.0); 
      } else {
        assist_action_start_time = 0;
        
        // transit to step 1
        assist_step = 4;
      }

    }
    */
    /*
    if(assist_step == 2){
      if (assist_action_start_time == 0.0){
        assist_action_start_time = Timer.getFPGATimestamp();
      }

      timeElapsed = Timer.getFPGATimestamp() - assist_action_start_time;
      
      if (timeElapsed < armDuration){
        setArmMotor(0);
        setIntakeMotor(-INTAKE_OUTPUT_POWER, INTAKE_CURRENT_LIMIT_A);
        setDriveMotors(0.0, 0.0);  
      } else {
        assist_action_start_time = 0;
        // transit to step 1
        assist_step = 4;

      }

    }
     * 
     */
    
    if(assist_step >= last_step){
      // setArmMotor(0);
      // setIntakeMotor(0, 0);
      // setDriveMotors(0.0, 0.0);  

      resetAssist();
    }

  }

  public void dropToTopRowCube(){
    int last_step = 3;
    double timeElapsed;
    double backDuration = 0.35;
    double armDuration = 0.85;
    double dropoffDuration = 0.2;
    double intakeHoldPower = -INTAKE_HOLD_POWER;

    // move backward
    if(assist_step == 0){
      if (assist_action_start_time == 0.0){
        assist_action_start_time = Timer.getFPGATimestamp();
      }

      timeElapsed = Timer.getFPGATimestamp() - assist_action_start_time;

      if (timeElapsed < backDuration){
        setArmMotor(0);
        setIntakeMotor(intakeHoldPower, INTAKE_HOLD_CURRENT_LIMIT_A);
        setDriveMotors(0.25, 0.0); 
      } else {
        assist_action_start_time = 0;
        // transit to step 1

        setDriveMotors(0.0, 0.0);
        assist_step = 1;
      }
    }

    // lift arm
    if(assist_step == 1){
      if (assist_action_start_time == 0.0){
        assist_action_start_time = Timer.getFPGATimestamp();
      }

      timeElapsed = Timer.getFPGATimestamp() - assist_action_start_time;
      
      if (timeElapsed < armDuration){
        setArmMotor(0.7);
        setIntakeMotor(intakeHoldPower, INTAKE_HOLD_CURRENT_LIMIT_A);
        setDriveMotors(0.0, 0.0);  
      } else {
        assist_action_start_time = 0;
        // transit to step 1


        assist_step = 2;

      }

    }

    if(assist_step == 2){
      if (assist_action_start_time == 0.0){
        assist_action_start_time = Timer.getFPGATimestamp();
      }

      timeElapsed = Timer.getFPGATimestamp() - assist_action_start_time;

      if (timeElapsed < armDuration){
        setArmMotor(0);
        setIntakeMotor(intakeHoldPower, INTAKE_HOLD_CURRENT_LIMIT_A);
        setDriveMotors(-0.3, 0.0); 
      } else {
        assist_action_start_time = 0;
        
        // transit to step 1
        assist_step = 3;
      }

    }

    /*if(assist_step == 3){
      if (assist_action_start_time == 0.0){
        assist_action_start_time = Timer.getFPGATimestamp();
      }

      timeElapsed = Timer.getFPGATimestamp() - assist_action_start_time;

      if(timeElapsed < dropoffDuration){
        setIntakeMotor(INTAKE_OUTPUT_POWER, INTAKE_CURRENT_LIMIT_A);
        setArmMotor(0);
        setDriveMotors(0.0, 0.0);  
      } else {
        //transit to last step
        assist_step = 4;
        assist_action_start_time = 0;
      }

    }*/

    if(assist_step >= last_step){
      /*setArmMotor(0);
      setIntakeMotor(0, 0);
      setDriveMotors(0.0, 0.0);*/  

      resetAssist();
    }

  }

  public void dropToTopRowCone(){
    int last_step = 3;
    double timeElapsed;
    double backDuration = 0.4;
    double armDuration = 1.0;
    double forwardDuration = 0.32;
    double dropoffDuration = 0.2;
    double intakeHoldPower = INTAKE_HOLD_POWER;

    // move backward
    if(assist_step == 0){
      if (assist_action_start_time == 0.0){
        assist_action_start_time = Timer.getFPGATimestamp();
      }

      timeElapsed = Timer.getFPGATimestamp() - assist_action_start_time;

      if (timeElapsed < backDuration){
        setArmMotor(0);
        setIntakeMotor(intakeHoldPower, INTAKE_HOLD_CURRENT_LIMIT_A);
        setDriveMotors(0.25, 0.0); 
      } else {
        assist_action_start_time = 0;
        // transit to step 1

        setDriveMotors(0.0, 0.0);
        assist_step = 1;
      }
    }

    // lift arm
    if(assist_step == 1){
      if (assist_action_start_time == 0.0){
        assist_action_start_time = Timer.getFPGATimestamp();
      }

      timeElapsed = Timer.getFPGATimestamp() - assist_action_start_time;
      
      if (timeElapsed < armDuration){
        setArmMotor(0.7);
        setIntakeMotor(intakeHoldPower, INTAKE_HOLD_CURRENT_LIMIT_A);
        setDriveMotors(0.0, 0.0);  
      } else {
        assist_action_start_time = 0;
        // transit to step 1


        assist_step = 2;

      }

    }

    if(assist_step == 2){
      if (assist_action_start_time == 0.0){
        assist_action_start_time = Timer.getFPGATimestamp();
      }

      timeElapsed = Timer.getFPGATimestamp() - assist_action_start_time;

      if (timeElapsed < forwardDuration){
        setArmMotor(0);
        setIntakeMotor(intakeHoldPower, INTAKE_HOLD_CURRENT_LIMIT_A);
        setDriveMotors(-0.3, 0.0); 
      } else {
        assist_action_start_time = 0;
        
        // transit to step 1
        assist_step = 3;
      }

    }
    


    if(assist_step >= last_step){
      /*setArmMotor(0);
      setIntakeMotor(0, 0);
      setDriveMotors(0.0, 0.0);*/

      resetAssist();
    }
  }


  //assist mode features
  public void assistMode(){
    //todo logic will be here for player assist
    if (assist_button_mode == ASSIST_MODE_PICKUP){
      if (assist_obj_type == ASSIST_OBJ_TYPE_CONE){
        pickupCone();
      }

      if (assist_obj_type == ASSIST_OBJ_TYPE_CUBE){
        pickupCube();

      }
    }

    if (assist_button_mode == ASSIST_MODE_DROPOFF){
      if (assist_drop_row == ASSIST_DROP_ROW_BOTTOM){
        if (assist_obj_type == ASSIST_OBJ_TYPE_CONE){
          dropToBottomRowWithCone();
        }
        if(assist_obj_type == ASSIST_OBJ_TYPE_CUBE){
          dropToBottomRowWithCube();
        }
      }
    }

    if (assist_button_mode == ASSIST_MODE_DROPOFF){
      if (assist_drop_row == ASSIST_DROP_ROW_MID){
        if (assist_obj_type == ASSIST_OBJ_TYPE_CONE){
          dropToMidRowCone();
        }
        if(assist_obj_type == ASSIST_OBJ_TYPE_CUBE){
          dropToMidRowCube();
        }
      }
    }

    if (assist_button_mode == ASSIST_MODE_DROPOFF){
      if (assist_drop_row == ASSIST_DROP_ROW_HIGH){
        if (assist_obj_type == ASSIST_OBJ_TYPE_CONE){
          dropToTopRowCone();
        }
        if(assist_obj_type == ASSIST_OBJ_TYPE_CUBE){
          dropToTopRowCube();
        }
      }
    }
  }
  
  /** This is function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {
    //The generical concept here is that
    // 1) normally, we will use optimised cubedDrive
    // 2) but controller 2 can trig auto assistant if required 
    // 2.a) the automatic assitance mode can be trigged with driver 2 stick 
    // 2.b) the automatic assitance mode can be disabled with driver 1 stick
    /** comment out test the arm motor only */

    //setButtonBox();

    if (assist_started == 1){
      assistMode();
    }else {
      //for now we disable other manual driving for safty
      manualMode();      
    }

    buttonValues();
    

  }

  


  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {
    autonomousStartTime = Timer.getFPGATimestamp();
    maxSpeedForBalance = 0.63;


  }

  private void setDriveDrainMotorBrake() {
    m_frontLeft.setIdleMode(IdleMode.kBrake);
    m_frontRight.setIdleMode(IdleMode.kBrake);
    m_rearLeft.setIdleMode(IdleMode.kBrake);
    m_rearRight.setIdleMode(IdleMode.kBrake);
  }

  private void setDriveDrainMotorCoast(){
    m_frontLeft.setIdleMode(IdleMode.kCoast);
    m_frontRight.setIdleMode(IdleMode.kCoast);
    m_rearLeft.setIdleMode(IdleMode.kCoast);
    m_rearRight.setIdleMode(IdleMode.kCoast);    
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    double timeElapsed = Timer.getFPGATimestamp() - autonomousStartTime;
    double backDuration = 0.4;
    double balancingTime = 20;
    if ( timeElapsed < backDuration){
      setDriveMotors(0.7, 0);
    } else if (timeElapsed < backDuration + balancingTime) {
      autoBalanceOnCharging();
    } else {
      setDriveMotors(0, 0);
    }
    
    smartDashboardGyro(); //test gyro figure, please comment this out in real matches
  }

  /** Below IS Testing/Debugging Feature Area */
  /**
   * Sample code copying from NavX
   * Very good for debugging purpose
   */
  private void smartDashboardGyro() {
    boolean zero_yaw_pressed = m_controller.getAButtonPressed();
    if (zero_yaw_pressed) {
      gyro.zeroYaw();
    }

    /* Display 6-axis Processed Angle Data */
    SmartDashboard.putBoolean("IMU_Connected", gyro.isConnected());
    SmartDashboard.putBoolean("IMU_IsCalibrating", gyro.isCalibrating());
    SmartDashboard.putNumber("IMU_Yaw", gyro.getYaw());
    SmartDashboard.putNumber("IMU_Pitch", gyro.getPitch());
    SmartDashboard.putNumber("IMU_Roll", gyro.getRoll());

    /* Display tilt-corrected, Magnetometer-based heading (requires */
    /* magnetometer calibration to be useful) */

    SmartDashboard.putNumber("IMU_CompassHeading", gyro.getCompassHeading());

    /* Display 9-axis Heading (requires magnetometer calibration to be useful) */
    SmartDashboard.putNumber("IMU_FusedHeading", gyro.getFusedHeading());

    /* These functions are compatible w/the WPI Gyro Class, providing a simple */
    /* path for upgrading from the Kit-of-Parts gyro to the navx MXP */

    SmartDashboard.putNumber("IMU_TotalYaw", gyro.getAngle());
    SmartDashboard.putNumber("IMU_YawRateDPS", gyro.getRate());

    /* Display Processed Acceleration Data (Linear Acceleration, Motion Detect) */

    SmartDashboard.putNumber("IMU_Accel_X", gyro.getWorldLinearAccelX());
    SmartDashboard.putNumber("IMU_Accel_Y", gyro.getWorldLinearAccelY());
    SmartDashboard.putBoolean("IMU_IsMoving", gyro.isMoving());
    SmartDashboard.putBoolean("IMU_IsRotating", gyro.isRotating());

    /* Display estimates of velocity/displacement. Note that these values are */
    /* not expected to be accurate enough for estimating robot position on a */
    /* FIRST FRC Robotics Field, due to accelerometer noise and the compounding */
    /* of these errors due to single (velocity) integration and especially */
    /* double (displacement) integration. */

    SmartDashboard.putNumber("Velocity_X", gyro.getVelocityX());
    SmartDashboard.putNumber("Velocity_Y", gyro.getVelocityY());
    SmartDashboard.putNumber("Displacement_X", gyro.getDisplacementX());
    SmartDashboard.putNumber("Displacement_Y", gyro.getDisplacementY());

    /* Display Raw Gyro/Accelerometer/Magnetometer Values */
    /* NOTE: These values are not normally necessary, but are made available */
    /* for advanced users. Before using this data, please consider whether */
    /* the processed data (see above) will suit your needs. */

    SmartDashboard.putNumber("RawGyro_X", gyro.getRawGyroX());
    SmartDashboard.putNumber("RawGyro_Y", gyro.getRawGyroY());
    SmartDashboard.putNumber("RawGyro_Z", gyro.getRawGyroZ());
    SmartDashboard.putNumber("RawAccel_X", gyro.getRawAccelX());
    SmartDashboard.putNumber("RawAccel_Y", gyro.getRawAccelY());
    SmartDashboard.putNumber("RawAccel_Z", gyro.getRawAccelZ());
    SmartDashboard.putNumber("RawMag_X", gyro.getRawMagX());
    SmartDashboard.putNumber("RawMag_Y", gyro.getRawMagY());
    SmartDashboard.putNumber("RawMag_Z", gyro.getRawMagZ());
    SmartDashboard.putNumber("IMU_Temp_C", gyro.getTempC());
    SmartDashboard.putNumber("IMU_Timestamp", gyro.getLastSensorTimestamp());

    /* Omnimount Yaw Axis Information */
    /* For more info, see http://navx-mxp.kauailabs.com/installation/omnimount */
    AHRS.BoardYawAxis yaw_axis = gyro.getBoardYawAxis();
    SmartDashboard.putString("YawAxisDirection", yaw_axis.up ? "Up" : "Down");
    SmartDashboard.putNumber("YawAxis", yaw_axis.board_axis.getValue());

    /* Sensor Board Information */
    SmartDashboard.putString("FirmwareVersion", gyro.getFirmwareVersion());

    /* Quaternion Data */
    /* Quaternions are fascinating, and are the most compact representation of */
    /* orientation data. All of the Yaw, Pitch and Roll Values can be derived */
    /* from the Quaternions. If interested in motion processing, knowledge of */
    /* Quaternions is highly recommended. */
    SmartDashboard.putNumber("QuaternionW", gyro.getQuaternionW());
    SmartDashboard.putNumber("QuaternionX", gyro.getQuaternionX());
    SmartDashboard.putNumber("QuaternionY", gyro.getQuaternionY());
    SmartDashboard.putNumber("QuaternionZ", gyro.getQuaternionZ());

    /* Connectivity Debugging Support */
    SmartDashboard.putNumber("IMU_Byte_Count", gyro.getByteCount());
    SmartDashboard.putNumber("IMU_Update_Count", gyro.getUpdateCount());
    SmartDashboard.putNumber("IMU Rate", gyro.getRate());  
  }

  private void smartDashboardController(){
    SmartDashboard.putNumber("CL_X", m_controller.getLeftX());
    SmartDashboard.putNumber("CL_Y", m_controller.getLeftY());
    SmartDashboard.putNumber("CR_X", m_controller.getRightX());
    SmartDashboard.putNumber("CR_Y", m_controller.getRightY());
  }

}