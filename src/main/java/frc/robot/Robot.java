package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Robot extends TimedRobot {


  //底盤馬達宣告
  WPI_TalonFX leftFront = new WPI_TalonFX(1);
  WPI_TalonFX leftBack = new WPI_TalonFX(2);
  WPI_TalonFX rightFront = new WPI_TalonFX(3);
  WPI_TalonFX rightBack = new WPI_TalonFX(4);

  // SpeedControllerGroup leftDrive = new SpeedControllerGroup(leftFront, leftBack);
  // SpeedControllerGroup  rightDrive = new SpeedControllerGroup(rightFront, rightBack);
  // DifferentialDrive differentialDrive = new DifferentialDrive(leftDrive, rightDrive);

  //射球馬達宣告
  WPI_TalonFX masterShooter = new WPI_TalonFX(9);
  WPI_TalonFX slaveeShooter1 = new WPI_TalonFX(5);
  WPI_TalonFX slaveeShooter2 = new WPI_TalonFX(6);

  //跟隨馬達宣告
  WPI_TalonFX transporterDown = new WPI_TalonFX(10);
  WPI_TalonFX transporterUp = new WPI_TalonFX(8);

  //吸球馬達宣告
  WPI_TalonFX intaker = new WPI_TalonFX(7);

  //空壓機 Compressor
  Compressor compressor = new Compressor(0);
  //電磁閥 Solenoid

  DoubleSolenoid suckerS = new DoubleSolenoid(0, 1);

  //搖桿按鈕宣告
  Joystick js1 = new Joystick(0);
  Joystick js2 = new Joystick(1);
  public final int leftStick_X = 0;
  public final int leftStick_Y = 1;
  public final int rightStick_X = 4;
  public final int rightStick_Y = 5;
  public final int trigger_L =2;
  public final int trigger_R =3;
  public final int Btn_A = 1;
  public final int Btn_B = 2;
  public final int Btn_X = 3;
  public final int Btn_Y = 4;
  public final int Btn_LB = 5;
  public final int Btn_RB = 6;
  public final int Btn_LS = 9;  
  public final int Btn_RS = 10;

  //變數
  public boolean collectorContinous = false ;
  public boolean aimContinous = false;
  public double sensitivity = 0.3;

  double Speed = 0.272;

  //limelight宣告
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tv = table.getEntry("tv");  
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");

  //陀螺儀宣告
  AHRS gyro = new AHRS(SPI.Port.kMXP);

  //PID
  int kSlotIdx = 0;
  int kTimeoutMs = 30;

  //timer
  double start_time = Timer.getFPGATimestamp();

  @Override
  public void robotInit() {

    compressor.start();
    gyro.calibrate();//校正


    //底盤反轉與歸零
    leftFront.setInverted(true);
    leftBack.setInverted(true);
    rightFront.setInverted(false);
    rightBack.setInverted(false);

    leftFront.setSelectedSensorPosition(0);
    leftBack.setSelectedSensorPosition(0);
    rightFront.setSelectedSensorPosition(0);
    rightBack.setSelectedSensorPosition(0);

    //射球反轉與歸零
    slaveeShooter1.setInverted(true);
    slaveeShooter2.setInverted(false);
    masterShooter.setInverted(false);

    slaveeShooter1.setSelectedSensorPosition(0);
    slaveeShooter2.setSelectedSensorPosition(0);
    masterShooter.setSelectedSensorPosition(0);

    //運輸反轉
    transporterUp.setInverted(false);
    transporterDown.setInverted(false);

    //吸球反轉
    intaker.setInverted(true);

    compressor.start();
    // suckerS.set(Value.kReverse);
    collectorContinous  = false;

    //吸球反轉
    intaker.setInverted(false);

  }

  @Override
  public void robotPeriodic() {
    boolean v = tv.getBoolean(false);
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);
    SmartDashboard.putBoolean("LimelightV", v);
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y); 
    SmartDashboard.putNumber("LimelightArea", area);
    
    SmartDashboard.putNumber("gyro_angle", gyro.getAngle());

    SmartDashboard.putNumber("leftFront_pos", leftFront.getSelectedSensorPosition());
    SmartDashboard.putNumber("leftBack_pos", leftBack.getSelectedSensorPosition());
    SmartDashboard.putNumber("rightFront_pos", rightFront.getSelectedSensorPosition());
    SmartDashboard.putNumber("rightBack_pos", rightBack.getSelectedSensorPosition());

    SmartDashboard.putNumber("leftFront_velocity", leftFront.getSelectedSensorVelocity());
    SmartDashboard.putNumber("leftBack_velocity", leftBack.getSelectedSensorVelocity());
    SmartDashboard.putNumber("rightFront_velocity", rightFront.getSelectedSensorVelocity());
    SmartDashboard.putNumber("rightBack_velocity", rightBack.getSelectedSensorVelocity());

    SmartDashboard.putNumber("js2_x", js2.getRawAxis(rightStick_X));
    SmartDashboard.putNumber("js_y", js2.getRawAxis(leftStick_Y));
  }

  @Override
  public void autonomousInit() {
    gyro.reset();
  }

  @Override

  public void autonomousPeriodic() {

    //case1 射三顆球
    double a2 = ty.getDouble(0.0);
    double forSpeed = 0.23;
    //double forSpeed = 0.0000007*Math.pow(a2, 6) + 0.00002*Math.pow(a2, 5) + 0.0003*Math.pow(a2, 4) + 0.0015*Math.pow(a2, 3) + 0.0027*Math.pow(a2, 2) + 0.005*a2 + 0.1001;
    masterShooter.set(ControlMode.PercentOutput, 0.3);
    slaveeShooter1.set(ControlMode.PercentOutput, 0.3+forSpeed);
    slaveeShooter2.set(ControlMode.PercentOutput, 0.3+forSpeed);
    Timer.delay(0.5);
    transporterUp.set(ControlMode.PercentOutput, 1);
    transporterDown.set(ControlMode.PercentOutput, 1);
    Timer.delay(6);
    masterShooter.set(ControlMode.PercentOutput, 0);
    slaveeShooter1.set(ControlMode.PercentOutput, 0);
    slaveeShooter2.set(ControlMode.PercentOutput, 0);
    transporterUp.set(ControlMode.PercentOutput, 0);
    transporterDown.set(ControlMode.PercentOutput, 0);
    
    // differentialDrive.tankDrive(0.5, 0.5);
    // Timer.delay(3);
    // differentialDrive.tankDrive(0.0, 0.0);

    leftFront.set(0.2);
    leftBack.set(0.2);
    rightFront.set(0.2);
    rightBack.set(0.2);
    Timer.delay(2);
    leftFront.set(0);
    leftBack.set(0);
    rightFront.set(0);
    rightBack.set(0);
    Timer.delay(6.5);
    

    /*//case2
    double time = Timer.getFPGATimestamp();
    if(time - start_time < 2){
      leftFront.set(0.5);
      leftBack.set(0.5);
      rightFront.set(0.5);
      rightBack.set(0.5);
    }else{
      leftFront.set(0);
      leftBack.set(0);
      rightFront.set(0);
      rightBack.set(0);
    }

    if(time - start_time < 4){
      leftFront.set(-0.5);
      leftBack.set(-0.5);
      rightFront.set(0.5);
      rightBack.set(0.5);
    }else{
      leftFront.set(0);
      leftBack.set(0);
      rightFront.set(0);
      rightBack.set(0);
    }*/
  }

  private double getGyroPos() {
    return gyro.getAngle();
  }

  @Override
  public void teleopInit() {
    gyro.reset(); //歸零
  }

  @Override
  public void teleopPeriodic() {
      //differentialDrive.tankDrive(-js1.getRawAxis(leftStick_Y), -js1.getRawAxis(rightStick_Y));
      //differentialDrive.arcadeDrive(-js2.getRawAxis(rightStick_X) * 0.6, js2.getRawAxis(leftStick_Y) * 0.705);

      
      if(js2.getRawAxis(leftStick_Y) > 0.1 || js2.getRawAxis(leftStick_Y) < -0.1){
        leftFront.set(ControlMode.PercentOutput, js2.getRawAxis(leftStick_Y) * 0.7);
        leftBack.set(ControlMode.PercentOutput, js2.getRawAxis(leftStick_Y) * 0.7);
        rightFront.set(ControlMode.PercentOutput, js2.getRawAxis(leftStick_Y) * 0.7);
        rightBack.set(ControlMode.PercentOutput, js2.getRawAxis(leftStick_Y) * 0.7);
      }else if(js2.getRawAxis(rightStick_X) > 0.1 || js2.getRawAxis(rightStick_X) < -0.1){
        leftFront.set(ControlMode.PercentOutput, js2.getRawAxis(rightStick_X) * -0.7);
        leftBack.set(ControlMode.PercentOutput, js2.getRawAxis(rightStick_X) * -0.7);
        rightFront.set(ControlMode.PercentOutput, js2.getRawAxis(rightStick_X) * 0.7);
        rightBack.set(ControlMode.PercentOutput, js2.getRawAxis(rightStick_X) * 0.7);
      }else{
        leftFront.set(ControlMode.PercentOutput,0);
        leftBack.set(ControlMode.PercentOutput,0);
        rightFront.set(ControlMode.PercentOutput,0);
        rightBack.set(ControlMode.PercentOutput,0);
      }

      // if(js2.getRawAxis(leftStick_Y) > 0.1 || js2.getRawAxis(leftStick_Y) < 0.1){
      //   leftFront.set(ControlMode.PercentOutput, js2.getRawAxis(leftStick_Y) * 0.7);
      //   leftBack.set(ControlMode.PercentOutput, js2.getRawAxis(leftStick_Y) * 0.7);
      // }
      // if(js2.getRawAxis(rightStick_Y)) > 0.1 || js2.getRawAxis(rightStick_Y) < 0.1){
      //   leftFront.set(ControlMode.PercentOutput, js2.getRawAxis(leftStick_Y) * 0.7);
      //   leftBack.set(ControlMode.PercentOutput, js2.getRawAxis(leftStick_Y) * 0.7);
      // }



      /*if(js2.getRawAxis(leftStick_Y) > 0.05 || js2.getRawAxis(leftStick_Y) < 0.05){
        leftFront.set(ControlMode.PercentOutput, js2.getRawAxis(leftStick_Y) * 0.7);
        leftBack.set(ControlMode.PercentOutput, js2.getRawAxis(leftStick_Y) * 0.7);
        rightFront.set(ControlMode.PercentOutput, js2.getRawAxis(leftStick_Y) * 0.7);
        rightBack.set(ControlMode.PercentOutput, js2.getRawAxis(leftStick_Y) * 0.7);
      }else if(js2.getRawAxis(rightStick_X) > 0.05 || js2.getRawAxis(rightStick_X) < 0.05){
        leftFront.set(ControlMode.PercentOutput, js2.getRawAxis(rightStick_X) * -0.7);
        leftBack.set(ControlMode.PercentOutput, js2.getRawAxis(rightStick_X) * -0.7);
        rightFront.set(ControlMode.PercentOutput, js2.getRawAxis(rightStick_X) * 0.7);
        rightBack.set(ControlMode.PercentOutput, js2.getRawAxis(rightStick_X) * 0.7);
      }else{
        leftFront.set(0);
        leftBack.set(0);
        rightFront.set(0);
        rightBack.set(0);
      }*/


      //射球(快) trigger_R 射球(慢) trigger_L
      if(js1.getRawAxis(trigger_R) > 0){
        //double a2 = ty.getDouble(0.0);
        double forSpeed = 0.26;
        /*double forDis = 0.0016*Math.pow(a2,4) -0.0701*Math.pow(a2,3) +1.6367*Math.pow(a2,2) -34.248*a2 +629.2;
        double forSpeed = 0.000001*Math.pow(forDis,2) -0.0011*forDis +0.4244;*/
        //double forSpeed = 0.0009*Math.pow(a2,2) - 0.0133*a2 + 0.2203;
        //double forSpeed = 0.0000007*Math.pow(a2, 6) + 0.00002*Math.pow(a2, 5) + 0.0003*Math.pow(a2, 4) + 0.0015*Math.pow(a2, 3) + 0.0027*Math.pow(a2, 2) + 0.005*a2 + 0.2101;
        //0.0009x2 - 0.0133x + 0.2203
        //7E-07x6 - 2E-05x5 + 0.0003x4 - 0.0015x3 + 0.0027x2 - 0.0051x + 0.2101

        masterShooter.set(ControlMode.PercentOutput, 0.3); 
        slaveeShooter1.set(ControlMode.PercentOutput, 0.3+forSpeed); 
        slaveeShooter2.set(ControlMode.PercentOutput, 0.3+forSpeed);
        Timer.delay(0.5);
        transporterUp.set(ControlMode.PercentOutput, 1);
        transporterDown.set(ControlMode.PercentOutput, 1);
      }else if(js1.getRawAxis(trigger_L) > 0){
        //double a2 = ty.getDouble(0.0);
        /*double forDis = 0.0016*Math.pow(a2,4) -0.0701*Math.pow(a2,3) +1.6367*Math.pow(a2,2) -34.248*a2 +629.2;
        double forSpeed = 0.000001*Math.pow(forDis,2) -0.0011*forDis +0.4244;*/
        //double forSpeed = 0.0009*Math.pow(a2,2) - 0.0133*a2 + 0.2203;
        //double forSpeed = 0.0000007*Math.pow(a2, 6) + 0.00002*Math.pow(a2, 5) + 0.0003*Math.pow(a2, 4) + 0.0015*Math.pow(a2, 3) + 0.0027*Math.pow(a2, 2) + 0.005*a2 + 0.2101;
        double forSpeed = 0.23;
        masterShooter.set(ControlMode.PercentOutput, 0.3); 
        slaveeShooter1.set(ControlMode.PercentOutput, 0.3+forSpeed); 
        slaveeShooter2.set(ControlMode.PercentOutput, 0.3+forSpeed);
        Timer.delay(0.5);
        transporterUp.set(ControlMode.PercentOutput, 1);
        transporterDown.set(ControlMode.PercentOutput, 1);
      }else{
        masterShooter.set(ControlMode.PercentOutput, 0); 
        slaveeShooter1.set(ControlMode.PercentOutput, 0); 
        slaveeShooter2.set(ControlMode.PercentOutput, 0);
        transporterUp.set(ControlMode.PercentOutput, 0);
        transporterDown.set(ControlMode.PercentOutput, 0);
      }

      if(js1.getRawButton(Btn_RB)) {
        masterShooter.set(ControlMode.PercentOutput, -0.3); 
        slaveeShooter1.set(ControlMode.PercentOutput, -0.3-0.23); 
        slaveeShooter2.set(ControlMode.PercentOutput, -0.3-0.23);
        Timer.delay(0.5);
        transporterUp.set(ControlMode.PercentOutput, -1);
        transporterDown.set(ControlMode.PercentOutput, -1);
      }
  }
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }
}