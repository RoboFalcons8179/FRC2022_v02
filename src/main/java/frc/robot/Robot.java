// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.PowerDistribution;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.wpilibj.Joystick;




// import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  // private static final String kDefaultAuto = "Default";
  // private static final String kCustomAuto = "My Auto";
  // private String m_autoSelected;
  // private final SendableChooser<String> m_chooser = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
 	private static PowerDistribution examplePD = new PowerDistribution();

	// SAFETY

	private static final double MAX_SPEED = 500;

	// DRIVE MOTORS
	private static WPI_TalonSRX rightDrive = new WPI_TalonSRX(1);
	private static WPI_TalonSRX leftDrive = new WPI_TalonSRX(3);
	
	private static WPI_VictorSPX rightFollow = new WPI_VictorSPX(2);
	private static WPI_VictorSPX leftFollow = new WPI_VictorSPX(4);


	// JOYSTICKS
	private static Joystick xbox_0 = new Joystick(0);

	// MOTION SYSTEMS
	private static Velocity vroom = new Velocity(leftDrive, rightDrive, leftFollow, rightFollow, MAX_SPEED);


	@Override
  public void robotInit() {


	////// Set drive motor phases and inversion //////////

	vroom._rightInvert = true;
	vroom._leftInvert = false;

		//does leftDriveInvert == leftFollowInvert? ect.
	vroom._rightFollowSame = true;
	vroom._leftFollowSame = true;

	vroom.rightPhase = false;
	vroom.leftPhase = false;
    
  }


  @Override
  public void robotPeriodic() {}


  @Override
  public void autonomousInit() {


  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {


  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {

    System.out.println(examplePD.getTemperature());

	vroom.vel_initalize();



  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

	double speed = deadband(xbox_0.getRawAxis(1) * -1* MAX_SPEED);
	double rot = deadband(xbox_0.getRawAxis(4) * .6);

	vroom.velPeriodic(speed, rot, true);

	System.out.println("-------------");
	System.out.println(rightDrive.getSelectedSensorVelocity() + leftDrive.getSelectedSensorVelocity());


  }




/** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  private double deadband(double d) {
	if (-0.20 < d && d < 0.20)
		return 0;
	else
		return d;
}
}
