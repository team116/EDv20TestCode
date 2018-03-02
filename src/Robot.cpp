/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <iostream>
#include <string>

#include "WPILib.h"
#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>

#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"

#include <Drive/DifferentialDrive.h>
#include <IterativeRobot.h>
#include <PowerDistributionPanel.h>
#include <ctre/phoenix.h>
#include <SpeedControllerGroup.h>
#include <Joystick.h>
#include <XboxController.h>
#include <CameraServer.h>
#include <DoubleSolenoid.h>
#include <Solenoid.h>
#include <Ultrasonic.h>
#include <Encoder.h>
#include <AnalogInput.h>

#define roboRioEncoders 	false
#define talonSRXEncoders 	false
#define USE_NAVX   			false
#define CNTL_BOX_A 			true
#define CNTL_BOX_B 			false
#define	ON_ROBOT   			true
#define ENABLE_USB_CAMERA	false
#define DUAL_JOYSTICKS		true
#define XBOX_CONTROLLER		false
#define OI_ENABLED			true

#if USE_NAVX
// NavX-MXP headers
#include "AHRS.h"
#endif

using namespace frc;
using namespace std;
using namespace nt;

class Robot: public frc::IterativeRobot {

public:
	void RobotInit() {
		m_chooser.AddDefault(kAutoNameDoNothing, kAutoNameDoNothing);
//		m_chooser.AddObject(kAutoNameCustom, kAutoNameCustom);
		SmartDashboard::PutData("Auto Modes", &m_chooser);

		// Pneumatics
		roboCompressor.SetClosedLoopControl(true);

		// Put Solenoids into known state
		backBar.Set(DoubleSolenoid::kReverse);
		liftLocker.Set(DoubleSolenoid::kReverse);
		gripper.Set(false);
		intakeDeploy.Set(false);
		engageHook.Set(false);

		m_autoSwitch1.SetOversampleBits(kOversampleBits);
		m_autoSwitch1.SetAverageBits(kAverageBits);
		m_autoSwitch2.SetOversampleBits(kOversampleBits);
		m_autoSwitch2.SetAverageBits(kAverageBits);
		m_liftHeight.SetOversampleBits(kOversampleBits);
		m_liftHeight.SetAverageBits(kAverageBits);
		m_infraredDistance.SetOversampleBits(kOversampleBits);
		m_infraredDistance.SetAverageBits(kAverageBits);

		// Set initial angle on servo
//		ultrasonicServo.SetAngle(90);

// Gyro calibration
//		analogGyro.Calibrate();
//		analogGyro.Reset();

#if ENABLE_USB_CAMERA
		// enable camera
		CameraServer::GetInstance()->StartAutomaticCapture();
#endif

		// Disable the watchdog timers to keep the warning messages down
		m_robotDrive.SetSafetyEnabled(false);

#if talonSRXEncoders
#endif

#if roboRioEncoders
		/* Defines the number of samples to average when determining the
		 * rate. On a quadrature encoder, values range from 1-255; larger
		 * values result in smoother but potentially less accurate
		 * rates than lower values.
		 */
		m_rightEncoder.SetSamplesToAverage(kNumSamplesToAverage);
		m_leftEncoder.SetSamplesToAverage(kNumSamplesToAverage);
		/* Defines how far the mechanism attached to the encoder moves
		 * per pulse.  In this case, we assume that a 360 count encoder
		 * is directly attached to a 6 inch diameter (3.0 inch radius) wheel, and that we want
		 * to measure distance in inches.
		 */
		m_rightEncoder.SetDistancePerPulse(kDistancePerPulse);
		m_leftEncoder.SetDistancePerPulse(kDistancePerPulse);
		/* Defines the lowest rate at which the encoder will not be
		 * considered stopped, for the purposes of the GetStopped() method.
		 * Units are in distance / second, where distance refers to the
		 * units of distance that you are using, in this case inches.
		 */
		m_rightEncoder.SetMinRate(kMinRateNotStopped);
		m_leftEncoder.SetMinRate(kMinRateNotStopped);
#endif

#if USE_NAVX
		NetworkTableInstance::GetDefault().GetTable("datatable");
		try {
			/***********************************************************************
			 * navX-MXP:
			 * - Communication via RoboRIO MXP (SPI, I2C, TTL UART) and USB.
			 * - See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface.
			 *
			 * navX-Micro:
			 * - Communication via I2C (RoboRIO MXP or Onboard) and USB.
			 * - See http://navx-micro.kauailabs.com/guidance/selecting-an-interface.
			 *
			 * Multiple navX-model devices on a single robot are supported.
			 ************************************************************************/
			ahrs = new AHRS(SPI::Port::kMXP);
			//ahrs = new AHRS(I2C::Port::kMXP);
			ahrs->EnableLogging(true);
		} catch (std::exception& ex ) {
			std::string err_string = "Error instantiating navX MXP:  ";
			err_string += ex.what();
			DriverStation::ReportError(err_string.c_str());
		}
#endif
	}

	/*
	 * This autonomous (along with the chooser code above) shows how to
	 * select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * GetString line to get the auto name from the text box below the Gyro.
	 *
	 * You can add additional auto modes by adding additional comparisons to
	 * the
	 * if-else structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as
	 * well.
	 */
	void AutonomousInit() override {
		std::string gameData;
		gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();

		if (gameData.length() > 0) {
			cSwitchVal   = gameData[0];
			cScaleVal    = gameData[1];
			cOppositeVal = gameData[2];
			if (cSwitchVal == 'L') {
				DriverStation::ReportWarning("Switch is on the left");
			} else {
				DriverStation::ReportWarning("Switch is on the right");
			}
			if (cScaleVal == 'L') {
				DriverStation::ReportWarning("Scale is on the left");
			} else {
				DriverStation::ReportWarning("Scale is on the right");
			}
		}

		autoLoopCounter = 0;
		m_autoSelected = m_chooser.GetSelected();
		// m_autoSelected = SmartDashboard::GetString(
		// 		"Auto Selector", kAutoNameDefault);
		std::cout << "Auto selected: " << m_autoSelected << std::endl;

//		if (m_autoSelected == kAutoNameCustom) {
//			// Custom Auto goes here
//		} else {
//			// Default Auto goes here
//		}
	}

	void AutonomousPeriodic() {
//		if (m_autoSelected == kAutoNameCustom) {
		// Custom Auto goes here
//			if(autoLoopCounter < 100) //Check if we've completed 100 loops (approximately 2 seconds)
//			{
//				gyroAngle = analogGyro.GetAngle();
//				// Do something w/ the Angle
//				m_robotDrive.ArcadeDrive(-0.5, -gyroAngle * Kp); 	// drive forwards half speed

//				autoLoopCounter++;
//				} else {
//					m_robotDrive.ArcadeDrive(0.0, 0.0); 	// stop robot
//			}
//		} else {
		// Default Auto goes here
//			if(autoLoopCounter < 100) //Check if we've completed 100 loops (approximately 2 seconds)
//			{
//				gyroAngle = analogGyro.GetAngle();
//				// Do something w/ the Angle
//				m_robotDrive.ArcadeDrive(-0.5, -gyroAngle * Kp); 	// drive forwards half speed
//
//				autoLoopCounter++;
//				} else {
//					m_robotDrive.ArcadeDrive(0.0, 0.0); 	// stop robot
//			}
//		}
	}

	void TeleopInit() {
	}

	double getInfraredDistance(int analogChannel) {
		double distance = 0.0;


		return (distance);
	}

	void TeleopPeriodic() {
		int offsetVal = 1;
		int servoAngle = 80;
		int passCount = 0;
		const int numLoopsPerScan = 100;
		double rightY = 0.0;
		double leftY = 0.0;
		double liftY = 0.0;
		double intakeY = 0.0;
		double intakeX = 0.0;
		double infraredDistance = 0.0;
		bool bRightEncoderDisable = false;
		bool bLeftEncoderDisable = false;
		bool bStringPotDisable = false;
		bool bRtIntakeLSDisable = false;
		bool bLftIntakeLSDisable = false;
		bool bLiftTopLSDisable = false;
		bool bLiftBottomLSDisable = false;
		bool bExtra1Disable = false;
		bool bExtra2Disable = false;
		bool bExtra3Disable = false;

		while (IsOperatorControl()) {

#if talonSRXEncoders
#endif

#if roboRioEncoders
			// Retrieve the net displacement of the Encoders since the last
			// Reset.
			SmartDashboard::PutNumber(
					"Left Encoder Distance", m_leftEncoder.GetDistance());
			SmartDashboard::PutNumber(
					"Right Encoder Distance", m_rightEncoder.GetDistance());

			// Retrieve the current rate of the encoders.
			SmartDashboard::PutNumber(
					"Encoder Rate", m_leftEncoder.GetRate());
			SmartDashboard::PutNumber(
					"Encoder Rate", m_rightEncoder.GetRate());
#endif

#if USE_NAVX
			if ( !ahrs ) return;

			// Uses button 6 on the XBox Controller = RB (right bumper)
			bool reset_yaw_button_pressed = DriverStation::GetInstance().GetStickButton(0,6);
			if ( reset_yaw_button_pressed ) {
				ahrs->ZeroYaw();
			}

			SmartDashboard::PutBoolean( "IMU_Connected", ahrs->IsConnected());
			SmartDashboard::PutNumber( "IMU_Yaw", ahrs->GetYaw());
			SmartDashboard::PutNumber( "IMU_Pitch", ahrs->GetPitch());
			SmartDashboard::PutNumber( "IMU_Roll", ahrs->GetRoll());
			SmartDashboard::PutNumber( "IMU_CompassHeading", ahrs->GetCompassHeading());
			SmartDashboard::PutNumber( "IMU_Update_Count", ahrs->GetUpdateCount());
			SmartDashboard::PutNumber( "IMU_Byte_Count", ahrs->GetByteCount());
			SmartDashboard::PutNumber( "IMU_Timestamp", ahrs->GetLastSensorTimestamp());

			/* These functions are compatible w/the WPI Gyro Class */
			SmartDashboard::PutNumber( "IMU_TotalYaw", ahrs->GetAngle());
			SmartDashboard::PutNumber( "IMU_YawRateDPS", ahrs->GetRate());

			SmartDashboard::PutNumber( "IMU_Accel_X", ahrs->GetWorldLinearAccelX());
			SmartDashboard::PutNumber( "IMU_Accel_Y", ahrs->GetWorldLinearAccelY());
			SmartDashboard::PutBoolean( "IMU_IsMoving", ahrs->IsMoving());
			SmartDashboard::PutNumber( "IMU_Temp_C", ahrs->GetTempC());
			SmartDashboard::PutBoolean( "IMU_IsCalibrating", ahrs->IsCalibrating());

			SmartDashboard::PutNumber( "Velocity_X", ahrs->GetVelocityX() );
			SmartDashboard::PutNumber( "Velocity_Y", ahrs->GetVelocityY() );
			SmartDashboard::PutNumber( "Displacement_X", ahrs->GetDisplacementX() );
			SmartDashboard::PutNumber( "Displacement_Y", ahrs->GetDisplacementY() );

			/* Display Raw Gyro/Accelerometer/Magnetometer Values                       */
			/* NOTE:  These values are not normally necessary, but are made available   */
			/* for advanced users.  Before using this data, please consider whether     */
			/* the processed data (see above) will suit your needs.                     */

			SmartDashboard::PutNumber( "RawGyro_X", ahrs->GetRawGyroX());
			SmartDashboard::PutNumber( "RawGyro_Y", ahrs->GetRawGyroY());
			SmartDashboard::PutNumber( "RawGyro_Z", ahrs->GetRawGyroZ());
			SmartDashboard::PutNumber( "RawAccel_X", ahrs->GetRawAccelX());
			SmartDashboard::PutNumber( "RawAccel_Y", ahrs->GetRawAccelY());
			SmartDashboard::PutNumber( "RawAccel_Z", ahrs->GetRawAccelZ());
			SmartDashboard::PutNumber( "RawMag_X", ahrs->GetRawMagX());
			SmartDashboard::PutNumber( "RawMag_Y", ahrs->GetRawMagY());
			SmartDashboard::PutNumber( "RawMag_Z", ahrs->GetRawMagZ());
			SmartDashboard::PutNumber( "IMU_Temp_C", ahrs->GetTempC());
			/* Omnimount Yaw Axis Information                                           */
			/* For more info, see http://navx-mxp.kauailabs.com/installation/omnimount  */
			AHRS::BoardYawAxis yaw_axis = ahrs->GetBoardYawAxis();
			SmartDashboard::PutString( "YawAxisDirection", yaw_axis.up ? "Up" : "Down" );
			SmartDashboard::PutNumber( "YawAxis", yaw_axis.board_axis );

			/* Sensor Board Information                                                 */
			SmartDashboard::PutString( "FirmwareVersion", ahrs->GetFirmwareVersion());

			/* Quaternion Data                                                          */
			/* Quaternions are fascinating, and are the most compact representation of  */
			/* orientation data.  All of the Yaw, Pitch and Roll Values can be derived  */
			/* from the Quaternions.  If interested in motion processing, knowledge of  */
			/* Quaternions is highly recommended.                                       */
			SmartDashboard::PutNumber( "QuaternionW", ahrs->GetQuaternionW());
			SmartDashboard::PutNumber( "QuaternionX", ahrs->GetQuaternionX());
			SmartDashboard::PutNumber( "QuaternionY", ahrs->GetQuaternionY());
			SmartDashboard::PutNumber( "QuaternionZ", ahrs->GetQuaternionZ());
#endif

#if XBOX_CONTROLLER
			// Implement the deadband
			rightY = -m_stick.GetY(frc::GenericHID::kRightHand);
			leftY = -m_stick.GetY(frc::GenericHID::kLeftHand);
#endif

#if DUAL_JOYSTICKS
			rightY = -m_stickR.GetY();
			leftY = -m_stickL.GetY();
#endif
			if ((rightY > kBottomOfDeadBand) and (rightY < kTopOfDeadBand))
				rightY = 0.0;

			if ((leftY > kBottomOfDeadBand) and (leftY < kTopOfDeadBand))
				leftY = 0.0;

			// Make it so....
			m_robotDrive.TankDrive(leftY, rightY);

#if OI_ENABLED

			// pneumatics
			// Singles
			intakeDeploy.Set(m_sensorBypassStick.GetRawButton(kDeployGrabber));
			gripper.Set(m_controlStick.GetRawButton(kGrabberEngage));
			engageHook.Set(m_controlStick.GetRawButton(kHookDeploy));

			// Doubles
			if (m_controlStick.GetRawButton(kBackBarDeploy)) {
				backBar.Set(DoubleSolenoid::kForward);
			} else {
				backBar.Set(DoubleSolenoid::kReverse);
			}

			if (m_controlStick.GetRawButton(kLiftLockEngage)) {
				liftLocker.Set(DoubleSolenoid::kForward);
			}  else {
				liftLocker.Set(DoubleSolenoid::kReverse);
			}


			//**********************************************************************************
			// Sensor disables
			if (!m_sensorBypassStick.GetRawButton(kDisableInfrared)) {
				infraredDistance = getInfraredDistance(kInfraredChannel);
			} else {
				infraredDistance = 0.0;
			}

			bLiftBottomLSDisable = m_sensorBypassStick.GetRawButton(kDisableLiftBottomLS);
			bLiftTopLSDisable    = m_sensorBypassStick.GetRawButton(kDisableLiftTopLS);
			bRtIntakeLSDisable   = m_sensorBypassStick.GetRawButton(kDisableRightIntakeLS);
			bLftIntakeLSDisable  = m_sensorBypassStick.GetRawButton(kDisableLeftIntakeLS);
			bRightEncoderDisable = m_sensorBypassStick.GetRawButton(kDisableRightEnc);
			bLeftEncoderDisable  = m_sensorBypassStick.GetRawButton(kDisableLeftEnc);
			bStringPotDisable    = m_sensorBypassStick.GetRawButton(kDisableStringPot);
			bExtra1Disable       = m_sensorBypassStick.GetRawButton(kDisableExtra1);
			bExtra2Disable       = m_sensorBypassStick.GetRawButton(kDisableExtra2);
			bExtra3Disable       = m_sensorBypassStick.GetRawButton(kDisableExtra3);

			//**********************************************************************************

			//**********************************************************************************
			// Mobility movement
			if (!bRightEncoderDisable) {

			}

			if (!bLeftEncoderDisable) {

			}

			//**********************************************************************************
			if (!bStringPotDisable) {
				dLiftHeight = m_liftHeight.GetVoltage();
				// TODO: Calculate the height from the voltage
			}
			//**********************************************************************************

			//**********************************************************************************
			// Lift Joystick
			liftY = -m_controlStick.GetRawAxis(kLiftJoystickY);

			if ((liftY > kBottomOfDeadBand) and (liftY < kTopOfDeadBand))
				liftY = 0.0;

			if (!bLiftTopLSDisable) {
				if (m_atLiftTop.Get()) {
					// stop the Lift
					liftY = 0.0;
				}
			}
			if (!bLiftBottomLSDisable) {
				if (m_atLiftBottom.Get()) {
					// stop the Lift
					liftY = 0.0;
				}
			}
			// Make it so....

			m_lift.Set(liftY);


			//**********************************************************************************

			//**********************************************************************************
			// Intake Subsystem
			intakeX = -m_controlStick.GetRawAxis(kIntakeJoystickX);
			intakeY = -m_controlStick.GetRawAxis(kIntakeJoystickY);

			if ((intakeX > kBottomOfDeadBand) and (intakeX < kTopOfDeadBand))
				intakeX = 0.0;

			if ((intakeY > kBottomOfDeadBand) and (intakeY < kTopOfDeadBand))
				intakeY = 0.0;

			// TODO: Figure out the mixing for the Intake Joystick
			//if ()

			// Is Cube contacting the left intake limit switch?
			if (!bLftIntakeLSDisable) {
				if (m_leftIntakeLimitSwitch.Get()) {
					intakeX = 0.0;
				}
			}
			// Is Cube contacting the right intake limit switch?
			if (!bRtIntakeLSDisable) {
				if (m_rightIntakeLimitSwitch.Get()) {
					intakeY = 0.0;
				}
			}

			// Make it so....
			m_rightIntake->Set(intakeX);
			m_leftIntake->Set(intakeY);

			//**********************************************************************************

#endif
			passCount++;

			if ((passCount % numLoopsPerScan) == 0) {
				if (servoAngle <= 80) {
					offsetVal = 1;
				} else if (servoAngle >= 120) {
					offsetVal = -1;
				}

				passCount = 0;
				servoAngle += offsetVal;

			}

		}

	}

	void TestPeriodic() {
	}

private:

	// Character arrays
	char buffer[255];
	char cSwitchVal = ' ';
	char cScaleVal = ' ';
	char cOppositeVal = ' ';


	// CAN IDs  ****************************************************************
	const static int kPdpCanAddress = 15;
	const static int kPcmCanAddress = 1;

	// Channels for the wheels  (CAN IDs)
	const static int frontLeftChannel = 3;
	const static int frontRightChannel = 6;

	const static int rearLeftChannel = 4;
	const static int rearRightChannel = 5;

	// End Effector
	const static int kRightIntake = 7;
	const static int kLeftIntake = 8;
	const static int kSpareMC = 9;
	const static int kRightLift = 10;
	const static int kLeftLift = 11;
	// End CAN IDs *************************************************************

	// Solenoid Assignments
	const static int backBarIn = 0;  // Double
	const static int backBarOut = 1;  // Double
	const static int liftLock = 2;  // Double
	const static int liftUnlock = 3;  // Double
	const static int deployIntake = 4;  // single
	const static int gripCube = 5;  // single
	const static int hook = 6;  // single

	// Analog Port assignments
	const static int kRotarySw1Channel = 0;
	const static int kRotarySw2Channel = 1;
	const static int kLiftHightChannel = 2;
	const static int kInfraredChannel = 3;

	// Digital IOs
	const static int kLeftIntakeLimitSw = 0;
	const static int kRightIntakeLimitSw = 1;
	const static int kLiftBottomLimitSw = 2;
	const static int kLiftTopLimitSw = 3;

	// Joystick port assignments
	const static int kLeftDriverJoystick = 0;
	const static int kRightDriverJoystick = 1;
	const static int kDisableSwitchJoystick = 2;
	const static int kControlJoystick = 3;
	const static int kJoystickChannel4 = 4;

	// Disable Sensor Switches (Generic USB Joystick)
	const int kDeployGrabber = 12;
	const int kDisableInfrared = 11;
	const int kDisableRightEnc = 10;
	const int kDisableLeftEnc = 9;
	const int kDisableStringPot = 8;
	const int kDisableLeftIntakeLS = 7;
	const int kDisableRightIntakeLS = 6;
	const int kDisableLiftTopLS = 5;
	const int kDisableLiftBottomLS = 4;
	const int kDisableExtra3 = 3;
	const int kDisableExtra2 = 2;
	const int kDisableExtra1 = 1;

	// Control Joystick inputs (UHID)
	const int kGrabberEngage = 1;
	const int kHookDeploy = 2;
	const int kBackBarDeploy = 3;
	const int kLiftLockEngage = 4;

	const int kLiftJoystickY = 4;  // Axis 4 Y
	const int kLiftJoystickX = 5;  // Axis 5 X
	const int kIntakeJoystickY = 1;  // Axis 1 Y
	const int kIntakeJoystickX = 0;  // Axis 0 X
	const int kSpeedPot = 2;  // Axis 2

#if talonSRXEncoders
#endif

#if roboRioEncoders
	// only used if we bring encoder inputs back to roboRIO
	const int kLeftEncoderAPin = 4;
	const int kLeftEncoderBPin = 5;
	const int kRightEncoderAPin = 6;
	const int kRightEncoderBPin = 7;

	// Quadrature Encoder related assignments
	const int kNumSamplesToAverage = 5;// Samples per round range 1-255
	const double kWheelRadius = 3.0;// Radius in inches
	const double kPulsesPerRevolution = 4096;// PPR of encoder
	const double k2Pi = 2.0 * 3.1415926;// 2 * Pi
	const double kDistancePerPulse =
	1.0 / (kPulsesPerRevolution * (k2Pi * kWheelRadius));
	const double kMinRateNotStopped = 1.0;

#endif

	// Miscellaneous constants
	const float Kp = 0.03;
	const double kUpdatePeriod = 0.010;  // update period in seconds

	// Analog Sampling settings
	const int kOversampleBits = 4;
	const int kAverageBits = 4;

	// Joystick deadband settings
	const float kBottomOfDeadBand = -0.1;
	const float kTopOfDeadBand = 0.1;

	// Analog scaling factor
	const double kVoltsPerAnalogDivision = 5.0 / 4096.0;

	// Height of the elevator
	double dLiftHeight = 0.0;

	//***********************************************************************
	// Miscellaneous Globals
	int autoLoopCounter;
	float gyroAngle;
	//***********************************************************************

	// Which Controls box are we dealing with?
#if CNTL_BOX_A
	WPI_VictorSPX *m_frontLeft = new WPI_VictorSPX(frontLeftChannel);
	WPI_VictorSPX *m_frontRight = new WPI_VictorSPX(frontRightChannel);

#if ON_ROBOT
	WPI_TalonSRX *m_rearLeft = new WPI_TalonSRX(rearLeftChannel);
	WPI_TalonSRX *m_rearRight = new WPI_TalonSRX(rearRightChannel);
#endif

	WPI_VictorSPX *m_rightIntake = new WPI_VictorSPX(kRightIntake);
	WPI_VictorSPX *m_leftIntake = new WPI_VictorSPX(kLeftIntake);
	WPI_VictorSPX *m_spareMC = new WPI_VictorSPX(kSpareMC);
	WPI_VictorSPX *m_rightLift = new WPI_VictorSPX(kRightLift);
	WPI_VictorSPX *m_leftLift = new WPI_VictorSPX(kLeftLift);

	// Lift SpeedControllerGroup
	SpeedControllerGroup m_lift { *m_leftLift, *m_rightLift };


	// Compressor channel -- not really needed but we'll keep it for completeness
	Compressor roboCompressor {kPcmCanAddress};

#endif

	// Object for dealing with the Power Distribution Panel (PDP).
	PowerDistributionPanel m_pdp { kPdpCanAddress };

#if CNTL_BOX_B
	WPI_VictorSPX *m_frontLeft = new WPI_VictorSPX(frontLeftChannel);
	WPI_VictorSPX *m_frontRight = new WPI_VictorSPX(frontRightChannel);

#if ON_ROBOT
	WPI_TalonSRX *m_rearLeft = new WPI_TalonSRX(rearLeftChannel);
	WPI_TalonSRX *m_rearRight = new WPI_TalonSRX(rearRightChannel);
#endif

	WPI_TalonSRX *m_rightIntake = new WPI_TalonSRX(kRightIntake);
	WPI_TalonSRX *m_leftIntake = new WPI_TalonSRX(kLeftIntake);
	WPI_TalonSRX *m_spareMC = new WPI_TalonSRX(kSpareMC);
	WPI_TalonSRX *m_rightLift = new WPI_TalonSRX(kRightLift);
	WPI_TalonSRX *m_leftLift = new WPI_TalonSRX(kLeftLift);

	// Compressor channel -- not really needed but we'll keep it for completeness
	Compressor roboCompressor { kPcmCanAddress };
#endif

	// Declare speed controller groups if we're on the robot
#if ON_ROBOT
	// Mobility SpeedControllerGroups
	SpeedControllerGroup m_left { *m_frontLeft, *m_rearLeft };
	SpeedControllerGroup m_right { *m_frontRight, *m_rearRight };

#else
	SpeedControllerGroup m_left {*m_frontLeft};
	SpeedControllerGroup m_right {*m_frontRight};
#endif

	// Tell WPIlib that we're Tank drive
	DifferentialDrive m_robotDrive { m_left, m_right };

#if XBOX_CONTROLLER
	// Joysticks
	XboxController m_stick {kLeftDriverJoystick};
#endif

#if DUAL_JOYSTICKS
	// Joysticks
	Joystick m_stickL { kLeftDriverJoystick };
	Joystick m_stickR { kRightDriverJoystick };
#endif

#if OI_ENABLED
	Joystick m_sensorBypassStick { kDisableSwitchJoystick };
	Joystick m_controlStick { kControlJoystick };
#endif

	// Instantiate the Analog Inputs
	AnalogInput m_autoSwitch1 { kRotarySw1Channel };
	AnalogInput m_autoSwitch2 { kRotarySw2Channel };
	AnalogInput m_liftHeight { kLiftHightChannel };
	AnalogInput m_infraredDistance { kInfraredChannel };

	// Instantiate the Digital IOs
	DigitalInput m_leftIntakeLimitSwitch { kLeftIntakeLimitSw };
	DigitalInput m_rightIntakeLimitSwitch { kRightIntakeLimitSw };
	DigitalInput m_atLiftTop { kLiftTopLimitSw };
	DigitalInput m_atLiftBottom { kLiftBottomLimitSw };

	// Pneumatics
	DoubleSolenoid backBar { kPcmCanAddress, backBarIn, backBarOut };
	DoubleSolenoid liftLocker { kPcmCanAddress, liftLock, liftUnlock };
	Solenoid gripper { kPcmCanAddress, gripCube };
	Solenoid intakeDeploy { kPcmCanAddress, deployIntake };
	Solenoid engageHook { kPcmCanAddress, hook };

#if talonSRXEncoders
#endif

#if roboRioEncoders
	// Quadrature Encoders on RoboRio
	Encoder m_rightEncoder {kLeftEncoderAPin, kLeftEncoderBPin, false, Encoder::k4X};
	Encoder m_leftEncoder {kRightEncoderAPin, kRightEncoderBPin, false, Encoder::k4X};
#endif

#if USE_NAVX
	// Declare the NAVX
	AHRS *ahrs;

	// Network Tables Crap used by NAVX
	std::shared_ptr<NetworkTable> table {NULL};

#endif

	frc::LiveWindow& m_lw = *LiveWindow::GetInstance();
	frc::SendableChooser<std::string> m_chooser;
	const std::string kAutoNameDoNothing = "Do Nothing";
	const std::string kAutoCrossAutoLine = "Cross Auto Line";
	const std::string kAutoSwitchCube = "Place Cube on Switch";
	const std::string kAutoScaleCube = "Place Cube on Scale";
	const std::string kAutoSecondCube = "Acquire Second Cube";

	std::string m_autoSelected;
};

START_ROBOT_CLASS(Robot)
