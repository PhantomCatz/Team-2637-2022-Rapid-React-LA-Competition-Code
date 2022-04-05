package frc.robot;

import java.util.ArrayList;

import javax.lang.model.util.ElementScanner6;

import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoMode.PixelFormat;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;

import com.kauailabs.navx.frc.AHRS;

import frc.DataLogger.*;
import frc.Autonomous.CatzAutonomous;
import frc.Mechanisms.CatzIntake;
import frc.Mechanisms.CatzClimber;
import frc.Mechanisms.CatzDriveTrain;
import frc.Mechanisms.CatzShooter;
import frc.Mechanisms.CatzShydexer;
import frc.Mechanisms.CatzTurret;
import frc.Mechanisms.CatzVision;
import frc.Mechanisms.CatzYdexer;
import frc.Mechanisms.LiDAR;
import frc.Mechanisms.PathSelection;



/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot 
{
  public static CatzDriveTrain driveTrain;
  public static CatzAutonomous auton;
  public static CatzIntake     intake;
  public static CatzShooter    shooter;
  public static CatzTurret     turret;
  public static CatzVision     vision;
  public static CatzYdexer     ydexer;
  public static CatzShydexer   shydexer;
  public static DataCollection dataCollection;
  public static LiDAR          lidar;
  public static CatzClimber    climb;
  public static CatzLog        catzLog;
  public static CatzConstants  constants;
  public static PathSelection  pathSelection;

  public static AHRS navx;

  public static XboxController xboxDrv;
  public static XboxController xboxAux;

  private final int XBOX_DRV_PORT = 0;
  private final int XBOX_AUX_PORT = 1;
  private final int DPAD_UP = 0;
  private final int DPAD_DN = 180;
  private final int DPAD_LT = 270;
  private final int DPAD_RT = 90;

  private double elevatorDnInput = 0.0;
  private double elevatorUpInput = 0.0;

  private double  extendRotatingArmPwrInput = 0.0;
  private boolean extendRotatingArmLT = false;
  private boolean extendRotatingArmRT = false;

  private double deployRotatingArmInput = 0.0;

  public static PowerDistribution pdp;
  public static Timer dataCollectionTimer;


  public ArrayList<CatzLog> dataArrayList;

  private UsbCamera driverCamera;

  private static int cameraResolutionWidth = 240;
  private static int cameraResolutionHeight = 160;
  private static int cameraFPS = 30;

  public double drvTrainPwrFactor = 1.0;
  

  double left;
  double right;

  boolean firstTele = true;
  int tele = 0;

  int hoodCounter = 0;
  int turretCounter = 0;
  
  private final int LED_HOODUP_DIO_PORT = 5;
  private final int LED_VISION_AIM_HUB_DIO_PORT = 6;
  private final int LED_NOT_ABLE_SHOOT_DIO_PORT = 7;
  private final int LED_IS_CLIMBING_DIO_PORT = 8;

  DigitalOutput LEDHoodUp;
  DigitalOutput LEDVisionAimHub;
  DigitalOutput LEDableShooting;
  DigitalOutput LEDIsClimbing;

  boolean hoodUp;
  boolean visionAimHub;
  boolean ableShooting;
  boolean isClimbing;
  
  double distance = 0.0;

  boolean firstDeployRoatatingArm = true;

  final int HOOD_COUNTER_SET_HOOD = 45;
  final int HOOD_COUNTER_DEPLOY_ARMS = HOOD_COUNTER_SET_HOOD + 10;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() 
  {  
    xboxDrv = new XboxController(XBOX_DRV_PORT);
    xboxAux = new XboxController(XBOX_AUX_PORT);

    pdp  = new PowerDistribution();
    navx = new AHRS(Port.kMXP, (byte) 200);
    navx.reset();

    dataCollectionTimer = new Timer();
    dataCollectionTimer.start();

    dataCollection = new DataCollection();

    dataArrayList = new ArrayList<CatzLog>();

    dataCollection.dataCollectionInit(dataArrayList);

    driveTrain = new CatzDriveTrain();
    intake     = new CatzIntake();
    auton      = new CatzAutonomous();
    shooter    = new CatzShooter();
    vision     = new CatzVision();
    turret     = new CatzTurret();
    climb      = new CatzClimber();
    lidar      = new LiDAR();
    shydexer   = new CatzShydexer();
    ydexer     = new CatzYdexer();
    constants  = new CatzConstants();
    pathSelection = new PathSelection();
    
    /*driverCamera = CameraServer.startAutomaticCapture(0);
    driverCamera.setFPS(cameraFPS);
    driverCamera.setResolution(cameraResolutionWidth, cameraResolutionHeight);
    driverCamera.setPixelFormat(PixelFormat.kMJPEG);

    driverCamera.setConnectionStrategy(ConnectionStrategy.kKeepOpen);*/
    
    LEDHoodUp = new DigitalOutput(LED_HOODUP_DIO_PORT);
    LEDVisionAimHub = new DigitalOutput(LED_VISION_AIM_HUB_DIO_PORT);
    LEDableShooting = new DigitalOutput(LED_NOT_ABLE_SHOOT_DIO_PORT);
    LEDIsClimbing = new DigitalOutput(LED_IS_CLIMBING_DIO_PORT);
    
    /*driverCamera = CameraServer.startAutomaticCapture(0);
    driverCamera.setFPS(cameraFPS);
    driverCamera.setResolution(cameraResolutionWidth, cameraResolutionHeight);
    driverCamera.setPixelFormat(PixelFormat.kMJPEG);

    driverCamera.setConnectionStrategy(ConnectionStrategy.kKeepOpen);*/

    pathSelection.initializePathOptions();

  } // End of robotInit()

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and
   * test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() 
  {

    lidar.smartDashboardLiDAR();
    shooter.smartDashboardShooter();
    turret.smartDashboardTurret();
    ydexer.smartDashboardYdexer();
    climb.smartDashboardClimb();
    vision.smartDashboardVision();
    //shooter.smartDashboardShooter_DEBUG();

    
    if(!(isDisabled()))
    {
      hoodUp = (shooter.getHoodPosition() == 1); //named variable is private, lol
      visionAimHub = (Math.abs(vision.getXErrorOffset()) <= 2.0)&& vision.hasValidTarget();
      distance = Robot.vision.getDistanceToTarget();
      isClimbing = (hoodCounter > 0);
      if ((distance <= 70.0 && distance > 30.0) && !isClimbing || (distance > 100.0 && distance <= 400) && !isClimbing)
      {
        ableShooting = true;
      }
      else
      {
        ableShooting = false;
      }
    }
    else
    {
      ableShooting = true;
      isClimbing = true;
      hoodUp =false;
      visionAimHub = false;
    }
    
  
    LEDHoodUp.set(hoodUp);
    LEDVisionAimHub.set(visionAimHub);
    LEDableShooting.set(ableShooting);
    LEDIsClimbing.set(isClimbing);

    //Robot.auton.smartDashBoardAutonomous_DEBUG();
    //shooter.smartDashboardShooter_DEBUG();
    
  }



  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different
   * autonomous modes using the dashboard. The sendable chooser code works with
   * the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the
   * chooser code and
   * uncomment the getString line to get the auto name from the text box below the
   * Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure
   * below with additional strings. If using the SendableChooser make sure to add
   * them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() 
  {
    turret.resetTurretEncoder();
    shooter.inAutonomous = true;
    driveTrain.setToBrakeMode();
    pathSelection.determinePath();

    turret.setTurretBrakeMode();
    turret.resetTurretEncoder();

    dataCollectionTimer.reset();
    dataCollectionTimer.start();
    dataCollection.setLogDataID(dataCollection.LOG_ID_DRV_TRAIN);
    dataCollection.startDataCollection();
    
  }

  /* This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() 
  {
    ableShooting = false;
  }



  /* This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() 
  {
    //turret.resetTurretEncoder();

    shooter.inAutonomous = false;
      
    if(firstTele == true)
    {
      driveTrain.instantiateDifferentialDrive();
      firstTele = false;
    }
    driveTrain.setToBrakeMode();
    turret.setTurretBrakeMode();

    dataCollectionTimer.reset();
    dataCollectionTimer.start();
    dataCollection.setLogDataID(dataCollection.LOG_ID_CLIMB);
    dataCollection.startDataCollection();

    firstDeployRoatatingArm = true;
      
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() 
  {
    ableShooting = false;
    
    //---------------vision----------------// 
    
    vision.turretTracking();

    //--------------drivetrain------------//
    if(xboxDrv.getAButtonPressed())
    {
      driveTrain.inSlewRate = true;
    }
    else if(xboxDrv.getBButtonPressed())
    {
      driveTrain.inSlewRate = false;
    }


    if(xboxAux.getRightBumper())
    {
      driveTrain.arcadeDrive((xboxAux.getLeftY()), xboxAux.getRightX() );
    }
    else
    {
      left  = xboxDrv.getLeftY();
      right = xboxDrv.getRightX();
      
      driveTrain.arcadeDrive(left,right);      
    } 

    if (xboxDrv.getLeftBumper()) 
    {
      driveTrain.shiftToHighGear();
    } 
    else if (xboxDrv.getRightBumper()) 
    {
      driveTrain.shiftToLowGear();
    }

    
    
    /*--------------------------------------------------------------------------------
    *  Intake Control
    *-------------------------------------------------------------------------------*/
    if (xboxDrv.getLeftTriggerAxis() > 0.2) 
    {
      if(Robot.intake.intakeStowed == false)
      {
        intake.intakeRollerOut();    
      }
      intake.intakeRollerOut = true;
       
    }
    else if (xboxDrv.getRightTriggerAxis() > 0.2)
    {
      if(Robot.intake.intakeStowed == false)
      {
        intake.intakeRollerIn();  
      }
      intake.intakeRollerOn = true;
    }
    else
    {
      intake.intakeRollerOff();
    }

    if (xboxDrv.getLeftStickButton())
    {
      intake.deployIntake(); 
    }
    else if (xboxDrv.getRightStickButton())
    {
      intake.stowIntake();
    }

    /*--------------------------------------------------------------------------------
    *  ShyDexer Control
    *-------------------------------------------------------------------------------*/
    if(ydexer.yDexerOn || intake.intakeRollerOn) //
    {
      shydexer.shyDexerOn();
    }
    else if(intake.intakeRollerOut)
    {
      shydexer.shyDexerReverse();
    }
    else
    {
      shydexer.shyDexerOff();
    }

    /*--------------------------------------------------------------------------------
    *  Turret Control
    *-------------------------------------------------------------------------------*/
    if ((xboxAux.getRightX() >=  0.2 && xboxAux.getRightBumper() == false) ||
        (xboxAux.getRightX() <= -0.2 && xboxAux.getRightBumper() == false)) 
    {
      turret.turretState = turret.TURRET_STATE_MANUAL;
      if (xboxAux.getXButton() == true) 
      {
        turret.rotateTurret(xboxAux.getRightX() * turret.TURRET_LOW_RATE);
      } 
      else 
      {
        turret.rotateTurret(xboxAux.getRightX() * turret.TURRET_HIGH_RATE);
      }
    } 
    else if (turret.turretState != turret.TURRET_STATE_MOVE_TO_POSITION) 
    {
      turret.stopTurret();
    }
    
    if (xboxAux.getYButton()) 
    {
      turret.autoAimTurret();
    }


    /*--------------------------------------------------------------------------------
    *  Shooter Control
    *-------------------------------------------------------------------------------*/
    if (xboxAux.getBButtonPressed()) 
    {
      shooter.shoot();
    }
    else if (xboxAux.getStartButton()) 
    {
      shooter.shooterOff();
    } 

    if (xboxAux.getPOV() == DPAD_DN) 
    {                             
      shooter.setTargetRPM(shooter.SHOOTER_TARGET_RPM_FENDER);
    } 
    else if (xboxAux.getPOV() == DPAD_UP) 
    {                             
      shooter.setTargetRPM(shooter.SHOOTER_TARGET_RPM_VARIABLE_SHOOTER);
    } 

    if (xboxAux.getPOV() == DPAD_RT) 
    {
      shooter.hoodSetTop();
    }
    else if (xboxAux.getPOV() == DPAD_LT)
    { 
      shooter.hoodSetBot(); 
    }
    
    /*--------------------------------------------------------------------------------
    *  YDexer Control
    *-------------------------------------------------------------------------------*/
    if(xboxDrv.getYButtonPressed())
    {
      lidar.manualOveride = true;
    }  

    /*--------------------------------------------------------------------------------
    *  Climb Control
    *-------------------------------------------------------------------------------*/
     if(xboxAux.getRightBumper())
     {
       

       /*----------------------------------------------------------------------------------------------
       *  In Climb mode
       *---------------------------------------------------------------------------------------------*/
       elevatorDnInput               = xboxAux.getRightTriggerAxis();
       elevatorUpInput               = xboxAux.getLeftTriggerAxis();
 
       extendRotatingArmPwrInput     = xboxDrv.getLeftY();
       extendRotatingArmLT           = xboxDrv.getXButton();
       extendRotatingArmRT           = xboxDrv.getBButton();
 
       deployRotatingArmInput        = xboxDrv.getRightY();

       
 
       /*----------------------------------------------------------------------------------------------
       *  Check elevator controls
       *---------------------------------------------------------------------------------------------*/
       if(Math.abs(elevatorDnInput) > 0.15)
       {
         climb.climbElevatorManualCntrl(elevatorDnInput); 
       }
       else if(Math.abs(elevatorUpInput) > 0.15)
       {
         climb.climbElevatorManualCntrl(-elevatorUpInput); 
       }
       else
       {
         climb.elevatorMtrPwrOff();
         climb.climbElevatorManualCntrl(0.0);
       }
     
  
       /*----------------------------------------------------------------------------------------------
       *  Check outer arm extension control
       *---------------------------------------------------------------------------------------------*/
       if(Math.abs(extendRotatingArmPwrInput) > 0.15)
       {
         if(extendRotatingArmLT == true)
         {
           climb.climbExtendRotatingArmManualCtrlRT(extendRotatingArmPwrInput);
         }
         else if(extendRotatingArmRT == true)
         {
           climb.climbExtendRotatingArmManualCtrlLT(extendRotatingArmPwrInput);
         }
         else 
         {
           climb.climbExtendRotatingArmManualCtrlBOTH(extendRotatingArmPwrInput);
         }
       } 
       else
       {
         climb.climbExtendRotatingArmManualCtrlBOTH(0.0);
       }
     
       /*----------------------------------------------------------------------------------------------
       *  check rotating arm deploy controls
       *---------------------------------------------------------------------------------------------*/
      /*if(hoodCounter >= 100)
       {
         if(shooter.getHoodPosition() ==  shooter.HOOD_BOT_POS)
         {
           shooter.hoodSetTop();
           hoodCounter = 0;
         }
         else
         {
          hoodCounter = 0;
         }
       }
       hoodCounter++;*/

       if(Math.abs(deployRotatingArmInput) > 0.2)
       {
        shooter.hoodSetTop();
        if(hoodCounter >= 5) 
        {
          climb.climbRotatingArmDeploy();
        }
        /*if(firstDeployRoatatingArm)
        {
          turret.setTargetPos(93.0);
          firstDeployRoatatingArm = false;
        }

        if(turret.getTurretPositionDeg() >= 88)
        {

        }

        if(hoodCounter >= HOOD_COUNTER_SET_HOOD) 
        {
          shooter.hoodSetTop();
          if(hoodCounter >= HOOD_COUNTER_DEPLOY_ARMS) 
          {
            //climb.climbRotatingArmDeploy();
          }
        }*/
  
        hoodCounter++;  
        
      }
      else if(Math.abs(deployRotatingArmInput) < 0.2)
      {
        climb.climbRotatingArmStow(); 
      }
       
     } 
     else
     {
       //normal operation, not climbing
       
       climb.climbElevatorManualCntrl(0.0);
       climb.climbExtendRotatingArmManualCtrlBOTH(0.0);
       hoodCounter = 0;
     }
  
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() 
  {
    if(dataCollection.logDataValues == true)
    {
      dataCollection.stopDataCollection();
      driveTrain.setToCoastMode();

      try 
      {
        dataCollection.exportData(dataArrayList);
      } 
      catch (Exception e) 
      {
        e.printStackTrace();
      }
    }
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    
  ableShooting = true;
  }
  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

}
