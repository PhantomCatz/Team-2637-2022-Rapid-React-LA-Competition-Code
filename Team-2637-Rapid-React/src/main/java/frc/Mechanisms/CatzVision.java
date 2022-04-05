package frc.Mechanisms;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.*;

public class CatzVision 
{
    public boolean hasValidTarget;
    public boolean inShootingRange;

    public double xErrorOffsetDeg;      //Positive value = target is to the right, Negative value = target is to the left
    public double yErrorOffsetDeg;
    public double targetPresent;

    public final double HOOD_BTM_Y_POS = 0.0; //TBD
    public final double HOOD_TOP_Y_POS = 0.0; //TBD
    public final double HOOD_Y_OFFSET  = 1.0; //TBD

    public final double HOOD_TOP_MIN_POS = HOOD_TOP_Y_POS - HOOD_Y_OFFSET;
    public final double HOOD_TOP_MAX_POS = HOOD_TOP_Y_POS + HOOD_Y_OFFSET;
    public final double HOOD_BTM_MIN_POS = HOOD_BTM_Y_POS - HOOD_Y_OFFSET;
    public final double HOOD_BTM_MAX_POS = HOOD_BTM_Y_POS + HOOD_Y_OFFSET;

    private final double LIMELIGHT_BTM_MOUNT_HEIGHT  = 36.0;
    private final double LIMELIGHT_TOP_MOUNT_HEIGHT  = 39.5;

    private final double LIMELIGHT_TOP_MOUNT_ANGLE = 30.0;
    private final double LIMELIGHT_BTM_MOUNT_ANGLE = 43.0;

    private final double HUB_TARGET_HEIGHT_TOP = 100.0;

    private double angleToTargetDeg;
    private double angleToTargetRad;
    private double distanceToTargetInch;

    private double LimelightMountAngle;
    private double LimelightMountHeight;

    public void turretTracking()
    {
        targetPresent    = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
        xErrorOffsetDeg  = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
        yErrorOffsetDeg  = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);

        if (targetPresent == 1.0)
        {
            hasValidTarget = true;

            if (Robot.shooter.getHoodPosition() == Robot.shooter.HOOD_TOP_POS)
            {
                if (yErrorOffsetDeg >= HOOD_TOP_MIN_POS && yErrorOffsetDeg <= HOOD_TOP_MAX_POS)
                {
                    inShootingRange = true;
                }
                else
                {
                    inShootingRange = false;
                }
            }
            else
            {
                if (yErrorOffsetDeg >= HOOD_BTM_MIN_POS && yErrorOffsetDeg <= HOOD_BTM_MAX_POS)
                {
                    inShootingRange = true;
                }
                else
                {
                    inShootingRange = false;
                }
            }
        }
        else
        {
            hasValidTarget  = false;
            inShootingRange = false;
        }
        
        smartDashboardVision();
    }

    public double getDistanceToTarget()
    {
        
        yErrorOffsetDeg  = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);

        if (Robot.shooter.getHoodPosition() == Robot.shooter.HOOD_TOP_POS)
        {
           LimelightMountAngle = LIMELIGHT_TOP_MOUNT_ANGLE;
           LimelightMountHeight = LIMELIGHT_TOP_MOUNT_HEIGHT;
        }
        else if (Robot.shooter.getHoodPosition() == Robot.shooter.HOOD_BOT_POS)
        {
            LimelightMountAngle = LIMELIGHT_BTM_MOUNT_ANGLE;
            LimelightMountHeight = LIMELIGHT_BTM_MOUNT_HEIGHT;
        }

        angleToTargetDeg = LimelightMountAngle + yErrorOffsetDeg;
        angleToTargetRad = angleToTargetDeg * (Math.PI / 180); //Convert angle from degrees to radians
        distanceToTargetInch = (HUB_TARGET_HEIGHT_TOP - LimelightMountHeight) / Math.tan(angleToTargetRad);

        return distanceToTargetInch;
    }

    public double getXErrorOffset()
    {
        xErrorOffsetDeg  = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
        return xErrorOffsetDeg;
    }

    public double getYErrorOffset()
    {
        yErrorOffsetDeg  = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
        return yErrorOffsetDeg;
    }

    public void smartDashboardVision()
    {
        SmartDashboard.putBoolean("Has Valid Target", hasValidTarget);
        SmartDashboard.putNumber("X Offset", xErrorOffsetDeg);
        SmartDashboard.putNumber("Y Offset", yErrorOffsetDeg);
        SmartDashboard.putNumber("Distance To Target", getDistanceToTarget());
    }

    public boolean hasValidTarget()
    {
        return hasValidTarget;
    }
}
