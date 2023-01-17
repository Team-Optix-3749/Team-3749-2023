package frc.robot.subsystems;

public class autobalencing {
    public static double kOffBalanceThresholdDegrees = 10.0f;
    public static double kOnBalanceThresholdDegrees  = 5.0f;

void OperatorControl()
{
    while (IsOperatorControl() && IsEnabled()) {

        // double xAxisRate          = stick.GetX();
        // double yAxisRate          = stick.GetY();
        double pitchAngleDegrees  = ahrs->GetPitch();
        double rollAngleDegrees   = ahrs->GetRoll();

        if ( !autoBalanceXMode &&
             (fabs(pitchAngleDegrees) >=
              fabs(kOffBalanceThresholdDegrees))) {
            autoBalanceXMode = true;
        }
        else if ( autoBalanceXMode &&
                  (fabs(pitchAngleDegrees) <=
                   fabs(kOnBalanceThresholdDegrees))) {
            autoBalanceXMode = false;
        }
        if ( !autoBalanceYMode &&
             (fabs(pitchAngleDegrees) >=
              fabs(kOffBalanceThresholdDegrees))) {
            autoBalanceYMode = true;
        }
        else if ( autoBalanceYMode &&
                  (fabs(pitchAngleDegrees) <=
                   fabs(kOnBalanceThresholdDegrees))) {
            autoBalanceYMode = false;
        }

        // Control drive system automatically,
        // driving in reverse direction of pitch/roll angle,
        // with a magnitude based upon the angle

        if ( autoBalanceXMode ) {
            double pitchAngleRadians = pitchAngleDegrees * (M_PI / 180.0);
            xAxisRate = sin(pitchAngleRadians) * -1;
        }
        if ( autoBalanceYMode ) {
            double rollAngleRadians = rollAngleDegrees * (M_PI / 180.0);
            yAxisRate = sin(rollAngleRadians) * -1;
        }
        // Use the joystick X axis for lateral movement, 
        // Y axis for forward movement, and Z axis for rotation.
        robotDrive.MecanumDrive_Cartesian(xAxisRate, yAxisRate,stick.GetZ());
        Wait(0.005); // wait 5ms to avoid hogging CPU cycles
    }
}
}
