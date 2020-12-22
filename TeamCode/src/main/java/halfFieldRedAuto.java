public class halfFieldRedAuto {
}
    robotDeclarations         robot   = new robotDeclarations();



    static final double COUNTS_PER_MOTOR_REV    = 1440 ;
static final double DRIVE_GEAR_REDUCTION    = 2.0 ;
static final double WHEEL_DIAMETER_INCHES   = 4.0 ;
static final double COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                              (WHEEL_DIAMETER_INCHES * 3.1415);
static final double  DRIVE_SPEED            = 0.6;
static final double  TURN_SPEED             = 0.5;