/* Header to define Drive Calibration Constants */

#define DRVSYS_DRIVE_6
//#define DRVSYS_DRIVE_5
//#define DRVSYS_DRIVE_4
//#define DRVSYS_DRIVE_3
//#define DRVSYS_DRIVE_2
//#define DRVSYS_DRIVE_1
//#define DRVSYS_DRIVE_0


#ifdef DRVSYS_DRIVE_6
#define FOC_EMPIRIC_PHASE_ANGLE_OFFSET 15710//15715 //
#endif

#ifdef DRVSYS_DRIVE_5
#define FOC_EMPIRIC_PHASE_ANGLE_OFFSET 0//15715 //
#endif

#ifdef DRVSYS_DRIVE_4
#define FOC_EMPIRIC_PHASE_ANGLE_OFFSET 0//15715 //
#endif

#ifdef DRVSYS_DRIVE_3
#define FOC_EMPIRIC_PHASE_ANGLE_OFFSET 0//15715 //
#endif

#ifdef DRVSYS_DRIVE_2
#define FOC_EMPIRIC_PHASE_ANGLE_OFFSET 0//15715 //
#endif

#ifdef DRVSYS_DRIVE_1
#define FOC_EMPIRIC_PHASE_ANGLE_OFFSET 0//15715 //
#endif

#ifdef DRVSYS_DRIVE_0
#define FOC_EMPIRIC_PHASE_ANGLE_OFFSET 0//15715 //
#endif