#ifndef REGULATOR_H
#define REGULATOR_H

#define KP						2000.0f
#define SPEED_MIN				200
#define ROTATION_THRESHOLD		5 //20
#define BACK_WALL_STOP			40 //80
#define SIDE_STOP				300//A MODIFIER
#define ROTATION_COEFF			2.0f//0.5f 		KD

//start the regulator thread
void regulator_start(void);

#endif /* REGULATOR_H */
