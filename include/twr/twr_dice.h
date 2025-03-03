#ifndef TWR_DICE_H_
#define TWR_DICE_H_

enum twr_dice_face {
	TWR_DICE_FACE_UNKNOWN = 0,
	TWR_DICE_FACE_1 = 1,
	TWR_DICE_FACE_2 = 2,
	TWR_DICE_FACE_3 = 3,
	TWR_DICE_FACE_4 = 4,
	TWR_DICE_FACE_5 = 5,
	TWR_DICE_FACE_6 = 6
};

struct twr_dice {
	enum twr_dice_face _face;
	float _threshold;
};

void twr_dice_init(struct twr_dice *self, enum twr_dice_face start);
void twr_dice_set_threshold(struct twr_dice *self, float threshold);
void twr_dice_feed_vectors(struct twr_dice *self, float x_axis, float y_axis, float z_axis);
enum twr_dice_face twr_dice_get_face(struct twr_dice *self);

#endif /* TWR_DICE_H_ */