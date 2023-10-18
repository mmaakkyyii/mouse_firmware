
#define IO_OUT	(1)				//PFC��Input/Output ���W�X�^��1��ݒ肷��Əo�͂ɂȂ�
#define IO_IN	(0)				//PFC��Input/Output ���W�X�^��0��ݒ肷��Ɠ��͂ɂȂ�

//47k 10k�ŕ��� -> Vadc=0.1754Vin ->ADC=2^12*Vadc/3.3
#define BATT_MAX	1742		//4.0*2=8.0V
#define BATT_MIN		1524		//3,5*2=7.0V 

#define SW_OFF	(1)				//�X�C�b�`��OFF�̎��Ɏ��l
#define SW_ON	(0)				//�X�C�b�`��ON�̎��Ɏ��l

#define CHATTERING_WAIT	(50)	//�`���^�����O���p�҂�����

#define INC_FREQ	(2000)		//���[�h�������������ɖ炷���̎��g��
#define DEC_FREQ	(1000)		//���[�h�������������ɖ炷���̎��g��

#define MOT_FORWARD	(0)			//���[�^�h���C�o��CWCCW�[�q��LOW���o�͂���ƑO�i����
#define MOT_BACK	(1)			//���[�^�h���C�o��CWCCW�[�q��HIGH���o�͂���ƃo�b�N����

#define MIN_SPEED	(30)		//�Œᑬ�x.�W�F�l�������W�X�^��16bit�ł��邱�ƂƁAMTU�̓�����g�����狁�߂���l�������悻18mm/s�Ȃ̂ŁA�]�T��������30mm/s

#define PI (3.141592653589793)	//�~����


#define HALF_SECTION	(90)	//�����̋���
#define SECTION		(180)		//����̋���

#define MAZESIZE_X	(16)		//���H�̑傫��(MAZESIZE_X * MAZESIZE_Y)���H
#define MAZESIZE_Y	(16)		//���H�̑傫��(MAZESIZE_X * MAZESIZE_Y)���H

#define UNKNOWN	2				//�ǂ����邩�Ȃ�������Ȃ���Ԃ̏ꍇ�̒l
#define NOWALL	0				//�ǂ��Ȃ��΂����̒l
#define WALL	1				//�ǂ�����ꍇ�̒l
#define VWALL	3				//���z�ǂ̒l(���g�p)

#define MASK_SEARCH	0x01		//�T�����s�p�}�X�N�l.�Ǐ��Ƃ��̒l��AND�l���O�iNOWALL�j�Ȃ�ǂȂ�or���T�����
#define MASK_SECOND	0x03		//�ŒZ���s�p�}�X�N�l.�Ǐ��Ƃ��̒l��AND�l���O�iNOWALL�j�Ȃ�ǂȂ�

