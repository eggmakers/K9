#pragma once

#include "Basic.hpp"
#include "vector3.hpp"
#include "vector2.hpp"

struct AvoidanceCfg
{
	// �������(cm)
	float wheelbase[2];
	// ���ϲ���
	// bit0:����ʹ��
	// bit1:ʹ�����±���(�ٽ��������)
	// bit8:����������������
	// bit9:����������������
	// bit10:����������������
	// bit11:����������������
	// bit16:����������������
	// bit17:����������������
	// bit18:����������������
	// bit19:����������������
	uint32_t AvoidanceMode[2];
	// ���Ͼ��루���ϰ������cm��
	float AvoidanceDist[2];

	// Χ������
	uint32_t fenceEnable[2];
	// Χ������
	uint32_t fenceType[2];
	// Χ������
	uint32_t fenceAction[2];
	// Բ��Χ���뾶
	float fenceRadius[2];
	// �����С�߶�
	float fenceMinAlt[2];
	float fenceMaxAlt[2];

} __PACKED;

#define AvModeFlag_Ena (1 << 0)
#define AvModeFlag_DownsideAv_Ena (1 << 1)
#define AvModeFlag_Fence_Ena (1 << 3)
#define AvModeFlag_UpsideAcav_Ena (1 << 8)
#define AvModeFlag_LeftsideAcav_Ena (1 << 9)
#define AvModeFlag_RightsideAcav_Ena (1 << 10)
#define AvModeFlag_DownsideAcav_Ena (1 << 11)

#define AvMode_Enabled(x) (x & AvModeFlag_Ena)

// ��Χ��ʹ��(�뾶�͸߶�)
#define FenceEnable_SplFenceFlag (1 << 0)
// ����Χ��ʹ��(����վ���ĸ���Χ��)
#define FenceEnable_CpxFenceFlag (1 << 1)

/*����*/
enum AvTargetType
{
	// ��ά��Ŀ��
	AvTargetType_3dPoint = 0,
	AvTargetType_2dPoint,

	// ˮƽֱ��(��ֱ���޳�ֱ��ǽ��Ŀ��)(xy+zб��)
	AvTargetType_XYStraightLine,
	// ˮƽ�߶�(��ֱֱ�߶�ǽ��Ŀ��)(pos:xy+pos2:xy)
	AvTargetType_XYLineSegment,
	// ��ά�߶�(pos:xyz+pos2:xyz)
	AvTargetType_3dLineSegment,

	// Zƽ��(���޴�Zƽ��Ŀ��)(zΪǽ��z����)
	AvTargetType_ZSurface,
};
struct AvoidanceTarget
{
	// �Ƿ�ע��
	bool registered;
	// �Ƿ����
	bool available;
	// Ŀ������
	AvTargetType type;

	// Ŀ��λ��
	vector3<double> pos;
	vector3<double> pos2;
	// ����ʱ��
	TIME last_update_TIME;
};
#define max_AvTargets 8

// ��ȡ����Ŀ�����
uint8_t getAvTargetsCount();

// ע�����Ŀ��
// ���أ�-1�޷�ע�� >=0����Ŀ��id
int8_t registere_AvTarget(double TIMEOUT = -1);
// ע������Ŀ��
int8_t unregistere_AvTarget(uint8_t id, double TIMEOUT = -1);

// ��ȡ����Ŀ����Ϣ
bool get_AvTarget(uint8_t id, AvoidanceTarget *resTarget, double TIMEOUT = -1);

// ������Ŀ����Ϊ������
bool set_AvTargetInavailable(uint8_t id, double TIMEOUT = -1);

/*��ά��Ŀ��*/
// ���ñ���Ŀ�꣨��Ե�ǰENUλ�ã�
bool set_AvTarget3dPoint_RelativeEnu(uint8_t id, vector3<double> dis, double TIMEOUT = -1);
// ���ñ���Ŀ�꣨��Ե�ǰFLUλ�ã�
bool set_AvTargetPoint3dPoint_RelativeFlu(uint8_t id, vector3<double> dis, double TIMEOUT = -1);
/*��ά��Ŀ��*/

/*XYǽ��*/
bool set_AvTargetXYStraightLine_RelativeEnu(uint8_t id, vector2<double> dis, double angle, double TIMEOUT = -1);
bool set_AvTargetXYStraightLine_RelativeFlu(uint8_t id, vector2<double> dis, double angle = 0, double TIMEOUT = -1);
/*XYǽ��*/

/*��ȡ�����ٶȷ���������ϰ�����Ϣ���ٶ�ENUϵ��
	���أ�trueǰ�����ϰ��� falseǰ�����ϰ���
	resAvDistance�����ص�ǰ���ϰ������(cm)
		���鳤�ȱ������offsetsCount+1
		0����ƫ�Ʊ��ϼ�����
		1-offsetsCount����ǰλ��ƫ��ΪtagetOffsets[i]�ı��ϼ�����
	targetVel���ٶȷ���(ǰ������)
	inRange�����ϱ���ֱ��(�ɻ���Сcm)
	tagetOffsets����ǰλ��ƫ������ ƫ�Ʊ��봹ֱ���ٶȷ��� ˮƽ���ٶȷ���ķ�������ȥ��
	offsetsCount��λ��ƫ�����鳤��
*/
bool get_AvLineDistanceEnu(double *resAvDistance, vector3<double> targetVel, double inRange, const vector3<double> *posOffsets = 0, uint8_t offsetsCount = 0, double TIMEOUT = -1);
// ��ȡ�����ٶȷ���������ϰ�����Ϣ���ٶ�FLUϵ��
bool get_AvLineDistanceFlu(double *resAvDistance, vector3<double> targetVel, double inRange, const vector3<double> *posOffsets = 0, uint8_t offsetsCount = 0, double TIMEOUT = -1);
/*����*/

/*Χ��*/
enum FRS
{
	// ���ڿɷ���������
	FRS_NotInFlyZone = -1,
	// �ڽ�������
	FRS_NotOuNFlyZone = -2,
	// ��Χ��
	FRS_NoFence = 0,
	// �ڿɷ���������
	FRS_InFlyZone = 1,
};
/*����ɻ���ǰ�Ƿ���Χ����
	����ֵ:
		true:��Χ����Χ�ڣ�distance����Ϊ����Χ���߽���룬����Ϊ��Χ����Ϣ
		false:����Χ����Χ�ڣ�distance����ʱ��ʾ�ٶȷ���ָ��ǰΧ���ڲ�������ʱ��ʾָ��Χ����Χ�ڲ������ڲ��߽�ľ���
	pos:�ɻ���ǰλ��(cm)
	targetVel���ٶȷ���
	distance�����صľ���߽����(cm)
	fenceRs�����صı߽���
*/
bool is_insideFence(const vector3<double> &pos, const vector3<double> &targetVel = vector3<double>(0, 0, 0), double *distance = 0, FRS *fenceRs = 0);

/*����ɻ���ǰ�Ƿ���Բ��Χ����
	����ֵ:
		true:��Χ����Χ�ڣ�distance����Ϊ����Χ���߽���룬����Ϊ��Χ����Ϣ
		false:����Χ����Χ�ڣ�distance����ʱ��ʾ�ٶȷ���ָ��ǰΧ���ڲ�������ʱ��ʾָ��Χ����Χ�ڲ������ڲ��߽�ľ���
	pos:�ɻ���ǰλ��(cm)
	targetVel���ٶȷ���
	circleO_x:Բ��x   circleO_y: Բ��y   circleO_r: Բ�뾶
	distance�����صľ���߽����(cm)
*/
bool is_insideCircleFence(const vector3<double> &pos, const vector3<double> &targetVel,
						  double circleO_x, double circleO_y, double circle_r,
						  double *distance);
/*Χ��*/