#include "stdbool.h"
#include "stdint.h"
#include "mathfun.h"
#include "can.h"
#include "main.h"
#include "dj.h"


CAN_RxHeaderTypeDef Rxmsg;
uint8_t Rxmessage[8]={0};
uint8_t Txmessage[8]={0};
DJmotor djmotor[USE_DJ_NUM];
void PID_init(PIDtype* pid, float p, float i,float d, int32_t set) {//PID???
	pid->kp = p;
	pid->ki = i;
	pid->kd = d;
	pid->set_value = set;
	pid->current_value = 0;
	pid->err_delta = 0;
	pid->err_last = 0;
	pid->err_previous = 0;
}

void djsetzero(djmotorpointer motor) {//????
	motor->status_flag.is_setzero = false;
	motor->now_value.angle = 0;
	motor->now_value.pulse_total = 0;
	motor->counter.pulse_lock = 0;
}

void djmotor_init(void) {//??????????
	djmotorparam motor2006param;// motor3508param//????
	djmotorcounter counter;//??????
	djmotorlimit limit;//???????
	djmotorstatus status;//?????
	djmotorerror error;//???????
	motor2006param.StdId = 0x200;
	motor2006param.IDE = CAN_ID_STD;
	motor2006param.RTR = CAN_RTR_DATA;
	motor2006param.DLC = 8;
	motor2006param.motor_reduction_ratio = M2006_RATIO;//????????
	motor2006param.institutional_reduction_ratio = 5.06f;//????????
	motor2006param.round_pulse_number = 8191;//?????????????
	motor2006param.current_limit = 4500;//???????

	//motor3508param.StdId = 0x200;
	//motor3508param.IDE = CAN_id_Stander;
	//motor3508param.RTR = CAN_RTR_DATA;
	//motor3508param.DLC = 8;
	//motor3508param.motor_reduction_ratio = M3508_RATIO;
	//motor3508param.institutional_reduction_ratio = 2;
	//motor3508param.round_pulse_number = 8191;
	//motor3508param.current_limit = 10000;

	counter.last_rxtime = 0;
	counter.zero_count = 0;
	counter.pulse_lock = 0;
	counter.max_distance = 0;
	counter.paw_lock_count = 0;

	limit.current_limit_flag = false;//????????,??????????
	limit.position_angle_limit_flag = true;//???????????????
	limit.rpm_limit_flag = false;//?????????
	limit.position_speed_limit_flag = true;//???????????????
	limit.is_loose_stuck = false;//?????????
	limit.max_angle_limit = 300;//?????????
	limit.min_angle_limit = -300;//?????????
	limit.position_max_angle_limit = 2000;//????????????????
	limit.zero_max_current_limit = 3000;///??????????????
	limit.zero_max_speed_limit = 1000;//??????????????

	status.arrive_flag = false;
	status.overtime_flag = false;//???????
	status.stuck_flag = false;//???????
	status.zero_flag = false;//???????
	status.is_setzero = true;//?????????

	error.release_whenstuck_flag = false;//?????????????
	error.stuck_detcet_flag = true;//???????????
	error.timeout_detect_flag = true;//???????????
	error.timeout_count = 0;//???????
	error.stuck_count = 0;//???????

	for (int i = 0; i < USE_DJ_NUM; i++) {
		djmotor[i].mode = dj_zero;//?????????
		djmotor[i].enable = false;//???????
		djmotor[i].begin = false;//???????
		djmotor[i].counter = counter;//??????
		djmotor[i].limit = limit;//???????
		djmotor[i].error = error;//???????
		djmotor[i].status_flag = status;//???????
		djmotor[i].now_value.pulse_total = 0;//????????
		djmotor[i].previous_value.pulse_read = 0;//???????????
	};

	for (uint8_t i = 0; i < USE_M2006_NUM; i++) {
		djmotor[i].ID = i + 1;//???????
		djmotor[i].mode = dj_position;//??????????
		djmotor[i].param = motor2006param;//?????2006
		PID_init(&djmotor[i].pos_PID, 1.3,0.2 , 0, 0);//?????PID?????
		PID_init(&djmotor[i].rpm_PID, 8, 0.5, 0, 0);//?????PID?????
	};

	//for (uint8_t i = 0; i < USE_M2006_NUM; i++) {
		//djmotor[i].ID = i + 1;
		//djmotor[i].mode = dj_position;
		//djmotor[i].param = motor3508param;
		//PID_init(&djmotor[i].pos_PID, 2, 0.08, 0, 0);
		//PID_init(&djmotor[i].rpm_PID, 8, 0.5, 0, 0);
    //};
}

int32_t PID_caculate_data(PIDtype* pid) {//????????:kp?*(??????-??????)+ki?*??????+kd?*(???????-2*???????+???????)
	pid->err = pid->set_value - pid->current_value;//???????=???-???
	pid->err_delta = pid->kp * (pid->err - pid->err_last) + pid->ki * pid->err + pid->kd * (pid->err - 2 * pid->err_last + pid->err_previous);
	pid->err_previous = pid->err_last;
	pid->err_last = pid->err;
	return pid->err_delta;
}

void djlockposition(djmotorpointer motor){
	motor->pos_PID.set_value = motor->counter.pulse_lock;
	motor->pos_PID.current_value = motor->now_value.pulse_total;
	
	motor->rpm_PID.set_value = PID_caculate_data(&(motor->pos_PID));
	motor->rpm_PID.current_value = motor->now_value.speed;

	motor->set_value.current_value += PID_caculate_data(&(motor->rpm_PID));
	PEAK(motor->set_value.current_value, 3000);
}

void djpositionmode(djmotorpointer motor) {
	motor->set_value.pulse_total = motor->set_value.angle * motor->param.institutional_reduction_ratio * motor->param.round_pulse_number * motor->param.motor_reduction_ratio * 0.0027777f;
	if (motor->limit.position_angle_limit_flag == true) {
		int32_t maxpulse = motor->limit.max_angle_limit * motor->param.institutional_reduction_ratio * motor->param.round_pulse_number * motor->param.motor_reduction_ratio * 0.0027777f;
		int32_t minpulse = motor->limit.min_angle_limit * motor->param.institutional_reduction_ratio * motor->param.round_pulse_number * motor->param.motor_reduction_ratio * 0.0027777f;
		if (motor->set_value.pulse_total > maxpulse) {
			motor->pos_PID.set_value = maxpulse;
		} 
		else if (motor->set_value.pulse_total < minpulse) {
			motor->pos_PID.set_value = minpulse;
		}
		else {
			motor->pos_PID.set_value = motor->set_value.pulse_total;
		}
	}
	else {
		motor->pos_PID.set_value = motor->set_value.pulse_total;
	}
	motor->pos_PID.current_value = motor->now_value.pulse_total;
	motor->rpm_PID.set_value = PID_caculate_data(&(motor->pos_PID));
	if (motor->limit.position_speed_limit_flag) {
		PEAK(motor->rpm_PID.set_value, motor->limit.position_max_angle_limit);
	}
	motor->rpm_PID.current_value = motor->now_value.speed;
	//motor->set_value.current_value += PID_caculate_data(&(motor->rpm_PID));
	motor->set_value.current_value += PID_caculate_data(&motor->rpm_PID);
}

void djspeedmode(djmotorpointer motor) {
	motor->rpm_PID.set_value = motor->set_value.speed;
	motor->rpm_PID.current_value = motor->now_value.speed;
	motor->set_value.current_value += PID_caculate_data(&motor->rpm_PID);
}

void djzeromode(djmotorpointer motor) {
	motor->rpm_PID.set_value = motor->limit.zero_max_speed_limit;
	motor->rpm_PID.current_value = motor->now_value.speed;
	motor->set_value.current_value += PID_caculate_data(&motor->rpm_PID);
	PEAK(motor->set_value.current_value, motor->limit.zero_max_current_limit);

	if (ABS(motor->now_value.pulse_distance) < ZERO_DISTANCE_THRESHOLD) {
		if (motor->counter.zero_count++ > 100) {
			motor->counter.zero_count = 0;
			motor->status_flag.arrive_flag = false;
			motor->status_flag.zero_flag = true;
			motor->begin = false;
			djsetzero(motor);
		}
	}
	else {
		motor->counter.zero_count = 0;
	}
}

void djanglecaculate(djmotorpointer motor) {
	motor->now_value.pulse_distance = motor->now_value.pulse_read - motor->previous_value.pulse_read;
	motor->previous_value = motor->now_value;
	if (ABS(motor->now_value.pulse_distance) > 4096) {
		motor->now_value.pulse_distance = motor->now_value.pulse_distance - PlusOrMinus(motor->now_value.pulse_distance) * motor->param.round_pulse_number;
	}
	motor->now_value.pulse_total += motor->now_value.pulse_distance;
	motor->now_value.angle = motor->now_value.pulse_total * 360.f / (motor->param.round_pulse_number * motor->param.institutional_reduction_ratio * motor->param.motor_reduction_ratio);
	if (motor->begin) {
		motor->counter.pulse_lock = motor->now_value.pulse_total;
	}
	if (motor->status_flag.is_setzero) {
		djsetzero(motor);
	}
}

void djreceivedata_can2(CAN_RxHeaderTypeDef rxmsg) {
	if ((rxmsg.StdId >= 0x201) && (rxmsg.StdId <= 0x208)) {
		uint8_t ID = rxmsg.StdId - 0x200;
		for (int i = 0; i < USE_DJ_NUM; i++) {
			if (ID == djmotor[i].ID) {
				djmotor[i].now_value.pulse_read = (int16_t)((Rxmessage[0] << 8) | Rxmessage[1]);
				int16_t sp = (int16_t)((Rxmessage[2] << 8) | Rxmessage[3]);
				djmotor[i].now_value.speed = (float)sp;
				djmotor[i].now_value.current_value =((int32_t) (Rxmessage[4] << 8) | Rxmessage[5]);
				if (djmotor[i].param.motor_reduction_ratio == M3508_RATIO) {
					djmotor[i].now_value.real_speed = (float)djmotor[i].now_value.speed;
					djmotor[i].now_value.temperature = Rxmessage[6];
					djmotor[i].now_value.current_value = (float)djmotor[i].now_value.current_value * 0.0012207f;
				}
				else {
					djmotor[i].now_value.current_value = (float)djmotor[i].now_value.current_value * 0.001f;
				}
				djanglecaculate(&djmotor[i]);
				djmotor[i].counter.last_rxtime = 0;
			}
		}
	}
}

void djcurrenttransmit(djmotorpointer motor, uint8_t i) {
	uint8_t t;
	static CAN_TxHeaderTypeDef txmsg;
	PEAK(motor->set_value.current_value, motor->param.current_limit);

	if (motor->enable != ENABLE) {
		motor->set_value.current_value = 0;
	}
	uint32_t pTxMailbox = 0;
	txmsg.IDE = CAN_ID_STD;
	txmsg.RTR = CAN_RTR_DATA;
	txmsg.DLC = 8;

	if (i <= 4) {
		txmsg.StdId = 0x200;
	}
	else {
		txmsg.StdId = 0x1ff;
	}
	if (motor->ID <= 4) {
		t = ((motor->ID - 1) * 2);
	}
	else {
		t = ((motor->ID - 5) * 2);
	}
	EncodeS16Data(&motor->set_value.current_value, &Txmessage[t]);

	ChangeDataByte(&Txmessage[t], &Txmessage[t + 1]);

	if ((motor->ID == 4) || motor->ID == 8) {
		HAL_CAN_AddTxMessage(&hcan2, &txmsg, Txmessage, &pTxMailbox);
	}
}

void djfunc() {
	for (uint8_t i = 0; i < USE_DJ_NUM; i++) {
		if (djmotor[i].enable) {
			if (djmotor[i].begin) {
				switch (djmotor[i].mode) {
				case dj_rpm:djspeedmode(&djmotor[i]); break;
				case dj_position:djpositionmode(&djmotor[i]); break;
				case dj_zero:djzeromode(&djmotor[i]); break;
				}
			}
			else {
				djlockposition(&djmotor[i]);
			}
		}
		if (i <= 8) {
		djcurrenttransmit(&djmotor[i], (i + 1));
    }
	}
}
