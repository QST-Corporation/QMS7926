 
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "osal.h"
#include "app_wrist.h"
#include "QMA7981.h"

#ifdef SLEEP_AlGORITHM

#ifndef ABS
#define ABS(X) ((X) < 0 ? (-1 * (X)) : (X))
#endif
#ifndef FABS
#define FABS(X) ((X) < 0.0 ? (-1.0 * (X)) : (X))
#endif

#define QST_SLEEP_MOTION_LEVEL_0		0
#define QST_SLEEP_MOTION_LEVEL_1		1
#define QST_SLEEP_MOTION_LEVEL_2		9

//static int g_debug_h;
//static int g_debug_m;
//static int g_debug_s;

enum
{
  AXIS_X = 0,
  AXIS_Y,
  AXIS_Z,

  AXIS_TOTAL
};

typedef struct
{
  float	x;
  float	y;
  float	z;
}qst_acc_data;


typedef struct
{
  float	pitch;
  float	roll;
}qst_acc_angle;


typedef struct
{
  qst_sleep_status status;
  unsigned short i_awake;
  unsigned short i_light;
  unsigned short i_deep;
}qst_sleep_state;


typedef struct
{
  double				acc_offset_xyz[QST_SLEEP_TIMER_CYC];
#if defined(QST_SLEEP_USE_PITCH_ROLL)
  qst_acc_angle		acc_angle[QST_SLEEP_TIMER_CYC];
#endif
#if defined(QST_SLEEP_USE_AXIS_DIFF)
  qst_acc_data		acc_avg[QST_SLEEP_TIMER_CYC];
#endif
  unsigned short		s_index;	// second
} qst_sleep_min;

typedef struct 
{
  unsigned char motion_level;			// calc to record motion level 0:small or static, 1: maybe awakw 4: awake
  unsigned char big_motion_count;		// big montion count, 
  unsigned char motion_count;			// montion count
} qst_sleep_record;

typedef struct 
{
  unsigned int	start_min;
  unsigned int	end_min;
  unsigned char	awake_if;
  unsigned int	awake_count;
  unsigned int	light_count;
  unsigned int	deep_count;
  unsigned int 	total_min;
  unsigned int 	total_min2;
  unsigned int 	awake_min;
  unsigned int 	light_min;
  unsigned int 	deep_min;
  float			deep_percent;
  float			deep_percent2;
} qst_sleep_result;

typedef struct
{
  char              is_enabled;
  unsigned short    m_index;        // minute index
  qst_sleep_min     min;              // record data current minute
#if defined(QST_SLEEP_DATA_IN_RECORD_ARRAY)
  qst_sleep_record 	sleepRecord[QST_SLEEP_RECORD_LEN];  // store sleep parameter
#else
  qst_sleep_record	motion;
#endif
  qst_sleep_state		stat;
  qst_sleep_result	result;
}qst_sleep_t;


static qst_sleep_t g_sleep;
static qst_acc_data qst_fifo_acc[QST_SLEEP_FIFO_LEN];


void qma_sleep_start_init(void)
{
  memset(&g_sleep, 0, sizeof(g_sleep));
}

void qma7981_sleep_start(void)
{
  qma_sleep_start_init();
  qst_sleep_set(1);
  //start timer
  osal_start_timerEx(AppWrist_TaskID, ACC_SLEEP_FIFO_SET_EVT, QST_SLEEP_FIFO_SET_INTERVAL);
}

void qma7981_sleep_stop(void)
{
  qst_sleep_set(0);
  //stop timer
  osal_stop_timerEx(AppWrist_TaskID, ACC_SLEEP_FIFO_SET_EVT);
}

void qma7981_sleep_1min_handler()
{
  qst_sleep_status sleep_status;
  sleep_status = qst_sleep_get_status();
  //report sleep_status to application.
}

void qma7981_read_fifo(void)
{
  float accData_fifo[3];
  static uint16_t icount = 0;
  static uint16_t count2s = 0;

  QMA7981_read_acc(accData_fifo);

  qst_fifo_acc[icount].x = accData_fifo[0];
  qst_fifo_acc[icount].y = accData_fifo[1];
  qst_fifo_acc[icount].z = accData_fifo[2];
  // 62ms*32 = 2s
  if (icount++ >= QST_SLEEP_FIFO_LEN - 1) {
    qst_sleep_data_process();
    icount = 0;
    // 2s*30 = 1min
    if (count2s++ >= (30 -1)) {
      count2s = 0;
      qma7981_sleep_1min_handler();
    }
  }
}

qst_sleep_status qst_sleep_get_status(void)
{
  return g_sleep.stat.status;
}

#if defined(QST_SLEEP_DATA_IN_RECORD_ARRAY)
static void qst_sleep_record_analog(void)
{
  unsigned short i_count, j_count, k_count, record_count, record_index;
  unsigned short sleep_status_sum;
  unsigned short sleep_start_min, sleep_end_min;
  unsigned short deep_min_sum;

  record_count = g_sleep.m_index;
  g_sleep.result.total_min = 0;
  g_sleep.result.light_min = 0;
  g_sleep.result.deep_min = 0;
  g_sleep.result.deep_percent = 0.0;
  g_sleep.result.start_min = 0;
  g_sleep.result.end_min = 0;
  g_sleep.result.awake_count = 0;
  if(record_count <= QST_SLEEP_RECORD_COUNT_MIN)
  {
    //QST_TRACE(YZQLOG_TAG"record < QST_SLEEP_RECORD_COUNT_MIN ingore!");
  }
  else
  {
    record_index = 0;
    while(record_index < record_count)
    {
      sleep_start_min = 0;
      sleep_end_min = 0;
      for(i_count=record_index; i_count<record_count-QST_SLEEP_RECORD_COUNT_MIN; i_count++)
      {
        sleep_status_sum = 0;			
        for(j_count=0; j_count < QST_SLEEP_RECORD_COUNT_MIN; j_count++)
        {
          sleep_status_sum += g_sleep.sleepRecord[i_count+j_count].motion_level;
        }
        if(sleep_status_sum == 0)
        {
          sleep_start_min = i_count;
          if(g_sleep.result.start_min == 0)
          {
            g_sleep.result.start_min = sleep_start_min;
          }
          //QST_TRACE(YZQLOG_TAG"qst_sleep_record_analog sleep_start_min=%d", sleep_start_min);
          break;
        }
      }
      if(sleep_start_min > 0)
      {
        for(j_count = sleep_start_min; j_count<record_count; j_count++)
        {
          if(g_sleep.sleepRecord[j_count].motion_level == QST_SLEEP_MOTION_LEVEL_2)
          {
            sleep_end_min = j_count;
            //QST_TRACE(YZQLOG_TAG"qst_sleep_record_analog sleep_end_min=%d", sleep_end_min);
            break;
          }
        }
      }

      if(sleep_end_min > sleep_start_min)
      {
        g_sleep.result.total_min += sleep_end_min-sleep_start_min;
        deep_min_sum = 0;
        for(k_count=sleep_start_min; k_count<=sleep_end_min; k_count++)
        {
          if(g_sleep.sleepRecord[k_count].motion_level == QST_SLEEP_MOTION_LEVEL_0)
          {
            deep_min_sum++;
          }
          else
          {
            if(deep_min_sum >= QST_SLEEP_DEEPSLEEP_CONTINUE_NUM)
            {
              //QST_TRACE(YZQLOG_TAG"qst_sleep_record_analog add deep min=%d", deep_min_sum);
              g_sleep.result.deep_min += deep_min_sum;
            }
            deep_min_sum = 0;
          }
        }
        record_index = sleep_end_min;				
        g_sleep.result.awake_count++;
        g_sleep.result.end_min = sleep_end_min;
        //QST_TRACE(YZQLOG_TAG"total:%d deep=%d wake count:%d", g_sleep.result.total_min,g_sleep.result.deep_min,g_sleep.result.awake_count);
      }
      else
      {
        record_index++;
      }

    }
  }

  // calc light time
  g_sleep.result.light_min = g_sleep.result.total_min - g_sleep.result.deep_min;

  // calc light time
  // calc deep percentage
  if((g_sleep.result.total_min > 0)&&(g_sleep.result.deep_min > 0))
    g_sleep.result.deep_percent = (float)g_sleep.result.deep_min/(float)g_sleep.result.total_min;
  else
    g_sleep.result.deep_percent = 0.0;
}


static void qst_sleep_record_statistics(void)
{
  unsigned short i_count, record_count;
  float WEIGHT_PRE_1 = 0.2;
  float WEIGHT_PRE = 0.2;
  float WEIGHT_CUR = 0.4;
  float WEIGHT_NXT = 0.2;
  float calc_montion_count;
  float calc_big_montion_count;

  record_count = g_sleep.m_index;
  //QST_TRACE(YZQLOG_TAG"qst_sleep_record_statistics record_count = %d", record_count);
  if(record_count  <= QST_SLEEP_RECORD_COUNT_MIN)
  {
    //QST_TRACE(YZQLOG_TAG"record count < QST_SLEEP_RECORD_COUNT_MIN ingore!");
  }
  else
  {
    // first and last 2 minutes ,set status awake!
    g_sleep.sleepRecord[0].motion_level = QST_SLEEP_MOTION_LEVEL_2;
    g_sleep.sleepRecord[1].motion_level = QST_SLEEP_MOTION_LEVEL_2;		
    g_sleep.sleepRecord[0].big_motion_count++;
    g_sleep.sleepRecord[1].big_motion_count++;
    g_sleep.sleepRecord[record_count-1].motion_level = QST_SLEEP_MOTION_LEVEL_2;
    g_sleep.sleepRecord[record_count-2].motion_level = QST_SLEEP_MOTION_LEVEL_2;
    g_sleep.sleepRecord[record_count-1].big_motion_count++;
    g_sleep.sleepRecord[record_count-2].big_motion_count++;
    // first and last 2 minutes ,set status awake!
    for(i_count=2; i_count<record_count-2; i_count++)
    {
      calc_montion_count = 0.0;
      calc_montion_count = (float)g_sleep.sleepRecord[i_count].motion_count*1.0;
      calc_big_montion_count = (float)g_sleep.sleepRecord[i_count].big_motion_count*1.0;

      if(calc_montion_count <= QST_SLEEP_MOTION_LIGHT_THRESHOLD)	// old 8.00
      {
        g_sleep.sleepRecord[i_count].motion_level = QST_SLEEP_MOTION_LEVEL_0;
      }
      else if(calc_montion_count >= QST_SLEEP_MOTION_AWAKE_THRESHOLD)
      {
        g_sleep.sleepRecord[i_count].motion_level = QST_SLEEP_MOTION_LEVEL_2;
      }
      else
      {
        g_sleep.sleepRecord[i_count].motion_level = QST_SLEEP_MOTION_LEVEL_1;
      }

      if(g_sleep.sleepRecord[i_count].motion_level < QST_SLEEP_MOTION_LEVEL_2)
      {
        if(calc_big_montion_count >= QST_SLEEP_BIGMOTION_LIGHT_THRESHOLD)
        {
          g_sleep.sleepRecord[i_count].motion_level = QST_SLEEP_MOTION_LEVEL_1;
        }
        if(calc_big_montion_count >= QST_SLEEP_BIGMOTION_AWAKE_THRESHOLD)
        {
          g_sleep.sleepRecord[i_count].motion_level = QST_SLEEP_MOTION_LEVEL_2;
        }
      }

      //QST_TRACE(YZQLOG_TAG"minute=%d,motion level=%d, c:%f, b-c:%f", i_count, g_sleep.sleepRecord[i_count].motion_level, calc_montion_count, calc_big_montion_count);
    }
  }
}


static void qst_sleep_calc_montion(unsigned short min_index)
{
  unsigned short s_index;
  double acc_offset_sum;
#if defined(QST_SLEEP_USE_PITCH_ROLL)
  float pitch_1, roll_1;
  float pitch[2] = {0.0, 0.0};
  float roll[2] = {0.0, 0.0};
#endif
#if defined(QST_SLEEP_USE_AXIS_DIFF)
  float acc_min[3],acc_max[3];
  qst_acc_data acc_diff;
#endif

  g_sleep.sleepRecord[min_index].motion_count = 0;
  g_sleep.sleepRecord[min_index].big_motion_count = 0;
  g_sleep.sleepRecord[min_index].motion_level = QST_SLEEP_MOTION_LEVEL_2;

  for(s_index=0; s_index<QST_SLEEP_TIMER_CYC; s_index++)
  {
    acc_offset_sum = g_sleep.min.acc_offset_xyz[s_index];
    //QST_TRACE(YZQLOG_TAG"montion_count acc_offset_sum[%d]=%f", s_index, acc_offset_sum);
    // calc small montion count
    if(acc_offset_sum > QST_SLEEP_MONTION_THRESHOLD)
    {
      g_sleep.sleepRecord[min_index].motion_count++;
    }		
    // calc small montion count
    // calc if have large montion 
    if(acc_offset_sum > QST_SLEEP_BIGMONTION_THRESHOLD)
    {
      g_sleep.sleepRecord[min_index].big_motion_count++;
    }
    // calc if have large montion 
#if defined(QST_SLEEP_USE_PITCH_ROLL)
    //norm = (g_sleep.min.acc_avg[s_index].x*g_sleep.min.acc_avg[s_index].x)
    //		+(g_sleep.min.acc_avg[s_index].y*g_sleep.min.acc_avg[s_index].y)
    //		+(g_sleep.min.acc_avg[s_index].z*g_sleep.min.acc_avg[s_index].z);
    //norm = sqrtf(norm);
    //pitch_1 = asinf(-g_sleep.min.acc_avg[s_index].y/norm)*57.2958;	//(180/pi)
    //roll_1 = asinf(g_sleep.min.acc_avg[s_index].x/norm)*57.2958;
    pitch_1 = g_sleep.min.acc_angle[s_index].pitch;
    roll_1 = g_sleep.min.acc_angle[s_index].roll;
    if(s_index == 0)
    {
      pitch[0] = pitch[1] = pitch_1;
      roll[0] = roll[1] = roll_1;
    }
    else
    {
      pitch[0] = (pitch_1<pitch[0])?pitch_1:pitch[0];
      pitch[1] = (pitch_1>pitch[1])?pitch_1:pitch[1];

      roll[0] = (roll_1<roll[0])?roll_1:roll[0];
      roll[1] = (roll_1>roll[1])?roll_1:roll[1];
    }
#endif
#if defined(QST_SLEEP_USE_AXIS_DIFF)
    if(s_index == 0)
    {
      acc_min[0] = acc_max[0] = g_sleep.min.acc_avg[s_index].x;
      acc_min[1] = acc_max[1] = g_sleep.min.acc_avg[s_index].y;
      acc_min[2] = acc_max[2] = g_sleep.min.acc_avg[s_index].z;
    }
    else
    {
      acc_min[0]=(g_sleep.min.acc_avg[s_index].x<acc_min[0])?g_sleep.min.acc_avg[s_index].x:acc_min[0];
      acc_max[0]=(g_sleep.min.acc_avg[s_index].x>acc_max[0])?g_sleep.min.acc_avg[s_index].x:acc_max[0];
      acc_min[1]=(g_sleep.min.acc_avg[s_index].y<acc_min[1])?g_sleep.min.acc_avg[s_index].y:acc_min[1];
      acc_max[1]=(g_sleep.min.acc_avg[s_index].y>acc_max[1])?g_sleep.min.acc_avg[s_index].y:acc_max[1];
      acc_min[2]=(g_sleep.min.acc_avg[s_index].z<acc_min[2])?g_sleep.min.acc_avg[s_index].z:acc_min[2];
      acc_max[2]=(g_sleep.min.acc_avg[s_index].z>acc_max[2])?g_sleep.min.acc_avg[s_index].z:acc_max[2];
    }
#endif
  }

#if defined(QST_SLEEP_USE_PITCH_ROLL)
  //QST_TRACE(YZQLOG_TAG"montion_count pitch diff:%f, roll diff:%f", FABS(pitch[1]-pitch[0]), FABS(roll[1]-roll[0]));
  if((FABS(pitch[1]-pitch[0]) > QST_SLEEP_PITCH_ROLL_THRESHOLD)||(FABS(roll[1]-roll[0]) > QST_SLEEP_PITCH_ROLL_THRESHOLD))
  {
    g_sleep.sleepRecord[min_index].big_motion_count++;
  }
#endif
#if defined(QST_SLEEP_USE_AXIS_DIFF)
  acc_diff.x = FABS(acc_max[0]-acc_min[0]);
  acc_diff.y = FABS(acc_max[1]-acc_min[1]);
  acc_diff.z = FABS(acc_max[2]-acc_min[2]);	
  //QST_TRACE(YZQLOG_TAG"montion_count acc_diff:%f %f %f", acc_diff.x, acc_diff.y, acc_diff.z);
  if(acc_diff.z > QST_SLEEP_AXIS_DIFF_THRESHOLD)
  {
    g_sleep.sleepRecord[min_index].big_motion_count++;
  }
#endif

#if defined(QST_SLEEP_AWAKE_BY_CUSTOMER_CONDITION)
  if(qst_sleep_customer_is_awake())
  {
    g_sleep.sleepRecord[min_index].big_motion_count = QST_SLEEP_TIMER_CYC;
  }
#endif

}
#else
void qst_sleep_calc_result(void)
{
  g_sleep.result.awake_min = g_sleep.result.awake_min*QST_SLEEP_TIMER_RATIO;
  g_sleep.result.light_min = g_sleep.result.light_min*QST_SLEEP_TIMER_RATIO;
  g_sleep.result.deep_min  = g_sleep.result.deep_min*QST_SLEEP_TIMER_RATIO;

  g_sleep.result.total_min = g_sleep.result.awake_min+g_sleep.result.light_min+g_sleep.result.deep_min;
  if(g_sleep.result.total_min > 0)
    g_sleep.result.deep_percent = (float)g_sleep.result.deep_min/g_sleep.result.total_min;
  else
    g_sleep.result.deep_percent = 0.0;

  g_sleep.result.total_min2 = g_sleep.result.light_min+g_sleep.result.deep_min;
  if(g_sleep.result.total_min2 > 0)
    g_sleep.result.deep_percent2 = (float)g_sleep.result.deep_min/g_sleep.result.total_min2;
  else
    g_sleep.result.deep_percent2 = 0.0;
}

void qst_sleep_calc_motion(void)
{
  unsigned short s_index;
  double acc_offset_sum;
  float calc_motion_count, calc_big_motion_count;
#if defined(QST_SLEEP_USE_PITCH_ROLL)
  float pitch_1, roll_1;
  float pitch[2] = {0.0, 0.0};
  float roll[2] = {0.0, 0.0};
#endif
#if defined(QST_SLEEP_USE_AXIS_DIFF)
  float acc_min[3],acc_max[3];
  qst_acc_data acc_diff;
#endif

  // init motion
  g_sleep.motion.motion_count = 0;
  g_sleep.motion.big_motion_count = 0;
  g_sleep.motion.motion_level = QST_SLEEP_MOTION_LEVEL_2;
  // init motion
  // calc motion
  for(s_index=0; s_index<QST_SLEEP_TIMER_CYC; s_index++)
  {
    acc_offset_sum = g_sleep.min.acc_offset_xyz[s_index];
    // calc motion count
    if (acc_offset_sum > QST_SLEEP_MONTION_THRESHOLD)
    {
      g_sleep.motion.motion_count++;
    }
    // calc motion count
    // calc big montion count
    if (acc_offset_sum > QST_SLEEP_BIGMONTION_THRESHOLD)
    {
      g_sleep.motion.big_motion_count++;
    }
    // calc big motion count
#if defined(QST_SLEEP_USE_PITCH_ROLL)
    pitch_1 = g_sleep.min.acc_angle[s_index].pitch;
    roll_1 = g_sleep.min.acc_angle[s_index].roll;
    if(s_index == 0)
    {
      pitch[0] = pitch[1] = pitch_1;
      roll[0] = roll[1] = roll_1;
    }
    else
    {
      pitch[0] = (pitch_1<pitch[0])?pitch_1:pitch[0];
      pitch[1] = (pitch_1>pitch[1])?pitch_1:pitch[1];

      roll[0] = (roll_1<roll[0])?roll_1:roll[0];
      roll[1] = (roll_1>roll[1])?roll_1:roll[1];
    }
#endif
#if defined(QST_SLEEP_USE_AXIS_DIFF)
    if(s_index == 0)
    {
      acc_min[0] = acc_max[0] = g_sleep.min.acc_avg[s_index].x;
      acc_min[1] = acc_max[1] = g_sleep.min.acc_avg[s_index].y;
      acc_min[2] = acc_max[2] = g_sleep.min.acc_avg[s_index].z;
    }
    else
    {
      acc_min[0]=(g_sleep.min.acc_avg[s_index].x<acc_min[0])?g_sleep.min.acc_avg[s_index].x:acc_min[0];
      acc_max[0]=(g_sleep.min.acc_avg[s_index].x>acc_max[0])?g_sleep.min.acc_avg[s_index].x:acc_max[0];
      acc_min[1]=(g_sleep.min.acc_avg[s_index].y<acc_min[1])?g_sleep.min.acc_avg[s_index].y:acc_min[1];
      acc_max[1]=(g_sleep.min.acc_avg[s_index].y>acc_max[1])?g_sleep.min.acc_avg[s_index].y:acc_max[1];
      acc_min[2]=(g_sleep.min.acc_avg[s_index].z<acc_min[2])?g_sleep.min.acc_avg[s_index].z:acc_min[2];
      acc_max[2]=(g_sleep.min.acc_avg[s_index].z>acc_max[2])?g_sleep.min.acc_avg[s_index].z:acc_max[2];
    }
#endif
  }

#if defined(QST_SLEEP_USE_PITCH_ROLL)
  //QST_TRACE(YZQLOG_TAG"montion_count pitch diff:%f, roll diff:%f", FABS(pitch[1]-pitch[0]), FABS(roll[1]-roll[0]));
  if((FABS(pitch[1]-pitch[0]) > QST_SLEEP_PITCH_ROLL_THRESHOLD)||(FABS(roll[1]-roll[0]) > QST_SLEEP_PITCH_ROLL_THRESHOLD))
  {
    g_sleep.motion.big_motion_count++;
  }
#endif
#if defined(QST_SLEEP_USE_AXIS_DIFF)
  acc_diff.x = FABS(acc_max[0]-acc_min[0]);
  acc_diff.y = FABS(acc_max[1]-acc_min[1]);
  acc_diff.z = FABS(acc_max[2]-acc_min[2]);	
  //QST_TRACE(YZQLOG_TAG"montion_count acc_diff:%f %f %f", acc_diff.x, acc_diff.y, acc_diff.z);
  if(acc_diff.z > QST_SLEEP_AXIS_DIFF_THRESHOLD)
  {
    g_sleep.motion.big_motion_count++;
  }
#endif
#if defined(QST_SLEEP_AWAKE_BY_CUSTOMER_CONDITION)
  if (qst_sleep_customer_is_awake())
  {
    g_sleep.motion.big_motion_count = QST_SLEEP_TIMER_CYC;
  }
#endif
  // calc motion

  //QST_TRACE(YZQLOG_TAG"montion_count motion:%d, big motion:%d", g_sleep.motion.motion_count, g_sleep.motion.big_motion_count);
  calc_motion_count = (float)(g_sleep.motion.motion_count*1.0);
  calc_big_motion_count = (float)(g_sleep.motion.big_motion_count*1.0);
  // calc sleep level
  if (calc_motion_count <= QST_SLEEP_MOTION_LIGHT_THRESHOLD)
  {
    g_sleep.motion.motion_level = QST_SLEEP_MOTION_LEVEL_0;
  }
  else if (calc_motion_count >= QST_SLEEP_MOTION_AWAKE_THRESHOLD)
  {
    g_sleep.motion.motion_level = QST_SLEEP_MOTION_LEVEL_2;
  }
  else
  {
    g_sleep.motion.motion_level = QST_SLEEP_MOTION_LEVEL_1;
  }

  if (g_sleep.motion.motion_level < QST_SLEEP_MOTION_LEVEL_2)
  {
    if (calc_big_motion_count >= QST_SLEEP_BIGMOTION_LIGHT_THRESHOLD)
    {
      g_sleep.motion.motion_level = QST_SLEEP_MOTION_LEVEL_1;
    }
    if (calc_big_motion_count >= QST_SLEEP_BIGMOTION_AWAKE_THRESHOLD)
    {
      g_sleep.motion.motion_level = QST_SLEEP_MOTION_LEVEL_2;
    }
  }
  //QST_TRACE(YZQLOG_TAG"montion_count sleep level:%d", g_sleep.motion.motion_level);
  // calc sleep level
}

void qst_sleep_statistics(unsigned short m_min)
{

  qst_sleep_calc_motion();
  //LOG("%s\n",__func__);

  if(g_sleep.stat.status == QST_SLEEP_AWAKE)
  {
    g_sleep.result.awake_min++;

    if(g_sleep.motion.motion_level >= QST_SLEEP_MOTION_LEVEL_1)
    {
      g_sleep.stat.i_light = 0;
    }
    else if(g_sleep.motion.motion_level <= QST_SLEEP_MOTION_LEVEL_0)
    {
      g_sleep.stat.i_light++;
      if(g_sleep.stat.i_light >= QST_SLEEP_LIGHTSLEEP_CONTINUE_NUM)
      {
        //memset(&(g_sleep.stat), 0, sizeof(qst_sleep_state));
        g_sleep.stat.i_awake=0;
        g_sleep.stat.i_light=0;
        g_sleep.stat.i_deep=0;
        g_sleep.stat.status = QST_SLEEP_LIGHT;	
        g_sleep.result.light_count++;
        //QST_TRACE(YZQLOG_TAG"Min:%d, enter light sleep \n", m_min);
      }
    }
  }
  else if(g_sleep.stat.status == QST_SLEEP_LIGHT)
  {
    if(g_sleep.motion.motion_level <= QST_SLEEP_MOTION_LEVEL_1)
    {
      g_sleep.result.light_min++;
      g_sleep.stat.i_awake = 0;
      if(g_sleep.motion.motion_level == QST_SLEEP_MOTION_LEVEL_0)
      {
        g_sleep.stat.i_deep++;
      }
      else
      {
        g_sleep.stat.i_deep = 0;
      }

      if(g_sleep.stat.i_deep >= QST_SLEEP_DEEPSLEEP_CONTINUE_NUM)
      {
        //memset(&(g_sleep.stat), 0, sizeof(qst_sleep_state));				
        g_sleep.stat.i_awake=0;
        g_sleep.stat.i_light=0;
        g_sleep.stat.i_deep=0;
        g_sleep.stat.status = QST_SLEEP_DEEP;	
        g_sleep.result.deep_count++;
        //QST_TRACE(YZQLOG_TAG"Min:%d, enter deep sleep \n", m_min);
      }
    }
    else
    {
      g_sleep.stat.i_awake++;
      g_sleep.stat.i_deep = 0;
      if(g_sleep.stat.i_awake >= 1)
      {
        g_sleep.result.awake_min++;
        //memset(&(g_sleep.stat), 0, sizeof(qst_sleep_state));
        g_sleep.stat.i_awake=0;
        g_sleep.stat.i_light=0;
        g_sleep.stat.i_deep=0;
        g_sleep.stat.status = QST_SLEEP_AWAKE;
        //QST_TRACE(YZQLOG_TAG"Min:%d, enter awake \n", m_min);
        if(g_sleep.result.awake_if)
        {
          g_sleep.result.awake_count++;
          g_sleep.result.awake_if = 0;
          //QST_TRACE(YZQLOG_TAG"awake_count = %d \n", g_sleep.result.awake_count);
        }
      }
      else
      {
        // to do
      }
    }
  }
  else if(g_sleep.stat.status == QST_SLEEP_DEEP)
  {
    g_sleep.result.awake_if = 1;
    if(g_sleep.motion.motion_level <= QST_SLEEP_MOTION_LEVEL_0)
    {
      g_sleep.result.deep_min++;

      g_sleep.stat.i_light = 0;
      g_sleep.stat.i_awake = 0;
    }
    else if(g_sleep.motion.motion_level == QST_SLEEP_MOTION_LEVEL_1)
    {
      g_sleep.stat.i_awake = 0;
      g_sleep.stat.i_light++;
      if(g_sleep.stat.i_light >= 1)
      {
        g_sleep.result.light_min++;
        //memset(&(g_sleep.stat), 0, sizeof(qst_sleep_state));
        g_sleep.stat.i_awake=0;
        g_sleep.stat.i_light=0;
        g_sleep.stat.i_deep=0;
        g_sleep.stat.status = QST_SLEEP_LIGHT;
        g_sleep.result.light_count++;
        //QST_TRACE(YZQLOG_TAG"Min:%d, enter light sleep \n", m_min);
      }
      else
      {
        // to do 
      }
    }
    else if(g_sleep.motion.motion_level == QST_SLEEP_MOTION_LEVEL_2)
    {
      g_sleep.stat.i_light = 0;
      g_sleep.stat.i_awake++;
      if(g_sleep.stat.i_awake >= 1)
      {
        g_sleep.result.awake_min++;
        //memset(&(g_sleep.stat), 0, sizeof(qst_sleep_state));
        g_sleep.stat.i_awake=0;
        g_sleep.stat.i_light=0;
        g_sleep.stat.i_deep=0;
        g_sleep.stat.status = QST_SLEEP_AWAKE;
        //QST_TRACE(YZQLOG_TAG"Min:%d, enter awake \n", m_min);
        if(g_sleep.result.awake_if)
        {
          g_sleep.result.awake_count++;
          g_sleep.result.awake_if = 0;
          //QST_TRACE(YZQLOG_TAG"awake_count = %d \n", g_sleep.result.awake_count);
        }
      }
      else
      {
        // to do
      }
    }
  }

}

#endif
/*
Deal with data per-second.
*/
void qst_sleep_data_process(void)
{
  unsigned short s_index, icount;
  qst_acc_data	acc_sum;
  qst_acc_data	acc_avg;
  float offsetx, offsety, offsetz;
#if defined(QST_SLEEP_USE_PITCH_ROLL)
  float norm;
#endif

  if(g_sleep.is_enabled)
  {
    if(g_sleep.min.s_index < QST_SLEEP_TIMER_CYC)
    {
      s_index = g_sleep.min.s_index;
      //QST_TRACE(YZQLOG_TAG"Second: %d", g_sleep.min.s_index);
      // calc FIFO sum
      acc_sum.x = 0.0;
      acc_sum.y = 0.0;
      acc_sum.z = 0.0;

      for(icount=0; icount<QST_SLEEP_FIFO_LEN; icount++)
      {
        acc_sum.x += qst_fifo_acc[icount].x;
        acc_sum.y += qst_fifo_acc[icount].y;
        acc_sum.z += qst_fifo_acc[icount].z;
      }
      //QST_TRACE(YZQLOG_TAG"acc_sum %f,%f,%f", acc_sum.x, acc_sum.y, acc_sum.z);
      // calc FIFO sum end

      // calc FIFO average
      acc_avg.x = acc_sum.x/((float)QST_SLEEP_FIFO_LEN);
      acc_avg.y = acc_sum.y/((float)QST_SLEEP_FIFO_LEN);
      acc_avg.z = acc_sum.z/((float)QST_SLEEP_FIFO_LEN);
#if defined(QST_SLEEP_USE_AXIS_DIFF)
      g_sleep.min.acc_avg[s_index].x = acc_avg.x;
      g_sleep.min.acc_avg[s_index].y = acc_avg.y;
      g_sleep.min.acc_avg[s_index].z = acc_avg.z;
#endif
      //QST_TRACE(YZQLOG_TAG"acc_avg %f,%f,%f", acc_avg.x, acc_avg.y, acc_avg.z);
      // calc FIFO average end

      // calc average offset square sum
      g_sleep.min.acc_offset_xyz[s_index] = 0.0;
      for(icount=0; icount<QST_SLEEP_FIFO_LEN; icount++)
      {
        offsetx = qst_fifo_acc[icount].x - acc_avg.x;
        offsety = qst_fifo_acc[icount].y - acc_avg.y;
        offsetz = qst_fifo_acc[icount].z - acc_avg.z;
        g_sleep.min.acc_offset_xyz[s_index] += (offsetx*offsetx) + (offsety*offsety) + (offsetz*offsetz);
      }
    //if((int)(g_sleep.min.acc_offset_xyz[s_index]*1000) > 0)
      //LOG("acc_offset_xyz XXXX %d \n", (int)(g_sleep.min.acc_offset_xyz[s_index]*1000));
      // calc average offset square sum

#if defined(QST_SLEEP_USE_PITCH_ROLL)
      norm = (acc_avg.x * acc_avg.x)+(acc_avg.y * acc_avg.y)+(acc_avg.z * acc_avg.z);
      norm = sqrtf(norm);
      g_sleep.min.acc_angle[s_index].pitch = asinf(-acc_avg.y/norm)*57.2958;	//(180/pi)
      g_sleep.min.acc_angle[s_index].roll = asinf(acc_avg.x/norm)*57.2958;
#endif
      g_sleep.min.s_index++;
      //LOG("s_index XXXX %d \n", g_sleep.min.s_index);
      if(g_sleep.min.s_index == QST_SLEEP_TIMER_CYC)	// one minute reach , start to calc montion
      {
        //LOG("s_index AAAAA \n");
        //QST_TRACE(YZQLOG_TAG"minute %d, start calc!", g_sleep.m_index);
#if defined(QST_SLEEP_DATA_IN_RECORD_ARRAY)
        qst_sleep_calc_montion(g_sleep.m_index);
#else
        qst_sleep_statistics(g_sleep.m_index);
#endif
        memset(&g_sleep.min, 0, sizeof(g_sleep.min));

        g_sleep.m_index++;
#if defined(QST_SLEEP_DATA_IN_RECORD_ARRAY)
        if(g_sleep.m_index >= QST_SLEEP_RECORD_LEN)
        {
          g_sleep.m_index = QST_SLEEP_RECORD_LEN-1;
          //QST_TRACE(YZQLOG_TAG"g_sleep.sleepRecord exceed!!!");
        }
#endif
      }
    }
  }
  else
  {
    g_sleep.stat.status = QST_SLEEP_UNKNOW;
    //QST_TRACE(YZQLOG_TAG"Sleep monitor is disabled!!!");
  }
}

uint8_t qst_sleep_set(char enable)
{
  if((g_sleep.is_enabled == 0) && (enable))
  {
    memset(&g_sleep, 0, sizeof(g_sleep));
    g_sleep.stat.status = QST_SLEEP_AWAKE;
    g_sleep.is_enabled = enable;

  }
  else if((g_sleep.is_enabled == 1) && (enable==0))
  {
    g_sleep.is_enabled = enable;

    // sleep end, start to Statistics 
#if defined(QST_SLEEP_DATA_IN_RECORD_ARRAY)
    qst_sleep_record_statistics();
    qst_sleep_record_analog();
#else
    qst_sleep_calc_result();
    g_sleep.stat.status = QST_SLEEP_UNKNOW;
#endif

  }
  //LOG("qst_sleep_set:%d\n",g_sleep.is_enabled);

  return SUCCESS;
}
#endif
