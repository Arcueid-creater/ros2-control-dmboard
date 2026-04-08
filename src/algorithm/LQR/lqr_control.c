//
// Created by w on 25-12-1.
//
#include "lqr_control.h"
#include <stdlib.h>
#include <string.h>
lqr_object_t *lqr_obj[LQR_NUM_MAX] = {NULL};
uint8_t idx=0;
static void apply_control_limits(lqr_object_t *object);
lqr_object_t *lqr_register(lqr_config_t *config,void *update_control )
{
    if (config == NULL)
    {
        return NULL;
    }
    if (idx >= LQR_NUM_MAX || config->control_dim > LQR_CONTROL_DIM_MAX || config->state_dim > LQR_STATE_DIM_MAX)
    {
        return NULL;
    }
    lqr_object_t *object = (lqr_object_t *)pvPortMalloc(sizeof(lqr_object_t));
    if (object == NULL)
    {
        return NULL;
    }
    object->enable = 0;//默认不开启
    object->control_dim=config->control_dim;
    object->state_dim=config->state_dim;
    object->k_type = config->k_type;
    object->control = update_control;

    uint16_t K_size = config->control_dim * config->state_dim;
    //====================================分配内存==================================
    object->K_data = (float *)pvPortMalloc(K_size * sizeof(float));
    object->state = (float *)pvPortMalloc(config->state_dim * sizeof(float));
    object->lqr_output=(float*)pvPortMalloc(config->control_dim*sizeof(float));
    object->feedforward=(float *)pvPortMalloc(config->control_dim*sizeof(float));
    object->state_ref=(float *)pvPortMalloc(config->state_dim*sizeof(float));
    object->output_max=(float *)pvPortMalloc(config->control_dim*sizeof(float));
    object->output_min=(float *)pvPortMalloc(config->control_dim*sizeof(float));
    object->state_err=(float *)pvPortMalloc(config->state_dim*sizeof(float));
    object->temp_buffer=(float *)pvPortMalloc(config->control_dim*sizeof(float));

    //如果分配内存失败，需要释放内存
    if (!object->K_data || !object->state || !object->state_ref || !object->state_err ||
         !object->lqr_output || !object->feedforward || !object->output_max ||
         !object->output_min || !object->temp_buffer)
    {
        // ? 释放所有已分配的内存
        if (object->temp_buffer) vPortFree(object->temp_buffer);
        if (object->state_err) vPortFree(object->state_err);
        if (object->output_min) vPortFree(object->output_min);
        if (object->output_max) vPortFree(object->output_max);
        if (object->state_ref) vPortFree(object->state_ref);
        if (object->feedforward) vPortFree(object->feedforward);
        if (object->lqr_output) vPortFree(object->lqr_output);
        if (object->state) vPortFree(object->state);
        if (object->K_data) vPortFree(object->K_data);



        // 释放 object 本身
        vPortFree(object);
        return NULL;
    }

    //初始化K矩阵，因为K矩阵内部的数据是指向K_data的，两者公用同一块内存，所以只需要初始化一次
    if (config->K_data!=NULL)
    {
        memcpy(object->K_data, config->K_data, K_size * sizeof(float));
    }
    else
    {
        memset(object->K_data, 0, K_size * sizeof(float));

    }
    arm_mat_init_f32(&object->K_matrix, config->control_dim, config->state_dim, object->K_data);

    memset(object->state, 0, config->state_dim * sizeof(float));
    memset(object->state_ref, 0, config->state_dim * sizeof(float));
    memset(object->lqr_output, 0, config->control_dim * sizeof(float));
    memset(object->feedforward, 0, config->control_dim * sizeof(float));

    if (config->feedforward!=NULL)
    {
        memcpy(object->feedforward, config->feedforward, config->control_dim * sizeof(float));
    }
    //===================默认限幅非常大，没有限幅
    for (int i = 0; i < config->control_dim; i++)
    {
        object->output_max[i] = 1e10f;
        object->output_min[i] = -1e10f;
    }

    if (config->output_max!=NULL)
    {
        memcpy(object->output_max, config->output_max, config->control_dim * sizeof(float));
    }

    if (config->output_min!=NULL)
    {
        memcpy(object->output_min, config->output_min, config->control_dim * sizeof(float));
    }


    lqr_obj[idx++] = object;
    return object;
}

float* lqr_update(lqr_object_t *object,float*state,float *state_ref )
{
    if (object==NULL || state==NULL || state_ref==NULL)
    {
        return NULL;
    }
    if (object->enable==1)
    {
        if (object->control != NULL)
        {
            float* new_K = object->control();
            if (new_K != NULL)
            {
                memcpy(object->K_data, new_K, object->control_dim * object->state_dim * sizeof(float));//如果新的K矩阵和初始化时的K矩阵行列不一样，需要重新初始化
            }
        }

        arm_sub_f32((float32_t*)state, (float32_t *)state_ref, object->state_err, object->state_dim);
        arm_matrix_instance_f32 error_matrix;
        arm_mat_init_f32(&error_matrix, object->state_dim, 1, object->state_err);
        arm_matrix_instance_f32 result_matrix ;

        arm_mat_init_f32(&result_matrix,object->control_dim, 1, object->temp_buffer);
        arm_mat_mult_f32(&object->K_matrix, &error_matrix, &result_matrix);
        arm_sub_f32(object->feedforward, object->temp_buffer, object->lqr_output, object->control_dim);
        apply_control_limits(object);
        return object->lqr_output;
    }
    memset(object->lqr_output, 0, object->control_dim * sizeof(float));
    return object->lqr_output;
}


static void apply_control_limits(lqr_object_t *object)
{
    for (int i = 0; i < object->control_dim; i++) {
        if (object->lqr_output[i] > object->output_max[i]) {
            object->lqr_output[i] = object->output_max[i];
        }
        if (object->lqr_output[i] < object->output_min[i]) {
            object->lqr_output[i] = object->output_min[i];
        }
    }
}


void lqr_SetLimits(lqr_object_t *object,float *max_control,float*min_control)
{
    if (object==NULL)
    {
        return ;
    }
    if (max_control != NULL)
    {
        memcpy(object->output_max,max_control,object->control_dim * sizeof(float));
    }
    if (min_control != NULL)
    {
        memcpy(object->output_min,min_control,object->control_dim * sizeof(float));
    }

}

void lqr_SetFeedforward(lqr_object_t *object, const float *feedforward)
{
    if (object==NULL)
    {
        return ;
    }
    if (feedforward != NULL) {
        memcpy(object->feedforward, feedforward, object->control_dim * sizeof(float));
    }
}
void lqr_Clear (lqr_object_t *object)
{
    if (object==NULL)
    {
        return ;
    }
    memset(object->state_err, 0, object->state_dim * sizeof(float));
    memset(object->lqr_output, 0, object->control_dim * sizeof(float));
}
void lqr_Enable(lqr_object_t *object)
{
    if (object==NULL)
    {
        return ;
    }
    object->enable = 1;
}

void lqr_Disable (lqr_object_t *object)
{
    if (object==NULL)
    {
        return ;
    }
    object->enable = 0;
}
