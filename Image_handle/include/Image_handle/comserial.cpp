void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    static uint8_t Camara_FindTarget_Backup = 0;
    /* 视觉串口 */
    if(huart->Instance == USART6)
    {
        /* 最新数据处理
             1.相机实测会有在多组FindTarget之间插入零星LoseTarget的情况(Camara_FindTarget_Backup滤波)
             2.相机距离目标太近会失去目标 此时应假设已经对正
             3.需要小心失去目标后一直遵循最后一次视觉指令移动的情况
        */
        if(Camara_FindTarget)
        {
            yMove_Reference = Visual_Target_Array[0];
            wMove_Reference = Visual_Target_Array[0];
            xMove_Reference = Visual_Target_Array[2];

            if(xMove_Reference < 47)
            {
                yMove_Reference = Camara_Center_X;
                wMove_Reference = Camara_Center_X;
            }
        }
        if(Camara_LoseTarget && Camara_FindTarget_Backup != 0)
        {
            PID_DeDefultReset(Visual);
        }
        if(Camara_LoseTarget && Camara_FindTarget_Backup == 0)
        {
            PID_DeDefultReset(Visual);
        }

        /* xMove PID Target 实时更新 */
        if(xMove_Enable == 1 && RC_ChassisMotor_Mode != RC_ChassisMode_Control&&Camara_FindTarget)
        {
            xMove_PID.Target  = xMove_Reference;
            xMove_PID.Measure = 0;
            PID_Calculate(&xMove_PID);
            Target_Speed_x = xMove_PID.Out;
        }

        /* yMove PID Target 实时更新 */
        if(yMove_Enable == 1 && RC_ChassisMotor_Mode != RC_ChassisMode_Control&&Camara_FindTarget)
        {
            yMove_PID.Target  = yMove_Reference;
            yMove_PID.Measure = Camara_Center_X;
            PID_Calculate(&yMove_PID);
            if(xMove_Reference >= 80) Target_Speed_y = yMove_PID.Out;
            if(xMove_Reference <  80) Target_Speed_y = yMove_PID.Out * abs(yMove_PID.Err) * 0.005;
        }

        /* wMove PID Target 实时更新 */
        if(wMove_Enable == 1 && RC_ChassisMotor_Mode != RC_ChassisMode_Control&&Camara_FindTarget)
        {
            wMove_PID.Target  = wMove_Reference;

            if(Visual_Target_Variable==Visual_Target_Column)
            {
                if(Project_Variable == Project_PassBall)
                {

                    if(Region_Variable==Region_A)
                        wMove_PID.Measure = Camara_Center_X+16;
                    if(Region_Variable==Region_B)
                        wMove_PID.Measure = Camara_Center_X+10;
                }

                if(Project_Variable == Project_ShootBall)
                {
                    if(Region_Variable==Region_A)
                        wMove_PID.Measure = Camara_Center_X+3;
                    if(Region_Variable==Region_B)
                        wMove_PID.Measure = Camara_Center_X;
                }

            }
            else
                wMove_PID.Measure = Camara_Center_X;

            PID_Calculate(&wMove_PID);
            Target_Speed_w = wMove_PID.Out;
        }

        if(Camara_FindTarget) Camara_FindTarget_Backup = Camara_FindTarget_Backup << 1 | 1;
        if(Camara_LoseTarget) Camara_FindTarget_Backup = Camara_FindTarget_Backup << 1 | 0;
    }