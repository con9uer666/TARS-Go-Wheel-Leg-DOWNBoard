"""Host tests for the actual DR16 receive path; HAL/RTOS calls are stubbed."""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
with tempfile.TemporaryDirectory() as directory:
    tmp = Path(directory)
    (tmp / 'main.h').write_text('''#pragma once
#include <stdint.h>
typedef struct { void *Instance; } UART_HandleTypeDef;
typedef int DMA_HandleTypeDef;
#define UART5 ((void *)5)
#define DMA_IT_HT 1
#define __HAL_DMA_DISABLE_IT(a,b) ((void)0)
#define __HAL_UNLOCK(a) ((void)0)
uint32_t HAL_GetTick(void);
int HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *, uint8_t *, uint16_t);
''')
    (tmp / 'usart.h').write_text('#include "main.h"\nextern UART_HandleTypeDef huart5, huart2;\n')
    (tmp / 'cmsis_os.h').write_text('typedef void *osThreadId;\n')
    (tmp / 'USER_CAN.h').write_text('#include <stdint.h>\nextern uint8_t motor_output_enable;\n')
    (tmp / 'state.h').write_bytes((root / 'others/State.h').read_bytes())
    (tmp / 'motor.h').write_text('#pragma once\n#include <stdint.h>\ntypedef struct { float Target_Vx, Target_Vy; uint8_t Target_Leg_State, Chassis_Mode; } Foot_Chassis_t;\nextern Foot_Chassis_t Foot_Chassis;\nextern uint8_t spinning_enable, jump_enable, jump_cmd;\n')
    (tmp / 'Board2Board.h').write_text('#include <stdint.h>\nextern uint8_t upstairs_flag, sit_mode_enable, leg_state_locked_short;\n')
    (tmp / 'test.c').write_text('''
#include "remoter.h"
#include "state.h"
#include <assert.h>
#include "motor.h"
Foot_Chassis_t Foot_Chassis;
uint8_t spinning_enable, jump_enable = 1, jump_cmd = 1;
uint8_t upstairs_flag, sit_mode_enable, leg_state_locked_short;
uint8_t Rx_Data[BUFF_SIZE], usart2RxBuf[256], STOPFLAG, motor_output_enable = 1;
UART_HandleTypeDef huart5, huart2;
enum State System_State;
uint8_t State_changed;
static uint32_t tick;
uint32_t HAL_GetTick(void) { return tick; }
int HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *h, uint8_t *b, uint16_t s)
{ (void)h; (void)b; (void)s; return 0; }
int main(void)
{
    /* Centered sticks, left up, right down, centered wheel. */
    uint8_t frame[18] = {0,4,32,0,1,0x68,0,0,0,0,0,0,0,0,0,0,0,4};
    Remoter_Init();
    assert(STOPFLAG && !motor_output_enable);
    Remoter_Receive(frame, 18);
    assert(STOPFLAG && !motor_output_enable && SBUS_CH.ConnectState);
    frame[5] = 0x78; /* right middle: allow output; left remains up */
    Remoter_Receive(frame, 18);
    assert(!STOPFLAG && motor_output_enable && System_State == Normal);
    assert(SBUS_CH.CH1 == 992 && SBUS_CH.CH2 == 992);
    assert(SBUS_CH.CH3 == 992 && SBUS_CH.CH4 == 992);
    assert(Foot_Chassis.Target_Leg_State == 0); /* no initial edge */
    frame[5] = 0xF8; /* left middle, right middle */
    Remoter_Receive(frame, 18);
    frame[5] = 0x78; /* left up: long */
    Remoter_Receive(frame, 18);
    assert(Foot_Chassis.Target_Leg_State == 1);
    Remoter_Receive(frame, 18);
    assert(Foot_Chassis.Target_Leg_State == 1); /* hold does not toggle */
    leg_state_locked_short = 1;
    Remoter_Receive(frame, 18);
    assert(Foot_Chassis.Target_Leg_State == 0);
    frame[5] = 0xF8;
    Remoter_Receive(frame, 18);
    frame[5] = 0x78; /* toggle requested leg to short, release lock */
    Remoter_Receive(frame, 18);
    assert(!leg_state_locked_short && !Foot_Chassis.Target_Leg_State);
    frame[5] = 0xB8; /* left down: one stair event */
    Remoter_Receive(frame, 18);
    assert(upstairs_flag == 1);
    upstairs_flag = 0; /* motor task consumes event */
    Remoter_Receive(frame, 18);
    assert(upstairs_flag == 0);
    frame[5] = 0x98; /* right up: sit */
    Remoter_Receive(frame, 18);
    assert(sit_mode_enable && motor_output_enable);
    frame[5] = 0xB8; /* right middle: exit sit */
    Remoter_Receive(frame, 18);
    assert(!sit_mode_enable);
    assert(!spinning_enable && !jump_enable && !jump_cmd);
    /* Independent channels: right vertical negative, left horizontal positive. */
    const int16_t axes[4] = {0, -660, 660, 0};
    for (unsigned i = 0; i < 6; ++i) frame[i] = 0;
    for (unsigned i = 0; i < 4; ++i)
        for (unsigned bit = 0; bit < 11; ++bit)
            frame[(i * 11 + bit) / 8] |=
                (((unsigned)(axes[i] + 1024) >> bit) & 1U) << ((i * 11 + bit) % 8);
    frame[5] |= 0xF0; /* both middle=3 */
    Remoter_Receive(frame, 18);
    assert(DR16_Data.ch[1] == -660 && DR16_Data.ch[2] == 660);
    assert(SBUS_CH.CH2 == 192 && SBUS_CH.CH4 == 1792 && SBUS_CH.CH3 == 992);
    assert(DR16_SWITCH_UP == 1 && DR16_SWITCH_MIDDLE == 3 && DR16_SWITCH_DOWN == 2);
    tick = 99;
    Remoter_Heartbeat_Tick();
    assert(motor_output_enable);
    Remoter_Receive(frame, 17); /* malformed data must not refresh heartbeat */
    tick = 100;
    Remoter_Heartbeat_Tick();
    assert(STOPFLAG && !motor_output_enable && !SBUS_CH.ConnectState);
    Remoter_Receive(frame, 18);
    assert(!STOPFLAG && motor_output_enable);
    frame[5] = 0xA8;
    Remoter_Receive(frame, 18);
    assert(STOPFLAG && !motor_output_enable);
    return 0;
}
''')
    subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror',
                    '-I' + str(tmp), '-Iothers/Device/inc', '-Iothers',
                    'others/Device/src/remoter.c', 'others/Device/src/dr16.c',
                    str(tmp / 'test.c'), '-o', str(tmp / 'test')], cwd=root, check=True)
    subprocess.run([str(tmp / 'test')], check=True)
print('DR16 receive, switch edges, leg lock, sit, startup and timeout: passed')
