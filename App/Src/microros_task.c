#include "app_tasks.h"
#include "usart.h"
#include "cmsis_os2.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/float64_multi_array.h>
#include <rcutils/allocator.h>
#include <rcutils/error_handling.h>

void * microros_allocate(size_t size, void * state);
void microros_deallocate(void * pointer, void * state);
void * microros_reallocate(void * pointer, size_t size, void * state);
void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);

bool cubemx_dmatransport_open(struct uxrCustomTransport * transport);
bool cubemx_dmatransport_close(struct uxrCustomTransport * transport);
size_t cubemx_dmatransport_write(struct uxrCustomTransport* transport, uint8_t * buf, size_t len, uint8_t * err);
size_t cubemx_dmatransport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

volatile AppTauCommand g_tau_cmd = {0};
volatile int g_microros_step;
volatile int g_microros_rc;

static void tau_cmd_callback(const void * msgin)
{
    const std_msgs__msg__Float64MultiArray * msg =
        (const std_msgs__msg__Float64MultiArray *)msgin;

    if (msg == NULL || msg->data.data == NULL) {
        return;
    }

    if (msg->data.size != 12) {
        return;
    }

    for (size_t i = 0; i < 12; i++) {
        g_tau_cmd.tau[i] = msg->data.data[i];
        HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
    }

    g_tau_cmd.valid = 1;
    g_tau_cmd.update_count++;

}

void StartMicroRosTask(void *argument)
{
    (void)argument;
    int flag = 1;
    if (flag == 0) {
        while (1) {
            HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
            osDelay(100);
        }
    }
    else {
        int g_rc_ = 0;

        rmw_uros_set_custom_transport(
            true,
            (void *)&huart7,
            cubemx_dmatransport_open,
            cubemx_dmatransport_close,
            cubemx_dmatransport_write,
            cubemx_dmatransport_read);

        rcutils_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
        freeRTOS_allocator.allocate = microros_allocate;
        freeRTOS_allocator.deallocate = microros_deallocate;
        freeRTOS_allocator.reallocate = microros_reallocate;
        freeRTOS_allocator.zero_allocate = microros_zero_allocate;

        if (!rcutils_set_default_allocator(&freeRTOS_allocator))
        {
            for (;;)
            {
                HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
                HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
                HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
                HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
                osDelay(200);
            }
        }

        rcl_allocator_t allocator = rcl_get_default_allocator();
        rclc_support_t support;
        rcl_node_t node;
        rcl_subscription_t subscriber;
        rclc_executor_t executor;

        std_msgs__msg__Float64MultiArray msg;
        std_msgs__msg__Float64MultiArray__init(&msg);

        static double tau_buffer[12];
        msg.data.data = tau_buffer;
        msg.data.size = 0;
        msg.data.capacity = 12;

        rcl_ret_t rc;

        rc = rclc_support_init(&support, 0, NULL, &allocator);
        g_rc_ = (int)rc;
        if (rc != RCL_RET_OK) {
            for(;;) {HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);osDelay(1000); }
        }

        rc = rclc_node_init_default(&node, "mcu_tau_subscriber", "", &support);
        g_rc_ = (int)rc;
        if (rc != RCL_RET_OK) {
            for(;;) {HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);osDelay(1000);}
        }

        rc = rclc_subscription_init_default(
            &subscriber,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
            "/real_joint_effort_controller/commands");
        if (rc != RCL_RET_OK) {
            for(;;) {HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);osDelay(1000);}
        }

        rc = rclc_executor_init(&executor, &support.context, 1, &allocator);
        if (rc != RCL_RET_OK) {
            for(;;) {HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);osDelay(1000);}
        }

        rc = rclc_executor_add_subscription(
            &executor,
            &subscriber,
            &msg,
            &tau_cmd_callback,
            ON_NEW_DATA);
        if (rc != RCL_RET_OK) {
            for(;;) {HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
                HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
                osDelay(1000); }
        }

        for (;;) {
            rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
            osDelay(1);
        }
    }
}
