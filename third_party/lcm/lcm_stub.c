#include <stdlib.h>
#include "lcm/lcm.h"

struct _lcm_t {
    int unused;
};

struct _lcm_subscription_t {
    int unused;
};

lcm_t *lcm_create(const char *provider)
{
    (void)provider;
    return (lcm_t *)calloc(1, sizeof(lcm_t));
}

void lcm_destroy(lcm_t *lcm)
{
    free(lcm);
}

lcm_subscription_t *lcm_subscribe(lcm_t *lcm, const char *channel,
                                  lcm_msg_handler_t handler, void *userdata)
{
    (void)lcm;
    (void)channel;
    (void)handler;
    (void)userdata;
    return (lcm_subscription_t *)calloc(1, sizeof(lcm_subscription_t));
}

int lcm_unsubscribe(lcm_t *lcm, lcm_subscription_t *handler)
{
    (void)lcm;
    free(handler);
    return 0;
}

int lcm_publish(lcm_t *lcm, const char *channel, const void *data, unsigned int datalen)
{
    (void)lcm;
    (void)channel;
    (void)data;
    (void)datalen;
    return 0;
}

int lcm_handle(lcm_t *lcm)
{
    (void)lcm;
    return 0;
}

int lcm_handle_timeout(lcm_t *lcm, int timeout_millis)
{
    (void)lcm;
    (void)timeout_millis;
    return 0;
}

int lcm_subscription_set_queue_capacity(lcm_subscription_t *handler, int num_messages)
{
    (void)handler;
    (void)num_messages;
    return 0;
}

int lcm_subscription_get_queue_size(lcm_subscription_t *handler)
{
    (void)handler;
    return 0;
}
