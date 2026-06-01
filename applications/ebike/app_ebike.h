#ifndef APP_EBIKE_CONF_H_
#define APP_EBIKE_CONF_H_

#define APP_CUSTOM_TO_USE					"ebike/app_ebike.c"
#define APPCONF_APP_TO_USE					APP_CUSTOM

typedef enum {
	EBIKE_MODE_UNDEFINED = 0,
    EBIKE_MODE_LEGAL,
    EBIKE_MODE_UNRESTRICTED,
    EBIKE_MODE_UNRESTRICTED_PLUS
} EBIKE_MODE_T;

void app_ebike_set_mode(EBIKE_MODE_T mode);

#endif /* APP_EBIKE_CONF_H_ */