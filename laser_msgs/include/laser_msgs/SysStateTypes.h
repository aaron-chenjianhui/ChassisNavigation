#ifndef _SYS_STATE_TYPES_H
#define _SYS_STATE_TYPES_H

// node connection status
typedef enum {
	LIDAR_CONN	= 1,
	LIDAR_DISCONN	= 0
} lidar_conn_status_t;
typedef enum {
	TCP_CONN	= 1,
	TCP_DISCONN	= 0
} tcp_conn_status_t;
typedef enum {
	NAV_CONN	= 1,
	NAV_DISCONN	= 0
} nav_conn_status_t;

// detect node status
typedef enum {
	DETECT_CONN	= 1,
	DETECT_DISCONN	= 0
} detect_conn_status_t;
typedef enum {
	AMCL_CONN	= 1,
	AMCL_DISCONN	= 0
} amcl_conn_status_t;

// node status enum
typedef enum {
	DETECT_ERR		= 0,
	DETECT_READY		= 1,
	DETECT_PREPARATION	= 2,
	DETECT_CAL_ING		= 3,
	DETECT_CAL_OK		= 4,
} detect_status_t;

typedef enum {
	NAV_READY	= 0,
	NAV_CALIB	= 1,
	NAV_OK		= 2,
	NAV_ERR		= 3
} nav_status_t;

typedef enum {
	MSG_READY	= 0,
	MSG_GLOBAL	= 1,
	MSG_TRACK	= 2,
	NO_MSG		= 4
} msg_status_t;

// TODO(CJH): add error status
typedef enum {
	SYS_INIT	= 0,
	SYS_READY	= 1,
	SYS_GLOBAL_ING	= 2,
	SYS_GLOBAL_OK	= 3,
	SYS_TRACK	= 4
} sys_status_t;

// AMCL node status
typedef enum {
	AMCL_READY		= 0,
	AMCL_SEND_INIT		= 1,
	AMCL_SEND_UPDATE	= 2,
	AMCL_SEND_LOC		= 3,
	AMCL_DISCONN_STATUS	= 4,
	AMCL_ERR		= 5
} amcl_status_t;

#endif // ifndef _SYS_STATE_TYPES_H
