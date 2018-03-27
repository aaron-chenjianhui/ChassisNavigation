#ifndef _SYS_STATE_TYPES_H
#define _SYS_STATE_TYPES_H

// node connection status
typedef enum {
  lidar_conn    = 1,
  lidar_disconn = 0
} lidar_conn_status_t;
typedef enum {
  tcp_conn    = 1,
  tcp_disconn = 0
} tcp_conn_status_t;
typedef enum {
  nav_conn    = 1,
  nav_disconn = 0
} nav_conn_status_t;

// detect node status
typedef enum {
  detect_conn    = 1,
  detect_disconn = 0
} detect_conn_status_t;
typedef enum {
  amcl_conn    = 1,
  amcl_disconn = 0
} amcl_conn_status_t;

// node status enum
typedef enum {
  detect_ready   = 0,
  detect_cal_ing = 1,
  detect_cal_ok  = 2,
  detect_err     = 3
} detect_status_t;

typedef enum {
  nav_ready = 0,
  nav_calib = 1,
  nav_ok    = 2,
  nav_err   = 3
} nav_status_t;

typedef enum {
  msg_ready  = 0,
  msg_global = 1,
  msg_track  = 2,
  no_msg     = 4
} msg_status_t;

// TODO(CJH): add error status
typedef enum {
  sys_init       = 0,
  sys_ready      = 1,
  sys_global_ing = 2,
  sys_global_ok  = 3,
  sys_track      = 4
} sys_status_t;

// AMCL node status
typedef enum {
  amcl_ready          = 0,
  amcl_send_init      = 1,
  amcl_send_update    = 2,
  amcl_send_loc       = 3,
  amcl_disconn_status = 4,
  amcl_err            = 5
} amcl_status_t;

#endif // ifndef _SYS_STATE_TYPES_H
