/*
 * v4l2.c --
 *
 *      This file contains the implementation of the "v4l2" Tcl built-in
 *      command which allows to operate cameras using Video for Linux Two.
 *
 * Copyright (c) 2016-2025 Christian Werner <chw at ch minus werner dot de>
 *
 * See the file "license.terms" for information on usage and redistribution of
 * this file, and for a DISCLAIMER OF ALL WARRANTIES.
 */

#include <tk.h>
#include <string.h>
#include <fcntl.h>
#include <ctype.h>
#include <errno.h>
#include <sched.h>
#include <time.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/utsname.h>
#include <sys/stat.h>
#if defined(__OpenBSD__)
#include <sys/videoio.h>
#else
#include <linux/videodev2.h>
#endif
#include <dlfcn.h>
#ifdef HAVE_LIBUDEV
#include <libudev.h>
#endif
#ifdef USE_MJPEG
#include <jpeglib.h>
#include <setjmp.h>
#define V4L2_MJPEG_FAILED ((unsigned char *) -1)
#endif

/*
 * Missing stuff, depending on linux/videodev2.h
 */

#ifndef V4L2_PIX_FMT_Y10
#ifdef v4l2_fourcc
#define V4L2_PIX_FMT_Y10 v4l2_fourcc('Y', '1', '0', ' ')
#endif
#endif
#ifndef V4L2_PIX_FMT_Y16
#ifdef v4l2_fourcc
#define V4L2_PIX_FMT_Y16 v4l2_fourcc('Y', '1', '6', ' ')
#endif
#endif

/*
 * V4L2 frame buffer.
 */

typedef struct {
    void   *start;		/* Start address from mmap(). */
    size_t length;		/* Size of buffer. */
} VBUF;

/*
 * Structure describing V4L2 control.
 */

typedef struct {
    struct v4l2_queryctrl qry;	/* Filled from ioctl(). */
    int useOld;			/* Use old ioctl()s if positive. */
    Tcl_DString ds;		/* For menu choices. */
} VCTRL;

/*
 * Control structure for camera capture.
 */

typedef struct {
    int running;		/* Greater than zero when acquiring. */
    int stalled;		/* True when stalled in file handler. */
    int format;			/* Pixel format for capture. */
    int wantFormat;		/* Requested pixel format for capture. */
    int greyshift;		/* Bit shift for grey images. */
    int fd;			/* V4L2 file descriptor. */
    int isLoopDev;		/* True when loopback device. */
    int loopFormat;		/* Pixel format for writing. */
    Tcl_Interp *interp;		/* Interpreter for this object. */
    int mirror;			/* Image mirror flags. */
    int rotate;			/* Image rotation in degrees. */
    int bufrdy;			/* Index of last ready buffer or -1. */
    int bufdone;		/* True when buffer processed. */
    int width, height;		/* Width and height of frame buffers. */
    int loopWidth, loopHeight;	/* Ditto, for writing to loopback device. */
    int fps;			/* Frames per second. */
    char devId[32];		/* Device id. */
    Tcl_DString devName;	/* Device name. */
    int cbCmdLen;		/* Initial length of callback command. */
    Tcl_DString cbCmd;		/* Callback command prefix. */
    Tcl_HashTable ctrl;		/* V4L2 controls. */
    Tcl_HashTable nctrl;	/* V4L2 names to controls. */
    VCTRL fsize;		/* Special control: "frame-size". */
    VCTRL frate;		/* Special control: "frame-rate". */
    Tcl_WideInt counters[2];	/* Statistic counters. */
    int nvbufs;			/* Number of buffers used. */
    VBUF vbufs[16];		/* Frame buffers. */
} V4L2C;

/*
 * Per interpreter control structure.
 */

typedef struct {
    int idCount;
    int checkedTk;			/* Non-zero when Tk availability
					 * checked. */
    Tcl_HashTable v4l2c;		/* List of active V4L2C instances. */
#ifdef HAVE_LIBUDEV
    Tcl_Interp *interp;			/* Interpreter for this object. */
    Tcl_HashTable vdevs;		/* List of devices (udev). */
    int cbCmdLen;			/* Init. length of callback command. */
    Tcl_DString cbCmd;			/* Callback command prefix. */
    struct udev *udev;			/* udev instance. */
    struct udev_monitor *udevMon;	/* udev monitor. */
#endif
} V4L2I;

/*
 * Mutex and flag used during initialization etc.
 */

TCL_DECLARE_MUTEX(v4l2Mutex)
static int v4l2Initialized = 0;

/*
 * Stuff for dynamic linking libv4l2.so.
 */

static void *libv4l2 = NULL;

typedef int (*fn_open)(const char *, int, ...);
typedef int (*fn_close)(int);
typedef int (*fn_ioctl)(int, unsigned long, ...);
typedef void *(*fn_mmap)(void *, size_t, int, int, int, int64_t);
typedef int (*fn_munmap)(void *, size_t);

static struct {
    FILE **log_file;
    fn_open open;
    fn_close close;
    fn_ioctl ioctl;
    fn_mmap mmap;
    fn_munmap munmap;
} v4l2_dl = { NULL };

#define v4l2_log_file *v4l2_dl.log_file
#define v4l2_open v4l2_dl.open
#define v4l2_close v4l2_dl.close
#define v4l2_ioctl v4l2_dl.ioctl
#define v4l2_mmap v4l2_dl.mmap
#define v4l2_munmap v4l2_dl.munmap

/*
 * Make a format string to be appended to "width"x"height".
 */

static inline char *
fourcc_str(unsigned int fmt, char *buf)
{
    int i = 0;

    if (fmt) {
	fmt &= 0x7FFFFFFF;
	buf[i++] = '@';
	buf[i++] = fmt;
	buf[i++] = (fmt >> 8);
	buf[i++] = (fmt >> 16);
	buf[i++] = (fmt >> 24);
	while (i && (buf[i - 1] == ' ')) {
	    --i;
	}
    }
    buf[i++] = '\0';
    return buf;
}

/*
 * Supported formats for video devices in test order.
 */

static const int FormatsNormal[] = {
    V4L2_PIX_FMT_RGB24,
    V4L2_PIX_FMT_BGR24,
    V4L2_PIX_FMT_YUYV,
    V4L2_PIX_FMT_YVYU,
#ifdef USE_MJPEG
    V4L2_PIX_FMT_MJPEG,
#endif
#ifdef V4L2_PIX_FMT_Y16
    V4L2_PIX_FMT_Y16,
#endif
#ifdef V4L2_PIX_FMT_Y10
    V4L2_PIX_FMT_Y10,
#endif
    V4L2_PIX_FMT_GREY
};

/*
 * Supported formats for loopback video devices in test order.
 */

static const int FormatsLoop[] = {
    V4L2_PIX_FMT_RGB32,
    V4L2_PIX_FMT_BGR32,
    V4L2_PIX_FMT_RGB24,
    V4L2_PIX_FMT_BGR24,
    V4L2_PIX_FMT_YUYV,
    V4L2_PIX_FMT_YVYU,
    V4L2_PIX_FMT_GREY
};

#ifdef HAVE_LIBUDEV

/*
 * Stuff for dynamic linking libudev.so.
 * NB: mind the name bloat compared to libv4l2's.
 */

static void *libudev = NULL;

typedef const char *(*fn_device_get_action)(struct udev_device *);
typedef const char *(*fn_device_get_devnode)(struct udev_device *);
typedef struct udev_device *(*fn_device_new_from_syspath)
	(struct udev *, const char *);
typedef void (*fn_device_unref)(struct udev_device *);
typedef int (*fn_monitor_get_fd)(struct udev_monitor *);
typedef struct udev_device *(*fn_monitor_receive_device)
	(struct udev_monitor *);
typedef void (*fn_monitor_unref)(struct udev_monitor *);
typedef int (*fn_monitor_enable_receiving)(struct udev_monitor *);
typedef struct udev *(*fn_new)(void);
typedef void (*fn_unref)(struct udev *);
typedef int (*fn_monitor_filter_add_match_subsystem_devtype)
	(struct udev_monitor *, const char *, const char *);
typedef struct udev_monitor *(*fn_monitor_new_from_netlink)
	(struct udev *, const char *);
typedef struct udev_enumerate *(*fn_enumerate_new)(struct udev *);
typedef int (*fn_enumerate_add_match_subsystem)
	(struct udev_enumerate *, const char *);
typedef struct udev_list_entry *(*fn_enumerate_get_list_entry)
	(struct udev_enumerate *);
typedef int (*fn_enumerate_scan_devices)(struct udev_enumerate *);
typedef void (*fn_enumerate_unref)(struct udev_enumerate *);
typedef const char *(*fn_list_entry_get_name)(struct udev_list_entry *);
typedef struct udev_list_entry *(*fn_list_entry_get_next)
	(struct udev_list_entry *);

static struct {
    fn_device_get_action device_get_action;
    fn_device_get_devnode device_get_devnode;
    fn_device_new_from_syspath device_new_from_syspath;
    fn_device_unref device_unref;
    fn_monitor_get_fd monitor_get_fd;
    fn_monitor_receive_device monitor_receive_device;
    fn_monitor_unref monitor_unref;
    fn_monitor_enable_receiving monitor_enable_receiving;
    fn_new new;
    fn_unref unref;
    fn_monitor_filter_add_match_subsystem_devtype
	monitor_filter_add_match_subsystem_devtype;
    fn_monitor_new_from_netlink monitor_new_from_netlink;
    fn_enumerate_new enumerate_new;
    fn_enumerate_add_match_subsystem enumerate_add_match_subsystem;
    fn_enumerate_get_list_entry enumerate_get_list_entry;
    fn_enumerate_scan_devices enumerate_scan_devices;
    fn_enumerate_unref enumerate_unref;
    fn_list_entry_get_name list_entry_get_name;
    fn_list_entry_get_next list_entry_get_next;
} udev_dl;

#define udev_device_get_action udev_dl.device_get_action
#define udev_device_get_devnode udev_dl.device_get_devnode
#define udev_device_new_from_syspath udev_dl.device_new_from_syspath
#define udev_device_unref udev_dl.device_unref
#define udev_monitor_get_fd udev_dl.monitor_get_fd
#define udev_monitor_receive_device udev_dl.monitor_receive_device
#define udev_monitor_unref udev_dl.monitor_unref
#define udev_new udev_dl.new
#define udev_unref udev_dl.unref
#define udev_monitor_enable_receiving udev_dl.monitor_enable_receiving
#define udev_monitor_filter_add_match_subsystem_devtype \
    udev_dl.monitor_filter_add_match_subsystem_devtype
#define udev_monitor_new_from_netlink udev_dl.monitor_new_from_netlink
#define udev_enumerate_new udev_dl.enumerate_new
#define udev_enumerate_add_match_subsystem \
    udev_dl.enumerate_add_match_subsystem
#define udev_enumerate_get_list_entry udev_dl.enumerate_get_list_entry
#define udev_enumerate_scan_devices udev_dl.enumerate_scan_devices
#define udev_enumerate_unref udev_dl.enumerate_unref
#define udev_list_entry_get_name udev_dl.list_entry_get_name
#define udev_list_entry_get_next udev_dl.list_entry_get_next

#endif


#ifdef HAVE_LIBUDEV
/*
 *-------------------------------------------------------------------------
 *
 * UdevMonitor --
 *
 *	File handler for udev events. Depending on plug/unplug
 *	events, the table of devices is updated and the listen
 *	callback command is invoked.
 *
 *-------------------------------------------------------------------------
 */

static void
UdevMonitor(ClientData clientData, int mask)
{
    V4L2I *v4l2i = (V4L2I *) clientData;
    Tcl_Interp *interp = v4l2i->interp;
    struct udev_device *dev;
    Tcl_HashEntry *hPtr;
    const char *action, *devName;
    int isNew;

    if (!(mask & TCL_READABLE)) {
	return;
    }
    dev = udev_monitor_receive_device(v4l2i->udevMon);
    if (dev == NULL) {
	return;
    }
    action = udev_device_get_action(dev);
    devName = udev_device_get_devnode(dev);
    if (strcmp(action, "add") == 0) {
	hPtr = Tcl_CreateHashEntry(&v4l2i->vdevs, (ClientData) devName, &isNew);
	if (!isNew) {
	    action = NULL;
	}
	Tcl_SetHashValue(hPtr, (ClientData)
			 Tcl_GetHashKey(&v4l2i->vdevs, hPtr));
    } else if (strcmp(action, "remove") == 0) {
	hPtr = Tcl_FindHashEntry(&v4l2i->vdevs, (ClientData) devName);
	if (hPtr == NULL) {
	    action = NULL;
	} else {
	    Tcl_DeleteHashEntry(hPtr);
	}
    } else {
	action = NULL;
    }
    if ((v4l2i->cbCmdLen > 0) && (action != NULL) &&
	(interp != NULL) && !Tcl_InterpDeleted(interp)) {
	int ret;

	Tcl_DStringSetLength(&v4l2i->cbCmd, v4l2i->cbCmdLen);
	Tcl_DStringAppendElement(&v4l2i->cbCmd, action);
	Tcl_DStringAppendElement(&v4l2i->cbCmd, devName);
	Tcl_Preserve((ClientData) interp);
	ret = Tcl_EvalEx(interp, Tcl_DStringValue(&v4l2i->cbCmd),
			 Tcl_DStringLength(&v4l2i->cbCmd), TCL_EVAL_GLOBAL);
	if (ret != TCL_OK) {
	    Tcl_AddErrorInfo(interp, "\n    (v4l2 udev monitor)");
	    Tcl_BackgroundException(interp, ret);
	}
	Tcl_Release((ClientData) interp);
    }
    udev_device_unref(dev);
}
#endif

/*
 *-------------------------------------------------------------------------
 *
 * CheckForTk --
 *
 *	Check availability of Tk. Return standard Tcl error
 *	and appropriate error message if unavailable.
 *
 *-------------------------------------------------------------------------
 */

static int
CheckForTk(V4L2I *v4l2i, Tcl_Interp *interp)
{
    if (v4l2i->checkedTk > 0) {
	return TCL_OK;
    } else if (v4l2i->checkedTk < 0) {
	Tcl_SetResult(interp, "can't find package Tk", TCL_STATIC);
	return TCL_ERROR;
    }
#ifdef USE_TK_STUBS
    if (Tk_InitStubs(interp, "8.4", 0) == NULL) {
	v4l2i->checkedTk = -1;
	return TCL_ERROR;
    }
#else
    if (Tcl_PkgRequire(interp, "Tk", "8.4", 0) == NULL) {
	v4l2i->checkedTk = -1;
	return TCL_ERROR;
    }
#endif
    v4l2i->checkedTk = 1;
    return TCL_OK;
}

/*
 *-------------------------------------------------------------------------
 *
 * DoIoctl --
 *
 *	Wrapped EINTR safe v4l2_ioctl. Returns standard ioctl()
 *	result and sets errno variable on errors.
 *
 *-------------------------------------------------------------------------
 */

static int
DoIoctl(int fd, unsigned long cmd, void *arg)
{
    int ret;

    for (;;) {
	ret = v4l2_ioctl(fd, cmd, arg);
	if ((ret == -1) && (errno == EINTR)) {
	    sched_yield();
	    continue;
	}
	break;
    }
    return ret;
}

/*
 *-------------------------------------------------------------------------
 *
 * StopCapture --
 *
 *	Stop capture if running. Releases all frame buffers to
 *	allow to restart another capture. The file handler for
 *	the device is deleted, i.e. no further callbacks can
 *	be triggered.
 *
 *-------------------------------------------------------------------------
 */

static int
StopCapture(V4L2C *v4l2c)
{
    int i, type;
    struct v4l2_requestbuffers req;

    if (v4l2c->running > 0) {
	Tcl_DeleteFileHandler(v4l2c->fd);
	/* stop capture */
	type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	DoIoctl(v4l2c->fd, VIDIOC_STREAMOFF, &type);
	/* unmap buffers */
	for (i = 0; i < v4l2c->nvbufs; i++) {
	    v4l2_munmap(v4l2c->vbufs[i].start, v4l2c->vbufs[i].length);
	}
	/* release buffers */
	memset(&req, 0, sizeof (req));
	req.count = 0;
	req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	req.memory = V4L2_MEMORY_MMAP;
	DoIoctl(v4l2c->fd, VIDIOC_REQBUFS, &req);
	/* done */
	v4l2c->running = 0;
	v4l2c->stalled = 0;
	v4l2c->bufrdy = -1;
	v4l2c->bufdone = 0;
    }
    return TCL_OK;
}

/*
 *-------------------------------------------------------------------------
 *
 * BufferReady --
 *
 *	A frame buffer became ready as is indicated by readability
 *	of the device. This is the file handler procedure which
 *	reads out and remembers the frame buffer index. An already
 *	obtained older frame buffer is released before the Tcl
 *	callback is evaluated.
 *
 *-------------------------------------------------------------------------
 */

static void
BufferReady(ClientData clientData, int mask)
{
    V4L2C *v4l2c = (V4L2C *) clientData;
    Tcl_Interp *interp = v4l2c->interp;
    char buffer[64];
    struct v4l2_buffer vbuf;
    int ret, sequence;

    if (!(mask & TCL_READABLE)) {
	return;
    }
    memset(&vbuf, 0, sizeof (vbuf));
    vbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    vbuf.memory = V4L2_MEMORY_MMAP;
    if (DoIoctl(v4l2c->fd, VIDIOC_DQBUF, &vbuf) < 0) {
	if ((errno == EAGAIN) || (errno == EWOULDBLOCK)) {
	    const struct timespec ms10 = { 0, 10000000 };

	    /*
	     * Allow one more try through this file handler after
	     * a very short delay.
	     */
	    if (v4l2c->stalled) {
		goto captureError;
	    }
	    v4l2c->stalled = 1;
	    nanosleep(&ms10, NULL);
	    return;
	}
	goto captureError;
    }
    sequence = vbuf.sequence;
    v4l2c->stalled = 0;
    v4l2c->bufdone = 0;
    v4l2c->counters[0] += 1;
    if (v4l2c->bufrdy >= 0) {
	int swap = vbuf.index;

	vbuf.index = v4l2c->bufrdy;
	v4l2c->bufrdy = swap;
	if (DoIoctl(v4l2c->fd, VIDIOC_QBUF, &vbuf) < 0) {
captureError:
	    StopCapture(v4l2c);
	    v4l2c->running = -1;
	    v4l2c->stalled = 0;
	    Tcl_DStringSetLength(&v4l2c->cbCmd, v4l2c->cbCmdLen);
	    Tcl_DStringAppendElement(&v4l2c->cbCmd, v4l2c->devId);
	    Tcl_DStringAppendElement(&v4l2c->cbCmd, "error");
	    goto doCallback;
	}
    } else {
	v4l2c->bufrdy = vbuf.index;
    }

    Tcl_DStringSetLength(&v4l2c->cbCmd, v4l2c->cbCmdLen);
    Tcl_DStringAppendElement(&v4l2c->cbCmd, v4l2c->devId);
    sprintf(buffer, "%d", sequence);
    Tcl_DStringAppendElement(&v4l2c->cbCmd, buffer);
doCallback:
    Tcl_Preserve((ClientData) interp);
    ret = Tcl_EvalEx(interp, Tcl_DStringValue(&v4l2c->cbCmd),
		     Tcl_DStringLength(&v4l2c->cbCmd), TCL_EVAL_GLOBAL);
    if (ret != TCL_OK) {
	Tcl_AddErrorInfo(interp, "\n    (v4l2 event handler)");
	Tcl_BackgroundException(interp, ret);
	StopCapture(v4l2c);
    }
    Tcl_Release((ClientData) interp);
}

/*
 *-------------------------------------------------------------------------
 *
 * StartCapture --
 *
 *	Setup image acquisition:
 *	  - set capture format and size
 *	  - request frame buffers
 *	  - mmap() frame buffers
 *	  - queue buffers and start capture
 *	  - add file handler for buffer indications
 *
 *-------------------------------------------------------------------------
 */

static int
StartCapture(V4L2C *v4l2c)
{
    Tcl_Interp *interp = v4l2c->interp;
    int i, type;
    struct v4l2_format fmt;
    struct v4l2_buffer buf;
    struct v4l2_requestbuffers req;
    struct v4l2_streamparm stp;
    const int *tryFmts;
    int maxFmt;

    if (v4l2c->running > 0) {
	return TCL_OK;
    }
    if (v4l2c->isLoopDev) {
	tryFmts = FormatsLoop;
	maxFmt = sizeof (FormatsLoop) / sizeof (FormatsLoop[0]);
    } else {
	tryFmts = FormatsNormal;
	maxFmt = sizeof (FormatsNormal) / sizeof (FormatsNormal[0]);
    }

    /* set format/size */
    memset(&fmt, 0, sizeof (fmt));
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = v4l2c->width;
    fmt.fmt.pix.height = v4l2c->height;
    fmt.fmt.pix.field = V4L2_FIELD_ANY;
    if (v4l2c->wantFormat != 0) {
	for (i = 0; i < maxFmt; i++) {
	    if (v4l2c->wantFormat == tryFmts[i]) {
		break;
	    }
	}
	if (i < maxFmt) {
	    fmt.fmt.pix.pixelformat = v4l2c->wantFormat;
	    if (DoIoctl(v4l2c->fd, VIDIOC_S_FMT, &fmt) >= 0) {
		goto gotFormat;
	    }
	}
    }
    for (i = 0; i < maxFmt; i++) {
	fmt.fmt.pix.pixelformat = tryFmts[i];
	if (DoIoctl(v4l2c->fd, VIDIOC_S_FMT, &fmt) >= 0) {
	    break;
	}
    }
    if (i >= maxFmt) {
	Tcl_SetObjResult(interp,
			 Tcl_ObjPrintf("error setting format: %s",
				       Tcl_PosixError(interp)));
	v4l2c->running = -1;
	v4l2c->stalled = 0;
	return TCL_ERROR;
    }
    for (i = 0; i < maxFmt; i++) {
	if (fmt.fmt.pix.pixelformat == tryFmts[i]) {
	    break;
	}
    }
    if (i >= maxFmt) {
	Tcl_SetResult(interp, "unable to set supported pixel format",
		      TCL_STATIC);
	return TCL_ERROR;
    }
gotFormat:
    v4l2c->format = fmt.fmt.pix.pixelformat;
    if (v4l2c->wantFormat == 0) {
	v4l2c->wantFormat = v4l2c->format;
    }

    /* try to set frame rate */
    memset(&stp, 0, sizeof (stp));
    stp.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (DoIoctl(v4l2c->fd, VIDIOC_G_PARM, &stp) >= 0) {
	int compFps = 0;

	if (stp.parm.capture.capability & V4L2_CAP_TIMEPERFRAME) {
	    stp.parm.capture.timeperframe.numerator = 100;
	    stp.parm.capture.timeperframe.denominator = v4l2c->fps * 100;
	    if (DoIoctl(v4l2c->fd, VIDIOC_S_PARM, &stp) >= 0) {
		compFps = DoIoctl(v4l2c->fd, VIDIOC_G_PARM, &stp) != -1;
	    }
	}
	if (compFps && (stp.parm.capture.timeperframe.numerator > 0)) {
	    v4l2c->fps = stp.parm.capture.timeperframe.denominator /
		stp.parm.capture.timeperframe.numerator;
	    if (v4l2c->fps <= 0) {
		v4l2c->fps = 1;
	    }
	}
    }

    /* request buffers */
    memset(&req, 0, sizeof (req));
    i = 2;
    while (i < sizeof (v4l2c->vbufs) / sizeof (v4l2c->vbufs[0])) {
	req.count = i;
	req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	req.memory = V4L2_MEMORY_MMAP;
	if (DoIoctl(v4l2c->fd, VIDIOC_REQBUFS, &req) < 0) {
	    Tcl_SetObjResult(interp,
			     Tcl_ObjPrintf("error requesting buffers: %s",
					   Tcl_PosixError(interp)));
	    v4l2c->running = -1;
	    v4l2c->stalled = 0;
	    return TCL_ERROR;
	}
	if ((req.count <= 0) ||
	    (req.count > sizeof (v4l2c->vbufs) / sizeof (v4l2c->vbufs[0]))) {
	    i *= 2;
	    continue;
	}
	break;
    }
    if ((req.count <= 0) ||
	(req.count > sizeof (v4l2c->vbufs) / sizeof (v4l2c->vbufs[0]))) {
	Tcl_SetResult(interp, "unable to get buffers", TCL_STATIC);
	v4l2c->running = -1;
	v4l2c->stalled = 0;
	return TCL_ERROR;
    }

    /* mmap() buffers */
    for (i = 0; i < req.count; i++) {
	memset(&buf, 0, sizeof (buf));
	buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	buf.memory = V4L2_MEMORY_MMAP;
	buf.index = i;
	if (DoIoctl(v4l2c->fd, VIDIOC_QUERYBUF, &buf) < 0) {
	    Tcl_SetObjResult(interp,
			     Tcl_ObjPrintf("error querying buffer: %s",
					   Tcl_PosixError(interp)));
	    v4l2c->running = -1;
	    v4l2c->stalled = 0;
	    return TCL_ERROR;
	}
	v4l2c->vbufs[i].start = v4l2_mmap(NULL, buf.length,
					  PROT_READ | PROT_WRITE, MAP_SHARED,
					  v4l2c->fd, buf.m.offset);
	v4l2c->vbufs[i].length = buf.length;
	if (v4l2c->vbufs[i].start == MAP_FAILED) {
	    Tcl_SetObjResult(interp,
			     Tcl_ObjPrintf("error mapping buffer: %s",
					   Tcl_PosixError(interp)));
	    while (--i >= 0) {
		v4l2_munmap(v4l2c->vbufs[i].start, v4l2c->vbufs[i].length);
	    }
	    v4l2c->running = -1;
	    v4l2c->stalled = 0;
	    return TCL_ERROR;
	}
    }

    /* queue buffers */
    for (i = 0; i < req.count; i++) {
	memset(&buf, 0, sizeof (buf));
	buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	buf.memory = V4L2_MEMORY_MMAP;
	buf.index = i;
	if (DoIoctl(v4l2c->fd, VIDIOC_QBUF, &buf) < 0) {
	    Tcl_SetObjResult(interp,
			     Tcl_ObjPrintf("error querying buffer: %s",
					   Tcl_PosixError(interp)));
unmapAll:
	    for (i = 0; i < req.count; i++) {
		v4l2_munmap(v4l2c->vbufs[i].start, v4l2c->vbufs[i].length);
	    }
	    v4l2c->running = -1;
	    v4l2c->stalled = 0;
	    return TCL_ERROR;
	}
    }

    /* start capture */
    v4l2c->nvbufs = req.count;
    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (DoIoctl(v4l2c->fd, VIDIOC_STREAMON, &type) < 0) {
	Tcl_SetObjResult(interp,
			 Tcl_ObjPrintf("error starting capture: %s",
				       Tcl_PosixError(interp)));
	goto unmapAll;
    }

    /* setup file handler */
    Tcl_CreateFileHandler(v4l2c->fd, TCL_READABLE, BufferReady,
			  (ClientData) v4l2c);
    v4l2c->width = fmt.fmt.pix.width;
    v4l2c->height = fmt.fmt.pix.height;
    v4l2c->running = 1;
    v4l2c->stalled = 0;
    v4l2c->bufrdy = -1;
    v4l2c->bufdone = 0;
    v4l2c->counters[0] = v4l2c->counters[1] = 0;
    return TCL_OK;
}

/*
 *-------------------------------------------------------------------------
 *
 * FixupName --
 *
 *	Fixes control/parameter names using conventions from Android:
 *
 *	  - everything in lower case
 *	  - words separated by single dashes, e.g. "scene-mode"
 *
 *	The string is modified in-place.
 *
 *-------------------------------------------------------------------------
 */

static void
FixupName(char *name, int length)
{
    char *p, *end;

    p = name;
    end = name + ((length < 0) ? strlen(name) : length);
    while (p < end) {
	if (p[0] >= '\0') {
	    p[0] = tolower(p[0]);
	}
	if (p[0] > '\0') {
	    if (strchr(" .,/_+(){}[]=&%$:;'#*~", p[0]) != NULL) {
		p[0] = '-';
	    }
	}
	++p;
    }
    if (length > 0) {
	name[length - 1] = '\0';
    } else {
	++end;
    }
    p = name;
    while ((p = strchr(p, '-')) != NULL) {
	if (p[1] == '-') {
	    memmove(p, p + 1, end - p);
	} else if (p[1] == '\0') {
	    p[0] = '\0';
	    break;
	} else {
	    ++p;
	}
    }
}

/*
 *-------------------------------------------------------------------------
 *
 * InitControls --
 *
 *	Fill (or release) V4L2C control structure with meta information
 *	about the device's controls.
 *
 *-------------------------------------------------------------------------
 */

static void
InitControls(V4L2C *v4l2c)
{
    long id;
    int i, k, numFS, isNew;
    Tcl_HashEntry *hPtr;
    Tcl_HashSearch search;
    struct v4l2_queryctrl qry;
    struct v4l2_querymenu qmenu;
    VCTRL *vctrl;
    char buffer[128], fcbuf[8];
#ifdef VIDIOC_ENUM_FRAMESIZES
    const int *tryFmts;
    int maxFmt;
    Tcl_HashTable fmtTab;
#endif

    /* first, free up old stuff */
    hPtr = Tcl_FirstHashEntry(&v4l2c->ctrl, &search);
    while (hPtr != NULL) {
	vctrl = (VCTRL *) Tcl_GetHashValue(hPtr);
	Tcl_DStringFree(&vctrl->ds);
	if ((vctrl != &v4l2c->fsize) && (vctrl != &v4l2c->frate)) {
	    ckfree((char *) vctrl);
	}
	hPtr = Tcl_NextHashEntry(&search);
    }
    Tcl_DeleteHashTable(&v4l2c->nctrl);
    Tcl_InitHashTable(&v4l2c->nctrl, TCL_STRING_KEYS);
    Tcl_DStringFree(&v4l2c->fsize.ds);
    Tcl_DStringFree(&v4l2c->frate.ds);

    /* done, when there's no opened device */
    if (v4l2c->fd < 0) {
	return;
    }

    /* fill in new information: base controls */
    for (id = V4L2_CID_BASE; id < V4L2_CID_LASTP1; id++) {
	memset(&qry, 0, sizeof (qry));
	qry.id = id;
	if ((DoIoctl(v4l2c->fd, VIDIOC_QUERYCTRL, &qry) != -1) &&
	    ((qry.type == V4L2_CTRL_TYPE_INTEGER) ||
	     (qry.type == V4L2_CTRL_TYPE_BOOLEAN) ||
	     (qry.type == V4L2_CTRL_TYPE_MENU) ||
	     (qry.type == V4L2_CTRL_TYPE_BUTTON) ||
	     (qry.type == V4L2_CTRL_TYPE_INTEGER64)) &&
	    !(qry.flags & V4L2_CTRL_FLAG_DISABLED)) {
	    FixupName((char *) qry.name, sizeof (qry.name));
	    if ((strcmp((char *) qry.name, "frame-size") == 0) ||
		(strcmp((char *) qry.name, "frame-rate") == 0)) {
		/* these names are reserved */
		continue;
	    }
	    hPtr = Tcl_CreateHashEntry(&v4l2c->ctrl, (ClientData) id, &isNew);
	    if (isNew) {
		vctrl = (VCTRL *) ckalloc(sizeof (VCTRL));
		Tcl_DStringInit(&vctrl->ds);
		Tcl_SetHashValue(hPtr, (ClientData) vctrl);
		vctrl->useOld = (qry.type != V4L2_CTRL_TYPE_INTEGER64) ? -1 : 0;
	    } else {
		vctrl = (VCTRL*) Tcl_GetHashValue(hPtr);
		Tcl_DStringSetLength(&vctrl->ds, 0);
	    }
	    vctrl->qry = qry;
	    if (qry.type == V4L2_CTRL_TYPE_MENU) {
		for (i = qry.minimum; i <= qry.maximum; i++) {
		    memset(&qmenu, 0, sizeof (qmenu));
		    qmenu.id = qry.id;
		    qmenu.index = i;
		    if (DoIoctl(v4l2c->fd, VIDIOC_QUERYMENU, &qmenu) < 0) {
			/* force empty menu entry */
			strcpy((char *) qmenu.name, ",");
		    } else {
			FixupName((char *) qmenu.name, sizeof (qmenu.name));
			if ((qmenu.name[0] == '\0') ||
			    (strcmp((char *) qmenu.name, "-") == 0)) {
			    sprintf((char *) qmenu.name, "%d", i);
			}
		    }
		    Tcl_DStringAppend(&vctrl->ds, (char *) qmenu.name, -1);
		    Tcl_DStringAppend(&vctrl->ds, "\0", 1);
		}
	    }
	}
    }

    /* older kernels lack this */
#ifndef V4L2_CTRL_CLASS_CAMERA
#define V4L2_CTRL_CLASS_CAMERA (V4L2_CTRL_CLASS_USER + 0x00020000)
#endif

    /* fill in new information: camera controls */
    for (id = V4L2_CTRL_CLASS_CAMERA | V4L2_CTRL_FLAG_NEXT_CTRL;;) {
	memset(&qry, 0, sizeof (qry));
	qry.id = id;
	if (DoIoctl(v4l2c->fd, VIDIOC_QUERYCTRL, &qry) == -1) {
	    break;
	}
	if (V4L2_CTRL_ID2CLASS(qry.id) != V4L2_CTRL_CLASS_CAMERA) {
	    break;
	}
	if (((qry.type == V4L2_CTRL_TYPE_INTEGER) ||
	     (qry.type == V4L2_CTRL_TYPE_BOOLEAN) ||
	     (qry.type == V4L2_CTRL_TYPE_MENU) ||
	     (qry.type == V4L2_CTRL_TYPE_BUTTON) ||
	     (qry.type == V4L2_CTRL_TYPE_INTEGER64)) &&
	    !(qry.flags & V4L2_CTRL_FLAG_DISABLED)) {
	    FixupName((char *) qry.name, sizeof (qry.name));
	    if ((strcmp((char *) qry.name, "frame-size") == 0) ||
		(strcmp((char *) qry.name, "frame-rate") == 0)) {
		/* these names are reserved */
		continue;
	    }
	    hPtr = Tcl_CreateHashEntry(&v4l2c->ctrl, (ClientData) id, &isNew);
	    if (isNew) {
		vctrl = (VCTRL *) ckalloc(sizeof (VCTRL));
		Tcl_DStringInit(&vctrl->ds);
		Tcl_SetHashValue(hPtr, (ClientData) vctrl);
		vctrl->useOld = 0;
	    } else {
		vctrl = (VCTRL*) Tcl_GetHashValue(hPtr);
		Tcl_DStringSetLength(&vctrl->ds, 0);
	    }
	    vctrl->qry = qry;
	    if (qry.type == V4L2_CTRL_TYPE_MENU) {
		for (i = qry.minimum; i <= qry.maximum; i++) {
		    memset(&qmenu, 0, sizeof (qmenu));
		    qmenu.id = qry.id;
		    qmenu.index = i;
		    if (DoIoctl(v4l2c->fd, VIDIOC_QUERYMENU, &qmenu) < 0) {
			/* force empty menu entry */
			strcpy((char *) qmenu.name, ",");
		    } else {
			FixupName((char *) qmenu.name, sizeof (qmenu.name));
			if ((qmenu.name[0] == '\0') ||
			    (strcmp((char *) qmenu.name, "-") == 0)) {
			    sprintf((char *) qmenu.name, "%d", i);
			}
		    }
		    Tcl_DStringAppend(&vctrl->ds, (char *) qmenu.name, -1);
		    Tcl_DStringAppend(&vctrl->ds, "\0", 1);
		}
	    }
	}
	id = qry.id | V4L2_CTRL_FLAG_NEXT_CTRL;
    }

    /* fill string mapping */
    hPtr = Tcl_FirstHashEntry(&v4l2c->ctrl, &search);
    while (hPtr != NULL) {
	Tcl_HashEntry *hPtr2;

	vctrl = (VCTRL *) Tcl_GetHashValue(hPtr);
	hPtr2 = Tcl_CreateHashEntry(&v4l2c->nctrl, (char *) vctrl->qry.name,
				    &isNew);
	Tcl_SetHashValue(hPtr2, (ClientData) vctrl);
	hPtr = Tcl_NextHashEntry(&search);
    }

    /* frame-size pseudo menu control */
    v4l2c->fsize.qry.type = V4L2_CTRL_TYPE_MENU;
    strcpy((char *) v4l2c->fsize.qry.name, "frame-size");
    numFS = 0;
#ifdef VIDIOC_ENUM_FRAMESIZES
    Tcl_InitHashTable(&fmtTab, TCL_STRING_KEYS);
    if (v4l2c->isLoopDev) {
	tryFmts = FormatsLoop;
	maxFmt = sizeof (FormatsLoop) / sizeof (FormatsLoop[0]);
    } else {
	tryFmts = FormatsNormal;
	maxFmt = sizeof (FormatsNormal) / sizeof (FormatsNormal[0]);
    }
    for (k = 0; k < maxFmt; k++) {
	for (i = 0; i >= 0; i++) {
	    struct v4l2_frmsizeenum qfsz;

	    memset(&qfsz, 0, sizeof (qfsz));
	    qfsz.index = i;
	    qfsz.pixel_format = tryFmts[k];
	    if (DoIoctl(v4l2c->fd, VIDIOC_ENUM_FRAMESIZES, &qfsz) < 0) {
		break;
	    }
	    if (qfsz.pixel_format != tryFmts[k]) {
		goto sizesDone;
	    }
	    switch (qfsz.type) {
	    case V4L2_FRMSIZE_TYPE_DISCRETE:
		sprintf(buffer, "%dx%d%s", qfsz.discrete.width,
			qfsz.discrete.height,
			fourcc_str(qfsz.pixel_format, fcbuf));
		Tcl_CreateHashEntry(&fmtTab, buffer, &isNew);
		if (isNew) {
		    Tcl_DStringAppend(&v4l2c->fsize.ds, buffer, -1);
		    Tcl_DStringAppend(&v4l2c->fsize.ds, "\0", 1);
		    numFS++;
		}
		break;
	    case V4L2_FRMSIZE_TYPE_STEPWISE:
		while (1) {
		    sprintf(buffer, "%dx%d%s", qfsz.stepwise.min_width,
			    qfsz.stepwise.min_height,
			    fourcc_str(qfsz.pixel_format, fcbuf));
		    Tcl_CreateHashEntry(&fmtTab, buffer, &isNew);
		    if (isNew) {
			Tcl_DStringAppend(&v4l2c->fsize.ds, buffer, -1);
			Tcl_DStringAppend(&v4l2c->fsize.ds, "\0", 1);
			numFS++;
		    }
		    qfsz.stepwise.min_width += qfsz.stepwise.step_width;
		    qfsz.stepwise.min_height += qfsz.stepwise.step_height;
		    if ((qfsz.stepwise.min_width >= qfsz.stepwise.max_width) ||
			(qfsz.stepwise.min_height >= qfsz.stepwise.max_height)) {
			break;
		    }
		}
		goto sizesDone;
	    case V4L2_FRMSIZE_TYPE_CONTINUOUS:
		sprintf(buffer, "+%dx%d%s", qfsz.stepwise.min_width,
			qfsz.stepwise.min_height,
			fourcc_str(qfsz.pixel_format, fcbuf));
		Tcl_CreateHashEntry(&fmtTab, buffer, &isNew);
		if (isNew) {
		    Tcl_DStringAppend(&v4l2c->fsize.ds, buffer, -1);
		    Tcl_DStringAppend(&v4l2c->fsize.ds, "\0", 1);
		    sprintf(buffer, "+%dx%d%s", qfsz.stepwise.max_width,
			    qfsz.stepwise.max_height,
			    fourcc_str(qfsz.pixel_format, fcbuf));
		    Tcl_CreateHashEntry(&fmtTab, buffer, &isNew);
		    Tcl_DStringAppend(&v4l2c->fsize.ds, buffer, -1);
		    Tcl_DStringAppend(&v4l2c->fsize.ds, "\0", 1);
		    numFS += 2;
		}
		/* FALLTHROUGH */
	    default:
		goto sizesDone;
	    }
	}
sizesDone:
	;
    }
    Tcl_DeleteHashTable(&fmtTab);
#endif
    if (numFS == 0) {
	/* fallback to last set format */
	sprintf(buffer, "%dx%d%s", v4l2c->width, v4l2c->height,
		fourcc_str(v4l2c->format, fcbuf));
	Tcl_DStringAppend(&v4l2c->fsize.ds, buffer, -1);
	Tcl_DStringAppend(&v4l2c->fsize.ds, "\0", 1);
	numFS++;
    }
    v4l2c->fsize.qry.minimum = 0;
    v4l2c->fsize.qry.maximum = numFS - 1;
    id = v4l2c->fsize.qry.id;	/* special, is 0x00000000 */
    hPtr = Tcl_CreateHashEntry(&v4l2c->ctrl, (ClientData) id, &isNew);
    Tcl_SetHashValue(hPtr, (ClientData) &v4l2c->fsize);
    hPtr = Tcl_CreateHashEntry(&v4l2c->nctrl, "frame-size", &isNew);
    Tcl_SetHashValue(hPtr, (ClientData) &v4l2c->fsize);

    /* frame-rate pseudo control */
    v4l2c->frate.qry.type = V4L2_CTRL_TYPE_INTEGER;
    v4l2c->frate.qry.id = 1;
    strcpy((char *) v4l2c->frate.qry.name, "frame-rate");
    v4l2c->frate.qry.minimum = 1;
    v4l2c->frate.qry.maximum = 200;
    v4l2c->frate.qry.default_value = 15;
    v4l2c->frate.qry.step = 1;
    id = v4l2c->frate.qry.id;	/* special, is 0x00000001 */
    hPtr = Tcl_CreateHashEntry(&v4l2c->ctrl, (ClientData) id, &isNew);
    Tcl_SetHashValue(hPtr, (ClientData) &v4l2c->frate);
    hPtr = Tcl_CreateHashEntry(&v4l2c->nctrl, "frame-rate", &isNew);
    Tcl_SetHashValue(hPtr, (ClientData) &v4l2c->frate);
}

/*
 *-------------------------------------------------------------------------
 *
 * GetZZString --
 *
 *	Given index return corresponding string from a double zero
 *	terminated string list.
 *
 *-------------------------------------------------------------------------
 */

static char *
GetZZString(char *string, int index)
{
    int i = 0;

    while (i < index) {
	if (string[0] == '\0') {
	    return string;
	}
	++i;
	string += strlen(string) + 1;
    }
    if (string[0] == ',') {
	++string;
    }
    return string;
}

/*
 *-------------------------------------------------------------------------
 *
 * GetControls --
 *
 *	Read out current values of device controls and return
 *	these as list made up of key value pairs. Added meta
 *	information entries to support user interface:
 *
 *	  <name>-values    comma separated list of menu choices
 *	  <name>-minimum   minimum value for bool/integer
 *	  <name>-maximum   minimum value for bool/integer
 *	  <name>-step      interval step value for integer
 *	  <name>-default   default value for bool/integer
 *
 *-------------------------------------------------------------------------
 */

static void
GetControls(V4L2C *v4l2c, Tcl_Obj *list)
{
    Tcl_HashEntry *hPtr;
    Tcl_HashSearch search;

    hPtr = Tcl_FirstHashEntry(&v4l2c->ctrl, &search);
    while (hPtr != NULL) {
	VCTRL *vctrl = (VCTRL *) Tcl_GetHashValue(hPtr);
	struct v4l2_ext_controls xs;
	struct v4l2_ext_control xc;
	Tcl_Obj *obj;
	char *p;
	int i, k;

	Tcl_ListObjAppendElement(NULL, list,
		Tcl_NewStringObj((char *) vctrl->qry.name, -1));
	memset(&xs, 0, sizeof (xs));
	xs.ctrl_class = V4L2_CTRL_ID2CLASS(vctrl->qry.id);
	xs.count = 1;
	xs.error_idx = 0;
	xs.controls = &xc;
	memset(&xc, 0, sizeof (xc));
	xc.id = vctrl->qry.id;
	if (vctrl == &v4l2c->fsize) {
	    goto doVal;
	}
	if (vctrl == &v4l2c->frate) {
	    xc.value = v4l2c->fps;
	    goto doVal;
	}

	/* older kernels lack this */
#ifndef V4L2_CTRL_FLAG_WRITE_ONLY
#define V4L2_CTRL_FLAG_WRITE_ONLY 0x0040
#endif

	if (!(vctrl->qry.flags & V4L2_CTRL_FLAG_WRITE_ONLY)) {
	    if (vctrl->useOld > 0) {
		struct v4l2_control xd;

		xd.id = xc.id;
		xd.value = 0;
		if (DoIoctl(v4l2c->fd, VIDIOC_G_CTRL, &xd) != -1) {
		    xc.value = xd.value;
		}
	    } else if (DoIoctl(v4l2c->fd, VIDIOC_G_EXT_CTRLS, &xs) < 0) {
		if ((errno == EINVAL) && (vctrl->useOld < 0)) {
		    struct v4l2_control xd;

		    /* retry with old ioctl() command */
		    xd.id = xc.id;
		    xd.value = 0;
		    if (DoIoctl(v4l2c->fd, VIDIOC_G_CTRL, &xd) != -1) {
			xc.value = xd.value;
			vctrl->useOld = 1;
		    }
		}
	    }
	}
doVal:
	switch (vctrl->qry.type) {
	case V4L2_CTRL_TYPE_INTEGER:
	    Tcl_ListObjAppendElement(NULL, list, Tcl_NewIntObj(xc.value));
	    obj = Tcl_NewStringObj((char *) vctrl->qry.name, -1);
	    Tcl_AppendToObj(obj, "-minimum", -1);
	    Tcl_ListObjAppendElement(NULL, list, obj);
	    Tcl_ListObjAppendElement(NULL, list,
				     Tcl_NewIntObj(vctrl->qry.minimum));
	    obj = Tcl_NewStringObj((char *) vctrl->qry.name, -1);
	    Tcl_AppendToObj(obj, "-maximum", -1);
	    Tcl_ListObjAppendElement(NULL, list, obj);
	    Tcl_ListObjAppendElement(NULL, list,
				     Tcl_NewIntObj(vctrl->qry.maximum));
	    obj = Tcl_NewStringObj((char *) vctrl->qry.name, -1);
	    Tcl_AppendToObj(obj, "-default", -1);
	    Tcl_ListObjAppendElement(NULL, list, obj);
	    Tcl_ListObjAppendElement(NULL, list,
				     Tcl_NewIntObj(vctrl->qry.default_value));
	    obj = Tcl_NewStringObj((char *) vctrl->qry.name, -1);
	    Tcl_AppendToObj(obj, "-step", -1);
	    Tcl_ListObjAppendElement(NULL, list, obj);
	    Tcl_ListObjAppendElement(NULL, list,
				     Tcl_NewIntObj(vctrl->qry.step));
	    break;
	case V4L2_CTRL_TYPE_BOOLEAN:
	    Tcl_ListObjAppendElement(NULL, list, Tcl_NewIntObj(xc.value != 0));
	    obj = Tcl_NewStringObj((char *) vctrl->qry.name, -1);
	    Tcl_AppendToObj(obj, "-minimum", -1);
	    Tcl_ListObjAppendElement(NULL, list, obj);
	    Tcl_ListObjAppendElement(NULL, list, Tcl_NewIntObj(0));
	    obj = Tcl_NewStringObj((char *) vctrl->qry.name, -1);
	    Tcl_AppendToObj(obj, "-maximum", -1);
	    Tcl_ListObjAppendElement(NULL, list, obj);
	    Tcl_ListObjAppendElement(NULL, list, Tcl_NewIntObj(1));
	    obj = Tcl_NewStringObj((char *) vctrl->qry.name, -1);
	    Tcl_AppendToObj(obj, "-default", -1);
	    Tcl_ListObjAppendElement(NULL, list, obj);
	    Tcl_ListObjAppendElement(NULL, list,
				     Tcl_NewIntObj(vctrl->qry.default_value));
	    break;
	case V4L2_CTRL_TYPE_INTEGER64:
	    Tcl_ListObjAppendElement(NULL, list, Tcl_NewWideIntObj(xc.value64));
	    break;
	case V4L2_CTRL_TYPE_BUTTON:
	    Tcl_ListObjAppendElement(NULL, list, Tcl_NewObj());
	    break;
	case V4L2_CTRL_TYPE_MENU:
	    p = Tcl_DStringValue(&vctrl->ds);
	    if (vctrl == &v4l2c->fsize) {
		int format;
		char buffer[128], fcbuf[8];

		format = v4l2c->running ? v4l2c->format : v4l2c->wantFormat;
		sprintf(buffer, "%dx%d%s", v4l2c->width, v4l2c->height,
			fourcc_str(format, fcbuf));
		Tcl_ListObjAppendElement(NULL, list,
		    Tcl_NewStringObj(buffer, -1));
	    } else {
		Tcl_ListObjAppendElement(NULL, list,
		    Tcl_NewStringObj(GetZZString(p, xc.value), -1));
	    }
	    obj = Tcl_NewStringObj((char *) vctrl->qry.name, -1);
	    Tcl_AppendToObj(obj, "-values", -1);
	    Tcl_ListObjAppendElement(NULL, list, obj);
	    obj = Tcl_NewStringObj("", 0);
	    k = 0;
	    for (i = 0; i <= vctrl->qry.maximum - vctrl->qry.minimum; i++) {
		char *entry = GetZZString(p, i);

		if (k > 0) {
		    Tcl_AppendToObj(obj, ",", 1);
		}
		if (entry[0] != '\0') {
		    Tcl_AppendToObj(obj, entry, -1);
		    k++;
		}
	    }
	    Tcl_ListObjAppendElement(NULL, list, obj);
	    break;
	default: /* V4L2_CTRL_TYPE_CTRL_CLASS, V4L2_CTRL_TYPE_STRING */
	    /* should not happen */
	    Tcl_ListObjAppendElement(NULL, list, Tcl_NewObj());
	    break;
	}
	hPtr = Tcl_NextHashEntry(&search);
    }
}

/*
 *-------------------------------------------------------------------------
 *
 * SetControls --
 *
 *	Set device controls given list of key value pairs.
 *
 *-------------------------------------------------------------------------
 */

static int
SetControls(V4L2C *v4l2c, int objc, Tcl_Obj * const objv[])
{
    Tcl_Interp *interp = v4l2c->interp;
    Tcl_HashEntry *hPtr;
    int i, k;

    for (i = 0; i < objc; i += 2) {
	VCTRL *vctrl;
	struct v4l2_ext_controls xs;
	struct v4l2_ext_control xc;
	int ival;
	Tcl_WideInt wval;

	hPtr = Tcl_FindHashEntry(&v4l2c->nctrl, Tcl_GetString(objv[i]));
	if (hPtr == NULL) {
	    continue;
	}
	vctrl = (VCTRL *) Tcl_GetHashValue(hPtr);
	if ((vctrl->qry.flags & V4L2_CTRL_FLAG_READ_ONLY) ||
	    (vctrl->qry.flags & V4L2_CTRL_FLAG_INACTIVE)) {
	    continue;
	}
	if (vctrl == &v4l2c->fsize) {
	    int w = -1, h = -1;
	    const char *fmt = Tcl_GetString(objv[i + 1]);

	    if ((sscanf(fmt, "%dx%d", &w, &h) == 2) &&
		(w > 0) && (h > 0) && (v4l2c->running <= 0)) {
		v4l2c->width = w;
		v4l2c->height = h;
		v4l2c->wantFormat = 0;
		fmt = strchr(fmt, '@');
		if ((fmt != NULL) && (strlen(fmt) > 1)) {
		    char fcbuf[4];

		    memset(fcbuf, ' ', 4);
		    fmt++;
		    k = 0;
		    while ((k < 4) && (*fmt != '\0')) {
			fcbuf[k] = *fmt++;
			k++;
		    }
		    v4l2c->wantFormat =
			v4l2_fourcc(fcbuf[0], fcbuf[1], fcbuf[2], fcbuf[3]);
		}
	    }
	    continue;
	}
	if (vctrl == &v4l2c->frate) {
	    int fps;

	    if (Tcl_GetIntFromObj(NULL, objv[i + 1], &fps) != TCL_OK) {
		/* ignored */
		continue;
	    }
	    if ((fps > 0) && (fps < 200)) {
		v4l2c->fps = fps;
	    }
	    continue;
	}
	memset(&xs, 0, sizeof (xs));
	xs.ctrl_class = V4L2_CTRL_ID2CLASS(vctrl->qry.id);
	xs.count = 1;
	xs.error_idx = 0;
	xs.controls = &xc;
	memset(&xc, 0, sizeof (xc));
	xc.id = vctrl->qry.id;
	switch (vctrl->qry.type) {
	case V4L2_CTRL_TYPE_INTEGER:
	case V4L2_CTRL_TYPE_BOOLEAN:
	    if (Tcl_GetIntFromObj(NULL, objv[i + 1], &ival) != TCL_OK) {
		/* ignored */
		continue;
	    }
	    xc.value = ival;
	    break;
	case V4L2_CTRL_TYPE_INTEGER64:
	    if (Tcl_GetWideIntFromObj(NULL, objv[i + 1], &wval) != TCL_OK) {
		/* ignored */
		continue;
	    }
	    xc.value64 = wval;
	    break;
	case V4L2_CTRL_TYPE_BUTTON:
	    /* value ignored */
	    break;
	case V4L2_CTRL_TYPE_MENU:
	    for (k = 0; k <= vctrl->qry.maximum - vctrl->qry.minimum; k++) {
		char *entry = GetZZString(Tcl_DStringValue(&vctrl->ds), k);

		/* ignore empty entries */
		if ((entry[0] != '\0') &&
		    (strcmp(entry, Tcl_GetString(objv[i + 1])) == 0)) {
		    xc.value = k + vctrl->qry.minimum;
		    break;
		}
	    }
	    if (k > vctrl->qry.maximum - vctrl->qry.minimum) {
		/* ignored */
		continue;
	    }
	    break;
	default: /* V4L2_CTRL_TYPE_CTRL_CLASS, V4L2_CTRL_TYPE_STRING */
	    /* should not happen, ignored */
	    continue;
	}
	if (vctrl->useOld > 0) {
	    struct v4l2_control xd;

	    xd.id = xc.id;
	    xd.value = xc.value;
	    if (DoIoctl(v4l2c->fd, VIDIOC_S_CTRL, &xd) < 0) {
		goto errorSet;
	    }
	} else if (DoIoctl(v4l2c->fd, VIDIOC_S_EXT_CTRLS, &xs) < 0) {
	    if ((errno == EINVAL) && (vctrl->useOld < 0)) {
		struct v4l2_control xd;

		/* retry with old ioctl() command */
		xd.id = xc.id;
		xd.value = xc.value;
		if (DoIoctl(v4l2c->fd, VIDIOC_S_CTRL, &xd) == 0) {
		    vctrl->useOld = 1;
		    continue;
		}
	    }
errorSet:
	    Tcl_SetErrno(errno);
	    Tcl_SetObjResult(interp,
			 Tcl_ObjPrintf("error setting \"%s\": %s",
				       Tcl_GetString(objv[i]),
				       Tcl_PosixError(interp)));
	    return TCL_ERROR;
	}
    }
    return TCL_OK;
}

/*
 *-------------------------------------------------------------------------
 *
 * ConvertFromYUV, ConvertToYUV, ConvertToGREY --
 *
 *	Perform colorspace conversions between YUYV/YVYU and RGB etc.
 *
 *-------------------------------------------------------------------------
 */

static inline unsigned char sat(int i)
{
    return (unsigned char) ((i >= 255) ? 255 : ((i < 0) ? 0 : i));
}

static unsigned char *
ConvertFromYUV(unsigned char *in, int width, int height, int isvu)
{
    unsigned char *out, *beg, *end;
    int r, g, b;

    out = attemptckalloc(width * height * 3);
    if (out == NULL) {
	return NULL;
    }
    beg = out;
    end = beg + width * height * 3;
    if (isvu) {
	while (beg < end) {
	    r = (22987 * (in[1] - 128)) >> 14;
	    g = (-5636 * (in[3] - 128) - 11698 * (in[1] - 128)) >> 14;
	    b = (29049 * (in[3] - 128)) >> 14;
	    beg[0] = sat(in[0] + r);
	    beg[1] = sat(in[0] + g);
	    beg[2] = sat(in[0] + b);
	    beg[3] = sat(in[2] + r);
	    beg[4] = sat(in[2] + g);
	    beg[5] = sat(in[2] + b);
	    beg += 6;
	    in += 4;
	}
    } else {
	while (beg < end) {
	    r = (22987 * (in[3] - 128)) >> 14;
	    g = (-5636 * (in[1] - 128) - 11698 * (in[3] - 128)) >> 14;
	    b = (29049 * (in[1] - 128)) >> 14;
	    beg[0] = sat(in[0] + r);
	    beg[1] = sat(in[0] + g);
	    beg[2] = sat(in[0] + b);
	    beg[3] = sat(in[2] + r);
	    beg[4] = sat(in[2] + g);
	    beg[5] = sat(in[2] + b);
	    beg += 6;
	    in += 4;
	}
    }
    return out;
}

static unsigned char *
ConvertToYUV(Tk_PhotoImageBlock *blk, int isvu, int *lenPtr)
{
    unsigned char *in, *out, *beg, *end;
    int r1, g1, b1, r2, g2, b2;

    if (blk->pitch != blk->width * blk->pixelSize) {
	return NULL;
    }
    lenPtr[0] = blk->width * blk->height * 2;
    out = attemptckalloc(lenPtr[0]);
    if (out == NULL) {
	return NULL;
    }
    beg = out;
    end = beg + lenPtr[0];
    in = blk->pixelPtr;
    while (beg < end) {
	r1 = in[blk->offset[0]];
	g1 = in[blk->offset[1]];
	b1 = in[blk->offset[2]];
	in += blk->pixelSize;
	beg[0] = sat(((4224 * r1 + 8256 * g1 + 1600 * b1) >> 14) + 16);
	r2 = in[blk->offset[0]];
	g2 = in[blk->offset[1]];
	b2 = in[blk->offset[2]];
	in += blk->pixelSize;
	beg[2] = sat(((4224 * r2 + 8256 * g2 + 1600 * b2) >> 14) + 16);
	r1 += r2;
	b1 += b2;
	g1 += g2;
	if (isvu) {
	    beg[3] = sat(((-2432 * r1 - 4736 * g1 + 7168 * b1) >> 15) + 128);
	    beg[1] = sat(((7168 * r1 - 6016 * g1 - 1152 * b1) >> 15) + 128);
	} else {
	    beg[1] = sat(((-2432 * r1 - 4736 * g1 + 7168 * b1) >> 15) + 128);
	    beg[3] = sat(((7168 * r1 - 6016 * g1 - 1152 * b1) >> 15) + 128);
	}
	beg += 4;
    }
    return out;
}

static unsigned char *
ConvertToGREY(Tk_PhotoImageBlock *blk, int *lenPtr)
{
    unsigned char *in, *out, *beg, *end;
    int r, g, b;

    if (blk->pitch != blk->width * blk->pixelSize) {
	return NULL;
    }
    lenPtr[0] = blk->width * blk->height;
    out = attemptckalloc(lenPtr[0]);
    if (out == NULL) {
	return NULL;
    }
    beg = out;
    end = beg + lenPtr[0];
    in = blk->pixelPtr;
    while (beg < end) {
	r = in[blk->offset[0]];
	g = in[blk->offset[1]];
	b = in[blk->offset[2]];
	in += blk->pixelSize;
	*beg++ = sat(((4224 * r + 8256 * g + 1600 * b) >> 14) + 16);
    }
    return out;
}

#ifdef USE_MJPEG
/*
 *-------------------------------------------------------------------------
 *
 * ConvertFromMJPEG --
 *
 *	Make RGB from a (M)JPEG frame. Logic is borrowed from libuvc.
 *
 *-------------------------------------------------------------------------
 */

struct error_mgr {
    struct jpeg_error_mgr super;
    jmp_buf jmp;
};

static void
j_out_msg(j_common_ptr info)
{
}

static void
j_fmt_msg(j_common_ptr info, char *buffer)
{
    buffer[0] = '\0';
}

static void
j_err_exit(j_common_ptr info)
{
    struct error_mgr *errmgr = (struct error_mgr *) info->err;

    (*info->err->output_message)(info);
    longjmp(errmgr->jmp, 1);
}

/*
 * ISO/IEC 10918-1:1993(E) K.3.3.
 * Default Huffman tables used by MJPEG UVC devices
 * which don't specify a Huffman table in the JPEG stream.
 */

static const unsigned char dc_lumi_len[] = {
    0, 0, 1, 5, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0
};

static const unsigned char dc_lumi_val[] = {
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11
};

static const unsigned char dc_chromi_len[] = {
    0, 0, 3, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0
};

static const unsigned char dc_chromi_val[] = {
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11
};

static const unsigned char ac_lumi_len[] = {
    0, 0, 2, 1, 3, 3, 2, 4, 3, 5, 5, 4, 4, 0, 0, 1, 0x7d
};

static const unsigned char ac_lumi_val[] = {
    0x01, 0x02, 0x03, 0x00, 0x04, 0x11, 0x05, 0x12, 0x21,
    0x31, 0x41, 0x06, 0x13, 0x51, 0x61, 0x07, 0x22, 0x71,
    0x14, 0x32, 0x81, 0x91, 0xa1, 0x08, 0x23, 0x42, 0xb1,
    0xc1, 0x15, 0x52, 0xd1, 0xf0, 0x24, 0x33, 0x62, 0x72,
    0x82, 0x09, 0x0a, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x25,
    0x26, 0x27, 0x28, 0x29, 0x2a, 0x34, 0x35, 0x36, 0x37,
    0x38, 0x39, 0x3a, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48,
    0x49, 0x4a, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59,
    0x5a, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6a,
    0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7a, 0x83,
    0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 0x8a, 0x92, 0x93,
    0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9a, 0xa2, 0xa3,
    0xa4, 0xa5, 0xa6, 0xa7, 0xa8, 0xa9, 0xaa, 0xb2, 0xb3,
    0xb4, 0xb5, 0xb6, 0xb7, 0xb8, 0xb9, 0xba, 0xc2, 0xc3,
    0xc4, 0xc5, 0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xd2, 0xd3,
    0xd4, 0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda, 0xe1, 0xe2,
    0xe3, 0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9, 0xea, 0xf1,
    0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8, 0xf9, 0xfa
};

static const unsigned char ac_chromi_len[] = {
    0, 0, 2, 1, 2, 4, 4, 3, 4, 7, 5, 4, 4, 0, 1, 2, 0x77
};

static const unsigned char ac_chromi_val[] = {
    0x00, 0x01, 0x02, 0x03, 0x11, 0x04, 0x05, 0x21, 0x31,
    0x06, 0x12, 0x41, 0x51, 0x07, 0x61, 0x71, 0x13, 0x22,
    0x32, 0x81, 0x08, 0x14, 0x42, 0x91, 0xa1, 0xb1, 0xc1,
    0x09, 0x23, 0x33, 0x52, 0xf0, 0x15, 0x62, 0x72, 0xd1,
    0x0a, 0x16, 0x24, 0x34, 0xe1, 0x25, 0xf1, 0x17, 0x18,
    0x19, 0x1a, 0x26, 0x27, 0x28, 0x29, 0x2a, 0x35, 0x36,
    0x37, 0x38, 0x39, 0x3a, 0x43, 0x44, 0x45, 0x46, 0x47,
    0x48, 0x49, 0x4a, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58,
    0x59, 0x5a, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69,
    0x6a, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7a,
    0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 0x8a,
    0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9a,
    0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7, 0xa8, 0xa9, 0xaa,
    0xb2, 0xb3, 0xb4, 0xb5, 0xb6, 0xb7, 0xb8, 0xb9, 0xba,
    0xc2, 0xc3, 0xc4, 0xc5, 0xc6, 0xc7, 0xc8, 0xc9, 0xca,
    0xd2, 0xd3, 0xd4, 0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda,
    0xe2, 0xe3, 0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9, 0xea,
    0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8, 0xf9, 0xfa
};

#define COPY_HUFF_TBL(dinfo, tbl, name)					\
do {									\
    if (dinfo->tbl == NULL) {						\
	dinfo->tbl = jpeg_alloc_huff_table((j_common_ptr) dinfo);	\
    }									\
    memcpy(dinfo->tbl->bits, name##_len, sizeof (name##_len));		\
    memset(dinfo->tbl->huffval, 0, sizeof (dinfo->tbl->huffval));	\
    memcpy(dinfo->tbl->huffval, name##_val, sizeof (name##_val));	\
} while (0)

static inline void
j_ins_huff_tbls(j_decompress_ptr dinfo)
{
    COPY_HUFF_TBL(dinfo, dc_huff_tbl_ptrs[0], dc_lumi);
    COPY_HUFF_TBL(dinfo, dc_huff_tbl_ptrs[1], dc_chromi);
    COPY_HUFF_TBL(dinfo, ac_huff_tbl_ptrs[0], ac_lumi);
    COPY_HUFF_TBL(dinfo, ac_huff_tbl_ptrs[1], ac_chromi);
}

#undef COPY_HUFF_TBL

static unsigned char *
ConvertFromMJPEG(unsigned char *in, int inlen, int width, int height)
{
    struct jpeg_decompress_struct dinfo;
    struct error_mgr jerr;
    unsigned char *out;
    size_t nlines;

    out = attemptckalloc(width * height * 3);
    if (out == NULL) {
	return NULL;
    }
    dinfo.err = jpeg_std_error(&jerr.super);
    jerr.super.output_message = j_out_msg;
    jerr.super.format_message = j_fmt_msg;
    jerr.super.error_exit = j_err_exit;
    if (setjmp(jerr.jmp)) {
	goto failed;
    }

    jpeg_create_decompress(&dinfo);
    jpeg_mem_src(&dinfo, in, inlen);
    jpeg_read_header(&dinfo, TRUE);
    if (dinfo.dc_huff_tbl_ptrs[0] == NULL) {
	j_ins_huff_tbls(&dinfo);
    }
    dinfo.out_color_space = JCS_RGB;
    dinfo.dct_method = JDCT_IFAST;
    jpeg_start_decompress(&dinfo);
    nlines = 0;
    while ((dinfo.output_scanline < dinfo.output_height) &&
	   (dinfo.output_scanline < height)) {
	unsigned char *buf[1];
	int n;

	buf[0] = out + nlines * width * 3;
	n = jpeg_read_scanlines(&dinfo, buf, 1);
	nlines += n;
    }
    jpeg_finish_decompress(&dinfo);
    jpeg_destroy_decompress(&dinfo);
    return out;

failed:
    jpeg_destroy_decompress(&dinfo);
    ckfree(out);
    return V4L2_MJPEG_FAILED;
}
#endif

/*
 *-------------------------------------------------------------------------
 *
 * GetImage --
 *
 *	Retrieve last captured buffer as photo image or byte array.
 *
 *-------------------------------------------------------------------------
 */

static int
GetImage(V4L2I *v4l2i, V4L2C *v4l2c, int flags, Tcl_Obj *arg)
{
    Tcl_Interp *interp = v4l2c->interp;
    Tk_PhotoHandle photo = NULL;
    int result = TCL_OK, done = 0;
    char *name;
    unsigned char *rgbToFree = NULL, *toFree = NULL;

    if (arg != NULL) {
	if (CheckForTk(v4l2i, interp) != TCL_OK) {
	    return TCL_ERROR;
	}
	if (Tk_MainWindow(interp) == NULL) {
	    Tcl_SetResult(interp, "application has been destroyed",
			  TCL_STATIC);
	    return TCL_ERROR;
	}
	name = Tcl_GetString(arg);
	photo = Tk_FindPhoto(interp, name);
	if (photo == NULL) {
	    Tcl_SetObjResult(interp,
		Tcl_ObjPrintf("can't use \"%s\": not a photo image", name));
	    return TCL_ERROR;
	}
    }
    if (v4l2c->bufrdy < 0) {
	/* no image available */
	if (photo != NULL) {
	    Tcl_SetObjResult(interp, Tcl_NewIntObj(0));
	} else {
	    Tcl_SetResult(interp, "no image available", TCL_STATIC);
	    result = TCL_ERROR;
	}
	goto done;
    }
    if (photo != NULL) {
	Tk_PhotoImageBlock block;
	int rot = v4l2c->rotate;
	int width = v4l2c->width;
	int height = v4l2c->height;

	switch (v4l2c->format) {
	case V4L2_PIX_FMT_YUYV:
	case V4L2_PIX_FMT_YVYU:
	    rgbToFree = ConvertFromYUV(v4l2c->vbufs[v4l2c->bufrdy].start,
				       width, height,
				       v4l2c->format == V4L2_PIX_FMT_YVYU);
	    if (rgbToFree == NULL) {
		Tcl_SetResult(interp, "out of memory", TCL_STATIC);
		result = TCL_ERROR;
		goto done;
	    }
	    block.pixelSize = 3;
	    block.offset[0] = 0;
	    block.offset[1] = 1;
	    block.offset[2] = 2;
	    block.offset[3] = 4;
	    block.pixelPtr = rgbToFree;
	    break;
	case V4L2_PIX_FMT_RGB32:
	    block.pixelSize = 4;
	    block.offset[0] = 0;
	    block.offset[1] = 1;
	    block.offset[2] = 2;
	    block.offset[3] = 3;
	    block.pixelPtr = v4l2c->vbufs[v4l2c->bufrdy].start;
	    break;
	case V4L2_PIX_FMT_BGR32:
	    block.pixelSize = 4;
	    block.offset[0] = 2;
	    block.offset[1] = 1;
	    block.offset[2] = 0;
	    block.offset[3] = 3;
	    block.pixelPtr = v4l2c->vbufs[v4l2c->bufrdy].start;
	    break;
	case V4L2_PIX_FMT_RGB24:
	default:
	    block.pixelSize = 3;
	    block.offset[0] = 0;
	    block.offset[1] = 1;
	    block.offset[2] = 2;
	    block.offset[3] = 4;
	    block.pixelPtr = v4l2c->vbufs[v4l2c->bufrdy].start;
	    break;
	case V4L2_PIX_FMT_BGR24:
	    block.pixelSize = 3;
	    block.offset[0] = 2;
	    block.offset[1] = 1;
	    block.offset[2] = 0;
	    block.offset[3] = 4;
	    block.pixelPtr = v4l2c->vbufs[v4l2c->bufrdy].start;
	    break;
#ifdef USE_MJPEG
	case V4L2_PIX_FMT_MJPEG:
	    rgbToFree = ConvertFromMJPEG(v4l2c->vbufs[v4l2c->bufrdy].start,
					 v4l2c->vbufs[v4l2c->bufrdy].length,
					 width, height);
	    if (rgbToFree == V4L2_MJPEG_FAILED) {
		rgbToFree = NULL;
		Tcl_SetResult(interp, "conversion from jpeg failed",
			      TCL_STATIC);
		result = TCL_ERROR;
		goto done;
	    }
	    if (rgbToFree == NULL) {
		Tcl_SetResult(interp, "out of memory", TCL_STATIC);
		result = TCL_ERROR;
		goto done;
	    }
	    block.pixelSize = 3;
	    block.offset[0] = 0;
	    block.offset[1] = 1;
	    block.offset[2] = 2;
	    block.offset[3] = 4;
	    block.pixelPtr = rgbToFree;
	    break;
#endif
	case V4L2_PIX_FMT_GREY:
	    block.pixelSize = 1;
	    block.offset[0] = 0;
	    block.offset[1] = 0;
	    block.offset[2] = 0;
	    block.offset[3] = 1;
	    block.pixelPtr = v4l2c->vbufs[v4l2c->bufrdy].start;
	    break;
#ifdef V4L2_PIX_FMT_Y10
	case V4L2_PIX_FMT_Y10:
#endif
#ifdef V4L2_PIX_FMT_Y16
	case V4L2_PIX_FMT_Y16:
#endif
#if defined(V4L2_PIX_FMT_Y16) || defined(V4L2_PIX_FMT_Y10)
	    block.pixelSize = 1;
	    block.offset[0] = 0;
	    block.offset[1] = 0;
	    block.offset[2] = 0;
	    block.offset[3] = 1;
	    block.pixelPtr = toFree = attemptckalloc(width * height);
	    if (block.pixelPtr == NULL) {
		Tcl_SetResult(interp, "out of memory", TCL_STATIC);
		result = TCL_ERROR;
		goto done;
	    } else {
		int n, v, shift;
		unsigned short *fromPtr;
		unsigned char *toPtr;

		fromPtr = (unsigned short *) v4l2c->vbufs[v4l2c->bufrdy].start;
		toPtr = block.pixelPtr;
		shift = v4l2c->greyshift;
		if (shift > 0) {
		    for (n = 0; n < width * height; n++) {
			v = *fromPtr++;
			v = v >> shift;
			*toPtr++ = v;
		    }
		} else {
		    shift = -shift;
		    for (n = 0; n < width * height; n++) {
			v = *fromPtr++;
			v = v << shift;
			*toPtr++ = v;
		    }
		}
	    }
	    break;
#endif
	}
	block.width = width;
	block.height = height;
	block.pitch = block.pixelSize * block.width;

	if ((flags & 0x07) && (block.pixelSize >= 3)) {
	    int x, y, w0, w1, w2;
	    unsigned char *src0, *src1, *src2, *dst;

	    toFree = attemptckalloc(width * height);
	    if (toFree == NULL) {
		Tcl_SetResult(interp, "out of memory", TCL_STATIC);
		result = TCL_ERROR;
		goto done;
	    }
	    dst = toFree;
	    switch (flags & 0x07) {
	    case 1:	/* blue */
		src0 = block.pixelPtr + block.offset[2];
		goto doOne;
	    case 2:	/* green */
		src0 = block.pixelPtr + block.offset[1];
		goto doOne;
	    case 4:	/* red */
		src0 = block.pixelPtr + block.offset[0];
	    doOne:
		for (y = 0; y < height; y++) {
		    unsigned char *src = src0 + block.pitch * y;

		    for (x = 0; x < width; x++) {
			*dst++ = *src;
			src += block.pixelSize;
		    }
		}
		break;
	    case 3:	/* blue + green */
		src0 = block.pixelPtr + block.offset[2];
		src1 = block.pixelPtr + block.offset[1];
		w0 = 162;
		w1 = 837;
		goto doTwo;
	    case 5:	/* blue + red */
		src0 = block.pixelPtr + block.offset[2];
		src1 = block.pixelPtr + block.offset[0];
		w0 = 276;
		w1 = 723;
		goto doTwo;
	    case 6:	/* green + red */
		src0 = block.pixelPtr + block.offset[1];
		src1 = block.pixelPtr + block.offset[0];
		w0 = 662;
		w1 = 337;
	    doTwo:
		for (y = 0; y < height; y++) {
		    unsigned char *srcA = src0 + block.pitch * y;
		    unsigned char *srcB = src1 + block.pitch * y;

		    for (x = 0; x < width; x++) {
			*dst++ = (w0 * srcA[0] + w1 * srcB[0]) / 1000;
			srcA += block.pixelSize;
			srcB += block.pixelSize;
		    }
		}
		break;
	    case 7:	/* all */
		src0 = block.pixelPtr + block.offset[2];
		src1 = block.pixelPtr + block.offset[1];
		src2 = block.pixelPtr + block.offset[0];
		w0 = 114;
		w1 = 587;
		w2 = 299;
		for (y = 0; y < height; y++) {
		    unsigned char *srcA = src0 + block.pitch * y;
		    unsigned char *srcB = src1 + block.pitch * y;
		    unsigned char *srcC = src2 + block.pitch * y;

		    for (x = 0; x < width; x++) {
			*dst++ =
			    (w0 * srcA[0] + w1 * srcB[0] + w2 * srcC[0]) / 1000;
			srcA += block.pixelSize;
			srcB += block.pixelSize;
			srcC += block.pixelSize;
		    }
		}
		break;
	    }
	    block.pitch = width;
	    block.pixelSize = 1;
	    block.offset[0] = 0;
	    block.offset[1] = 0;
	    block.offset[2] = 0;
	    block.offset[3] = 1;
	    block.pixelPtr = toFree;
	}

	if ((v4l2c->mirror & 3) == 3) {
	    rot = (rot + 180) % 360;
	}
	switch (rot) {
	case 270:	/* = 90 CW */
	    block.pitch = block.pixelSize;
	    block.pixelPtr += width * block.pixelSize * (height - 1);
	    block.pixelSize *= -width;
	    block.offset[3] = block.pixelSize + 1;	/* no alpha */
	    block.width = height;
	    block.height = width;
	    break;
	case 180:	/* = 180 CW */
	    block.pitch = -block.pitch;
	    block.pixelPtr += (width * height - 1) * block.pixelSize;
	    block.pixelSize = -block.pixelSize;
	    block.offset[3] = block.pixelSize + 1;	/* no alpha */
	    break;
	case 90:	/* = 270 CW */
	    block.pitch = -block.pixelSize;
	    block.pixelPtr += (width - 1) * block.pixelSize;
	    block.pixelSize *= width;
	    block.offset[3] = block.pixelSize + 1;	/* no alpha */
	    block.width = height;
	    block.height = width;
	    break;
	}
	if ((v4l2c->mirror & 3) == 2) {
	    /* mirror in X */
	    block.pixelPtr += (block.width - 1) * block.pixelSize;
	    block.pixelSize = -block.pixelSize;
	    block.offset[3] = block.pixelSize + 1;      /* no alpha */
	}
	if ((v4l2c->mirror & 3) == 1) {
	    /* mirror in Y */
	    block.pixelPtr += block.pitch * (block.height - 1);
	    block.pitch = -block.pitch;
	}

	if (Tk_PhotoExpand(interp, photo, block.width, block.height)
	    != TCL_OK) {
	    result = TCL_ERROR;
	    goto done;
	}
	if (Tk_PhotoPutBlock(interp, photo, &block, 0, 0,
			     block.width, block.height,
			     TK_PHOTO_COMPOSITE_SET) != TCL_OK) {
	    result = TCL_ERROR;
	} else {
	    Tcl_SetObjResult(interp, Tcl_NewIntObj(1));
	    done = 1;
	}
    } else {
	unsigned char *rawPtr = NULL;
	int rawSize, pixelSize;
	Tcl_Obj *list[4];

	switch (v4l2c->format) {
	case V4L2_PIX_FMT_YUYV:
	case V4L2_PIX_FMT_YVYU:
	    rgbToFree = ConvertFromYUV(v4l2c->vbufs[v4l2c->bufrdy].start,
				       v4l2c->width, v4l2c->height,
				       v4l2c->format == V4L2_PIX_FMT_YVYU);
	    if (rgbToFree == NULL) {
		Tcl_SetResult(interp, "out of memory", TCL_STATIC);
		result = TCL_ERROR;
		goto done;
	    }
	    rawSize = v4l2c->width * v4l2c->height * 3;
	    pixelSize = 3;
	    rawPtr = rgbToFree;
	    break;
	case V4L2_PIX_FMT_RGB32:
	case V4L2_PIX_FMT_BGR32:
	    rawSize = v4l2c->width * v4l2c->height * 4;
	    pixelSize = 4;
	    break;
	case V4L2_PIX_FMT_RGB24:
	case V4L2_PIX_FMT_BGR24:
	default:
	    rawSize = v4l2c->width * v4l2c->height * 3;
	    pixelSize = 3;
	    break;
#ifdef USE_MJPEG
	case V4L2_PIX_FMT_MJPEG:
	    rgbToFree = ConvertFromMJPEG(v4l2c->vbufs[v4l2c->bufrdy].start,
					 v4l2c->vbufs[v4l2c->bufrdy].length,
					 v4l2c->width, v4l2c->height);
	    if (rgbToFree == V4L2_MJPEG_FAILED) {
		rgbToFree = NULL;
		Tcl_SetResult(interp, "conversion from jpeg failed",
			      TCL_STATIC);
		result = TCL_ERROR;
		goto done;
	    }
	    if (rgbToFree == NULL) {
		Tcl_SetResult(interp, "out of memory", TCL_STATIC);
		result = TCL_ERROR;
		goto done;
	    }
	    rawSize = v4l2c->width * v4l2c->height * 3;
	    pixelSize = 3;
	    rawPtr = rgbToFree;
	    break;
#endif
	case V4L2_PIX_FMT_GREY:
	    rawSize = v4l2c->width * v4l2c->height;
	    pixelSize = 1;
	    break;
#ifdef V4L2_PIX_FMT_Y10
	case V4L2_PIX_FMT_Y10:
	    rawSize = v4l2c->width * v4l2c->height * 2;
	    pixelSize = 2;
	    break;
#endif
#ifdef V4L2_PIX_FMT_Y16
	case V4L2_PIX_FMT_Y16:
	    rawSize = v4l2c->width * v4l2c->height * 2;
	    pixelSize = 2;
	    break;
#endif
	}
	if (rawPtr == NULL) {
	    rawPtr = v4l2c->vbufs[v4l2c->bufrdy].start;
	}

	if ((flags & 0x07) && (pixelSize >= 3)) {
	    int x, y, w0, w1, w2;
	    unsigned char *src0, *src1, *src2, *dst;

	    toFree = attemptckalloc(v4l2c->width * v4l2c->height);
	    if (toFree == NULL) {
		Tcl_SetResult(interp, "out of memory", TCL_STATIC);
		result = TCL_ERROR;
		goto done;
	    }
	    dst = toFree;
	    switch (flags & 0x07) {
	    case 1:	/* blue */
		switch (v4l2c->format) {
		case V4L2_PIX_FMT_RGB32:
		case V4L2_PIX_FMT_RGB24:
		case V4L2_PIX_FMT_YUYV:
		case V4L2_PIX_FMT_YVYU:
		default:
		    src0 = rawPtr + 2;
		    break;
		case V4L2_PIX_FMT_BGR32:
		case V4L2_PIX_FMT_BGR24:
		    src0 = rawPtr + 0;
		    break;
		}
		goto rawOne;
	    case 2:	/* green */
		src0 = rawPtr + 1;
		goto rawOne;
	    case 4:	/* red */
		switch (v4l2c->format) {
		case V4L2_PIX_FMT_RGB32:
		case V4L2_PIX_FMT_RGB24:
		case V4L2_PIX_FMT_YUYV:
		case V4L2_PIX_FMT_YVYU:
		default:
		    src0 = rawPtr + 0;
		    break;
		case V4L2_PIX_FMT_BGR32:
		case V4L2_PIX_FMT_BGR24:
		    src0 = rawPtr + 2;
		    break;
		}
	    rawOne:
		for (y = 0; y < v4l2c->height; y++) {
		    unsigned char *src = src0 + v4l2c->width * y;

		    for (x = 0; x < v4l2c->width; x++) {
			*dst++ = *src;
			src += pixelSize;
		    }
		}
		break;
	    case 3:	/* blue + green */
		switch (v4l2c->format) {
		case V4L2_PIX_FMT_RGB32:
		case V4L2_PIX_FMT_RGB24:
		case V4L2_PIX_FMT_YUYV:
		case V4L2_PIX_FMT_YVYU:
		default:
		    src0 = rawPtr + 2;
		    src1 = rawPtr + 1;
		    break;
		case V4L2_PIX_FMT_BGR32:
		case V4L2_PIX_FMT_BGR24:
		    src0 = rawPtr + 0;
		    src1 = rawPtr + 1;
		    break;
		}
		w0 = 162;
		w1 = 837;
		goto rawTwo;
	    case 5:	/* blue + red */
		switch (v4l2c->format) {
		case V4L2_PIX_FMT_RGB32:
		case V4L2_PIX_FMT_RGB24:
		case V4L2_PIX_FMT_YUYV:
		case V4L2_PIX_FMT_YVYU:
		default:
		    src0 = rawPtr + 2;
		    src1 = rawPtr + 0;
		    break;
		case V4L2_PIX_FMT_BGR32:
		case V4L2_PIX_FMT_BGR24:
		    src0 = rawPtr + 0;
		    src1 = rawPtr + 2;
		    break;
		}
		w0 = 276;
		w1 = 723;
		goto rawTwo;
	    case 6:	/* green + red */
		switch (v4l2c->format) {
		case V4L2_PIX_FMT_RGB32:
		case V4L2_PIX_FMT_RGB24:
		case V4L2_PIX_FMT_YUYV:
		case V4L2_PIX_FMT_YVYU:
		default:
		    src0 = rawPtr + 1;
		    src1 = rawPtr + 0;
		    break;
		case V4L2_PIX_FMT_BGR32:
		case V4L2_PIX_FMT_BGR24:
		    src0 = rawPtr + 1;
		    src1 = rawPtr + 2;
		    break;
		}
		w0 = 662;
		w1 = 337;
	    rawTwo:
		for (y = 0; y < v4l2c->height; y++) {
		    unsigned char *srcA = src0 + v4l2c->width * y;
		    unsigned char *srcB = src1 + v4l2c->width * y;

		    for (x = 0; x < v4l2c->width; x++) {
			*dst++ = (w0 * srcA[0] + w1 * srcB[0]) / 1000;
			srcA += pixelSize;
			srcB += pixelSize;
		    }
		}
		break;
	    case 7:	/* all */
		switch (v4l2c->format) {
		case V4L2_PIX_FMT_RGB32:
		case V4L2_PIX_FMT_RGB24:
		case V4L2_PIX_FMT_YUYV:
		case V4L2_PIX_FMT_YVYU:
		default:
		    src0 = rawPtr + 2;
		    src1 = rawPtr + 1;
		    src2 = rawPtr + 0;
		    break;
		case V4L2_PIX_FMT_BGR32:
		case V4L2_PIX_FMT_BGR24:
		    src0 = rawPtr + 0;
		    src1 = rawPtr + 1;
		    src2 = rawPtr + 2;
		    break;
		}
		w0 = 114;
		w1 = 587;
		w2 = 299;
		for (y = 0; y < v4l2c->height; y++) {
		    unsigned char *srcA = src0 + v4l2c->width * y;
		    unsigned char *srcB = src1 + v4l2c->width * y;
		    unsigned char *srcC = src2 + v4l2c->width * y;

		    for (x = 0; x < v4l2c->width; x++) {
			*dst++ =
			    (w0 * srcA[0] + w1 * srcB[0] + w2 * srcC[0]) / 1000;
			srcA += pixelSize;
			srcB += pixelSize;
			srcC += pixelSize;
		    }
		}
		break;
	    }
	    rawPtr = toFree;
	    rawSize = v4l2c->width * v4l2c->height;
	    pixelSize = 1;
	}

	list[0] = Tcl_NewIntObj(v4l2c->width);
	list[1] = Tcl_NewIntObj(v4l2c->height);
	list[2] = Tcl_NewIntObj(pixelSize);
	list[3] = Tcl_NewByteArrayObj(rawPtr, rawSize);
	Tcl_SetObjResult(interp, Tcl_NewListObj(4, list));
	done = 1;
    }
done:
    if (toFree != NULL) {
	ckfree(toFree);
    }
    if (rgbToFree != NULL) {
	ckfree(rgbToFree);
    }
    if (done && !v4l2c->bufdone) {
	v4l2c->bufdone = 1;
	v4l2c->counters[1] += 1;
    }
    return result;
}

/*
 *-------------------------------------------------------------------------
 *
 * DataToPhoto --
 *
 *	Put byte array data to a Tk photo image.
 *
 *-------------------------------------------------------------------------
 */

static int
DataToPhoto(V4L2I *v4l2i, Tcl_Interp *interp,
	    int objc, Tcl_Obj * const objv[])
{
    int width, height, bpp, length;
    int rot = 0, mirx = 0, miry = 0, mirror;
    unsigned char *data;
    Tk_PhotoHandle photo;
    char *name;
    Tk_PhotoImageBlock block;

    if (CheckForTk(v4l2i, interp) != TCL_OK) {
	return TCL_ERROR;
    }
    if ((objc < 7) || (objc > 10)) {
	Tcl_WrongNumArgs(interp, 2, objv,
			 "photo width height bpp bytearray "
			 "?rotation mirrorx mirrory?");
	return TCL_ERROR;
    }
    if (Tk_MainWindow(interp) == NULL) {
	Tcl_SetResult(interp, "application has been destroyed",
		      TCL_STATIC);
	return TCL_ERROR;
    }
    name = Tcl_GetString(objv[2]);
    photo = Tk_FindPhoto(interp, name);
    if (photo == NULL) {
	Tcl_SetObjResult(interp,
	    Tcl_ObjPrintf("can't use \"%s\": not a photo image", name));
	return TCL_ERROR;
    }
    if (Tcl_GetIntFromObj(interp, objv[3], &width) != TCL_OK) {
	return TCL_ERROR;
    }
    if (Tcl_GetIntFromObj(interp, objv[4], &height) != TCL_OK) {
	return TCL_ERROR;
    }
    if (Tcl_GetIntFromObj(interp, objv[5], &bpp) != TCL_OK) {
	return TCL_ERROR;
    }
    if ((objc > 7) && (Tcl_GetIntFromObj(interp, objv[7], &rot) != TCL_OK)) {
	return TCL_ERROR;
    }
    if ((objc > 8) &&
	(Tcl_GetBooleanFromObj(interp, objv[8], &mirx) != TCL_OK)) {
	return TCL_ERROR;
    }
    if ((objc > 9) &&
	(Tcl_GetBooleanFromObj(interp, objv[9], &miry) != TCL_OK)) {
	return TCL_ERROR;
    }
    data = Tcl_GetByteArrayFromObj(objv[6], &length);
    if ((length < width * height * bpp) ||
	((bpp != 1) && (bpp != 3))) {
	Tcl_SetResult(interp, "unsupported data format", TCL_STATIC);
	return TCL_ERROR;
    }
    if (bpp == 1) {
	block.pixelSize = 1;
	block.offset[0] = 0;
	block.offset[1] = 0;
	block.offset[2] = 0;
	block.offset[3] = 1;
    } else {
	block.pixelSize = 3;
	block.offset[0] = 0;
	block.offset[1] = 1;
	block.offset[2] = 2;
	block.offset[3] = 4;
    }
    block.width = width;
    block.height = height;
    block.pitch = width * bpp;
    block.pixelPtr = data;
    mirror = (mirx ? 1 : 0) | (miry ? 2 : 0);
    rot = rot % 360;
    if (rot < 45) {
	rot = 0;
    } else if (rot < 135) {
	rot = 90;
    } else if (rot < 225) {
	rot = 180;
    } else if (rot < 315) {
	rot = 270;
    } else {
	rot = 0;
    }
    if ((mirror & 3) == 3) {
	rot = (rot + 180) % 360;
    }
    switch (rot) {
    case 270:	/* = 90 CW */
	block.pitch = block.pixelSize;
	block.pixelPtr += width * block.pixelSize * (height - 1);
	block.pixelSize *= -width;
	block.offset[3] = block.pixelSize + 1;	/* no alpha */
	block.width = height;
	block.height = width;
	break;
    case 180:	/* = 180 CW */
	block.pitch = -block.pitch;
	block.pixelPtr += (width * height - 1) * block.pixelSize;
	block.pixelSize = -block.pixelSize;
	block.offset[3] = block.pixelSize + 1;	/* no alpha */
	break;
    case 90:	/* = 270 CW */
	block.pitch = -block.pixelSize;
	block.pixelPtr += (width - 1) * block.pixelSize;
	block.pixelSize *= width;
	block.offset[3] = block.pixelSize + 1;	/* no alpha */
	block.width = height;
	block.height = width;
	break;
    }
    if ((mirror & 3) == 2) {
	/* mirror in X */
	block.pixelPtr += (block.width - 1) * block.pixelSize;
	block.pixelSize = -block.pixelSize;
	block.offset[3] = block.pixelSize + 1;      /* no alpha */
    }
    if ((mirror & 3) == 1) {
	/* mirror in Y */
	block.pixelPtr += block.pitch * (block.height - 1);
	block.pitch = -block.pitch;
    }
    if (Tk_PhotoExpand(interp, photo, block.width, block.height) != TCL_OK) {
	return TCL_ERROR;
    }
    if (Tk_PhotoPutBlock(interp, photo, &block, 0, 0, block.width,
			 block.height, TK_PHOTO_COMPOSITE_SET) != TCL_OK) {
	return TCL_ERROR;
    }
    return TCL_OK;
}

/*
 *-------------------------------------------------------------------------
 *
 * IsLoopDevice --
 *
 *	Determine if device is a loopback device.
 *
 *-------------------------------------------------------------------------
 */

#ifdef linux
static int
IsLoopDevice(const char *devName)
{
    const char *p;
    int rc;
    Tcl_DString ds;

    p = strrchr(devName, '/');
    if ((p != NULL) && p[1]) {
	devName = p + 1;
    }
    Tcl_DStringInit(&ds);
    Tcl_DStringAppend(&ds, "/sys/devices/virtual/video4linux/", -1);
    Tcl_DStringAppend(&ds, devName, -1);
    Tcl_DStringAppend(&ds, "/format", -1);
    rc = access(Tcl_DStringValue(&ds), R_OK);
    Tcl_DStringFree(&ds);
    return (rc == 0);
}
#endif

/*
 *-------------------------------------------------------------------------
 *
 * V4l2ObjCmdDeleted --
 *
 *	Destructor of "v4l2" Tcl command. Closes all device and
 *	releases all resources.
 *
 *-------------------------------------------------------------------------
 */

static void
V4l2ObjCmdDeleted(ClientData clientData)
{
    V4L2I *v4l2i = (V4L2I *) clientData;
    Tcl_HashEntry *hPtr;
    Tcl_HashSearch search;
    V4L2C *v4l2c;

    hPtr = Tcl_FirstHashEntry(&v4l2i->v4l2c, &search);
    while (hPtr != NULL) {
	v4l2c = (V4L2C *) Tcl_GetHashValue(hPtr);
	StopCapture(v4l2c);
	v4l2_close(v4l2c->fd);
	v4l2c->fd = -1;
	InitControls(v4l2c);	/* release */
	Tcl_DeleteHashTable(&v4l2c->ctrl);
	Tcl_DeleteHashTable(&v4l2c->nctrl);
	Tcl_DStringFree(&v4l2c->devName);
	Tcl_DStringFree(&v4l2c->cbCmd);
	ckfree((char *) v4l2c);
	hPtr = Tcl_NextHashEntry(&search);
    }
    Tcl_DeleteHashTable(&v4l2i->v4l2c);
#ifdef HAVE_LIBUDEV
    v4l2i->interp = NULL;
    Tcl_DStringFree(&v4l2i->cbCmd);
    Tcl_DeleteHashTable(&v4l2i->vdevs);
    if (v4l2i->udevMon != NULL) {
	Tcl_DeleteFileHandler(udev_monitor_get_fd(v4l2i->udevMon));
	udev_monitor_unref(v4l2i->udevMon);
	v4l2i->udevMon = NULL;
    }
    if (v4l2i->udev != NULL) {
	udev_unref(v4l2i->udev);
	v4l2i->udev = NULL;
    }
#endif
    ckfree((char *) v4l2i);
}

/*
 *-------------------------------------------------------------------------
 *
 * V4l2ObjCmd --
 *
 *	"v4l2" Tcl command dealing with Video For Linux Two.
 *
 * Results:
 *	A standard Tcl result.
 *
 * Side effects:
 *	See the user documentation.
 *
 *-------------------------------------------------------------------------
 */

static int
V4l2ObjCmd(ClientData clientData, Tcl_Interp *interp,
	   int objc, Tcl_Obj * const objv[])
{
    V4L2I *v4l2i = (V4L2I *) clientData;
    V4L2C *v4l2c;
    Tcl_HashEntry *hPtr;
    int ret = TCL_OK, command;

    static const char *cmdNames[] = {
	"close", "counters", "devices", "greyimage", "greyshift",
	"image", "info", "isloopback", "listen", "loopback",
	"mbcopy", "mcopy", "mirror", "open", "orientation",
	"parameters", "start", "state", "stop", "tophoto", "write",
	"writephoto", NULL
    };
    enum cmdCode {
	CMD_close, CMD_counters, CMD_devices, CMD_greyimage, CMD_greyshift,
	CMD_image, CMD_info, CMD_isloopback, CMD_listen, CMD_loopback,
	CMD_mbcopy, CMD_mcopy, CMD_mirror, CMD_open, CMD_orientation,
	CMD_parameters,	CMD_start, CMD_state, CMD_stop, CMD_tophoto, CMD_write,
	CMD_writephoto
    };

    if (objc < 2) {
	Tcl_WrongNumArgs(interp, 1, objv, "option ...");
	return TCL_ERROR;
    }
    if (Tcl_GetIndexFromObj(interp, objv[1], cmdNames, "option", 0,
			    &command) != TCL_OK) {
	return TCL_ERROR;
    }

    switch ((enum cmdCode) command) {

    case CMD_close:
	if (objc != 3) {
	    Tcl_WrongNumArgs(interp, 2, objv, "devid");
	    return TCL_ERROR;
	}
	hPtr = Tcl_FindHashEntry(&v4l2i->v4l2c, Tcl_GetString(objv[2]));
	if (hPtr != NULL) {
	    v4l2c = (V4L2C *) Tcl_GetHashValue(hPtr);
	    Tcl_DeleteHashEntry(hPtr);
	    StopCapture(v4l2c);
	    v4l2_close(v4l2c->fd);
	    v4l2c->fd = -1;
	    InitControls(v4l2c);	/* release */
	    Tcl_DeleteHashTable(&v4l2c->ctrl);
	    Tcl_DeleteHashTable(&v4l2c->nctrl);
	    Tcl_DStringFree(&v4l2c->devName);
	    Tcl_DStringFree(&v4l2c->cbCmd);
	    ckfree((char *) v4l2c);
	} else {
devNotFound:
	    Tcl_SetObjResult(interp,
		Tcl_ObjPrintf("device \"%s\" not found",
			      Tcl_GetString(objv[2])));
	    ret = TCL_ERROR;
	}
	break;

    case CMD_counters:
	if (objc != 3) {
	    Tcl_WrongNumArgs(interp, 2, objv, "devid");
	    return TCL_ERROR;
	}
	hPtr = Tcl_FindHashEntry(&v4l2i->v4l2c, Tcl_GetString(objv[2]));
	if (hPtr != NULL) {
	    Tcl_Obj *r[2];

	    v4l2c = (V4L2C *) Tcl_GetHashValue(hPtr);
	    r[0] = Tcl_NewWideIntObj(v4l2c->counters[0]);
	    r[1] = Tcl_NewWideIntObj(v4l2c->counters[1]);
	    Tcl_SetObjResult(interp, Tcl_NewListObj(2, r));
	} else {
	    goto devNotFound;
	}
	break;

    case CMD_devices:
	if (objc != 2) {
	    Tcl_WrongNumArgs(interp, 2, objv, NULL);
	    return TCL_ERROR;
	}
#ifdef HAVE_LIBUDEV
	if (v4l2i->udevMon != NULL) {
	    Tcl_Obj *list = Tcl_NewListObj(0, NULL);
	    Tcl_HashSearch search;

	    hPtr = Tcl_FirstHashEntry(&v4l2i->vdevs, &search);
	    while (hPtr != NULL) {
		Tcl_ListObjAppendElement(NULL, list,
			Tcl_NewStringObj(Tcl_GetHashValue(hPtr), -1));
		hPtr = Tcl_NextHashEntry(&search);
	    }
	    Tcl_SetObjResult(interp, list);
	} else
#endif
	{
	    ret = Tcl_EvalEx(interp,
			     "glob -nocomplain -types {c l s} /dev/video*",
			     -1, TCL_EVAL_GLOBAL);
	}
	break;

    case CMD_greyimage:
	if ((objc < 4) || (objc > 5)) {
	    Tcl_WrongNumArgs(interp, 2, objv, "devid mask ?photoImage?");
	    return TCL_ERROR;
	}
	hPtr = Tcl_FindHashEntry(&v4l2i->v4l2c, Tcl_GetString(objv[2]));
	if (hPtr != NULL) {
	    char *maskStr = Tcl_GetString(objv[3]);
	    int mask = 0;

	    v4l2c = (V4L2C *) Tcl_GetHashValue(hPtr);
	    if (strchr(maskStr, 'b') || strchr(maskStr, 'B')) {
		mask |= 0x01;
	    }
	    if (strchr(maskStr, 'g') || strchr(maskStr, 'G')) {
		mask |= 0x02;
	    }
	    if (strchr(maskStr, 'r') || strchr(maskStr, 'R')) {
		mask |= 0x04;
	    }
	    if (mask == 0) {
		mask = 0x07;
	    }
	    ret = GetImage(v4l2i, v4l2c, mask, (objc > 4) ? objv[4] : NULL);
	} else {
	    goto devNotFound;
	}
	break;

    case CMD_greyshift:
	if (objc != 3 && objc != 4) {
	    Tcl_WrongNumArgs(interp, 2, objv, "devid ?shift?");
	    return TCL_ERROR;
	}
	hPtr = Tcl_FindHashEntry(&v4l2i->v4l2c, Tcl_GetString(objv[2]));
	if (hPtr != NULL) {
	    v4l2c = (V4L2C *) Tcl_GetHashValue(hPtr);
	    if (objc > 3) {
		int shift;

		if (Tcl_GetIntFromObj(interp, objv[3], &shift) != TCL_OK) {
		    return TCL_ERROR;
		}
		v4l2c->greyshift = shift;
	    } else {
		Tcl_SetIntObj(Tcl_GetObjResult(interp), v4l2c->greyshift);
	    }
	} else {
	    goto devNotFound;
	}
	break;

    case CMD_image:
	if ((objc < 3) || (objc > 4)) {
	    Tcl_WrongNumArgs(interp, 2, objv, "devid ?photoImage?");
	    return TCL_ERROR;
	}
	hPtr = Tcl_FindHashEntry(&v4l2i->v4l2c, Tcl_GetString(objv[2]));
	if (hPtr != NULL) {
	    v4l2c = (V4L2C *) Tcl_GetHashValue(hPtr);
	    ret = GetImage(v4l2i, v4l2c, 0, (objc > 3) ? objv[3] : NULL);
	} else {
	    goto devNotFound;
	}
	break;

    case CMD_info:
	if (objc > 3) {
	    Tcl_WrongNumArgs(interp, 2, objv, "?devid?");
	    return TCL_ERROR;
	}
	if (objc == 2) {
	    Tcl_HashSearch search;
	    Tcl_Obj *list = Tcl_NewListObj(0, NULL);

	    hPtr = Tcl_FirstHashEntry(&v4l2i->v4l2c, &search);
	    while (hPtr != NULL) {
		v4l2c = (V4L2C *) Tcl_GetHashValue(hPtr);
		Tcl_ListObjAppendElement(NULL, list,
			Tcl_NewStringObj(v4l2c->devId, -1));
		hPtr = Tcl_NextHashEntry(&search);
	    }
	    Tcl_SetObjResult(interp, list);
	} else {
	    hPtr = Tcl_FindHashEntry(&v4l2i->v4l2c, Tcl_GetString(objv[2]));
	    if (hPtr != NULL) {
		Tcl_Obj *r[2];

		v4l2c = (V4L2C *) Tcl_GetHashValue(hPtr);
		r[0] = Tcl_NewStringObj(Tcl_DStringValue(&v4l2c->devName), -1);
		Tcl_DStringSetLength(&v4l2c->cbCmd, v4l2c->cbCmdLen);
		r[1] = Tcl_NewStringObj(Tcl_DStringValue(&v4l2c->cbCmd), -1);
		Tcl_SetObjResult(interp, Tcl_NewListObj(2, r));
	    } else {
		goto devNotFound;
	    }
	}
	break;

    case CMD_isloopback:
	if (objc > 3) {
	    Tcl_WrongNumArgs(interp, 2, objv, "device");
	    return TCL_ERROR;
	}
#ifdef linux
	Tcl_SetObjResult(interp, Tcl_NewBooleanObj(
		IsLoopDevice(Tcl_GetString(objv[2]))));
#else
	Tcl_SetObjResult(interp, Tcl_NewBooleanObj(0));
#endif
	break;

    case CMD_listen:
	if (objc > 3) {
	    Tcl_WrongNumArgs(interp, 2, objv, "?cmd?");
	    return TCL_ERROR;
	}
#ifdef HAVE_LIBUDEV
	if (v4l2i->udevMon != NULL) {
	    if (objc == 2) {
		Tcl_DStringSetLength(&v4l2i->cbCmd, v4l2i->cbCmdLen);
		Tcl_SetObjResult(interp,
			Tcl_NewStringObj(Tcl_DStringValue(&v4l2i->cbCmd),
					 Tcl_DStringLength(&v4l2i->cbCmd)));
	    } else {
		Tcl_DStringSetLength(&v4l2i->cbCmd, 0);
		Tcl_DStringAppend(&v4l2i->cbCmd, Tcl_GetString(objv[2]), -1);
		v4l2i->cbCmdLen = Tcl_DStringLength(&v4l2i->cbCmd);
	    }
	}
#endif
	break;

    case CMD_loopback: {
#ifdef linux
	struct v4l2_format fmt;
	struct v4l2_streamparm stp;
	int i, fd;
	char *devName, fcbuf[8];
#endif

	if ((objc != 3) && (objc != 7)) {
	    Tcl_WrongNumArgs(interp, 2, objv,
			     "device ?fourcc width height fps?");
	    return TCL_ERROR;
	}
#ifdef linux
	devName = Tcl_GetString(objv[2]);
	if (!IsLoopDevice(devName)) {
	    Tcl_SetObjResult(interp,
		    Tcl_ObjPrintf("\"%s\" is not a loop device", devName));
	    return TCL_ERROR;
	}
	if (objc == 3) {
	    char buf[64];
	    Tcl_Obj *list[4];

	    fd = open(devName, O_RDWR | O_NONBLOCK, 0);
	    if (fd < 0) {
		Tcl_SetObjResult(interp,
			Tcl_ObjPrintf("error while opening \"%s\": %s",
				      devName, Tcl_PosixError(interp)));
		return TCL_ERROR;
	    }
	    fcntl(fd, F_SETFD, FD_CLOEXEC);
	    memset(&fmt, 0, sizeof (fmt));
	    fmt.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
	    DoIoctl(fd, VIDIOC_G_FMT, &fmt);
	    memset(&stp, 0, sizeof (stp));
	    stp.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
	    DoIoctl(fd, VIDIOC_G_PARM, &stp);
	    i = 0;
	    fcbuf[i++] = fmt.fmt.pix.pixelformat;
	    fcbuf[i++] = fmt.fmt.pix.pixelformat >> 8;
	    fcbuf[i++] = fmt.fmt.pix.pixelformat >> 16;
	    fcbuf[i++] = (fmt.fmt.pix.pixelformat >> 24) & 0x7f;
	    while (i && (fcbuf[i - 1] == ' ')) {
		--i;
	    }
	    fcbuf[i] = '\0';
	    list[0] = Tcl_NewStringObj(fcbuf, -1);
	    list[1] = Tcl_NewIntObj(fmt.fmt.pix.width);
	    list[2] = Tcl_NewIntObj(fmt.fmt.pix.height);
	    if (stp.parm.capture.timeperframe.denominator == 1) {
		sprintf(buf, "%d", stp.parm.capture.timeperframe.numerator);
	    } else {
		sprintf(buf, "%d/%d",
			stp.parm.capture.timeperframe.numerator,
			stp.parm.capture.timeperframe.denominator);
	    }
	    list[3] = Tcl_NewStringObj(buf, -1);
	    Tcl_SetObjResult(interp, Tcl_NewListObj(4, list));
	} else {
	    int width, height, fps[2], v[3];
	    char *p;
	    struct v4l2_fract tpf;
	    struct v4l2_control xd;
	    struct utsname uts;

	    i = 0;
	    p = Tcl_GetString(objv[3]);
	    memset(fcbuf, ' ', 4);
	    while (i < 4) {
		if (p[i] == '\0') {
		    break;
		}
		fcbuf[i] = p[i];
		++i;
	    }
	    if ((Tcl_GetIntFromObj(interp, objv[4], &width) != TCL_OK) ||
		(Tcl_GetIntFromObj(interp, objv[5], &height) != TCL_OK)) {
		return TCL_ERROR;
	    }
	    i = sizeof (FormatsLoop) / sizeof (FormatsLoop[0]);
	    while (i > 0) {
		--i;
		if (v4l2_fourcc(fcbuf[0], fcbuf[1], fcbuf[2], fcbuf[3]) ==
		    FormatsLoop[i]) {
		    break;
		}
	    }
	    if (i < 0) {
		Tcl_SetResult(interp, "unsupported fourcc", TCL_STATIC);
		return TCL_ERROR;
	    }
	    fps[0] = 0;
	    fps[1] = 1;
	    if ((sscanf(Tcl_GetString(objv[6]), "%d/%d",
			fps + 0, fps + 1) < 1) ||
		(fps[1] <= 0) || (fps[0] <= 0)) {
		Tcl_SetResult(interp, "invalid frame rate parameter",
			      TCL_STATIC);
		return TCL_ERROR;
	    }
	    tpf.numerator = fps[0];
	    tpf.denominator = fps[1];
	    devName = Tcl_GetString(objv[2]);
	    fd = open(devName, O_RDWR | O_NONBLOCK, 0);
	    if (fd < 0) {
		Tcl_SetObjResult(interp,
			Tcl_ObjPrintf("error while opening \"%s\": %s",
				      devName, Tcl_PosixError(interp)));
		return TCL_ERROR;
	    }
	    fcntl(fd, F_SETFD, FD_CLOEXEC);
	    memset(&fmt, 0, sizeof (fmt));
	    fmt.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
	    if (DoIoctl(fd, VIDIOC_G_FMT, &fmt) < 0) {
		Tcl_SetObjResult(interp,
				 Tcl_ObjPrintf("error querying format: %s",
					       Tcl_PosixError(interp)));
		close(fd);
		return TCL_ERROR;
	    }
	    /* Private loopback control CID_KEEP_FORMAT depends on version. */
	    memset(&v, 0, sizeof (v));
	    memset(&uts, 0, sizeof (uts));
	    uname(&uts);
	    if (sscanf(uts.release, "%d.%d.%d", v + 0, v + 1, v + 2) > 0) {
		v[0] = v[0] * 100000 + v[1] * 1000 + v[2];
	    }
	    if (v[0] < 206036) {
		/* Linux version < 2.6.36 */
		xd.id = V4L2_CID_PRIVATE_BASE + 0;
		v[1] = v[2] = 0;
	    } else {
		xd.id = (V4L2_CID_USER_BASE | 0xf000) + 0;
		v[1] = V4L2_CID_PRIVATE_BASE + 0;
		v[2] = xd.id;
	    }
	    xd.value = 0;
	    if ((DoIoctl(fd, VIDIOC_S_CTRL, &xd) < 0) && (errno == EINVAL)) {
		/* Retry with alternative. */
		if (v[1]) {
		    xd.id = v[1];
		    if (DoIoctl(fd, VIDIOC_S_CTRL, &xd) < 0) {
			/* Give up but restore. */
			xd.id = v[2];
		    }
		}
	    }
	    /* Must close and reopen in order to clear CID_KEEP_FORMAT. */
	    close(fd);
	    fd = open(devName, O_RDWR | O_NONBLOCK, 0);
	    if (fd < 0) {
		Tcl_SetObjResult(interp,
			Tcl_ObjPrintf("error while opening \"%s\": %s",
				      devName, Tcl_PosixError(interp)));
		return TCL_ERROR;
	    }
	    fcntl(fd, F_SETFD, FD_CLOEXEC);
	    fmt.fmt.pix.width = width;
	    fmt.fmt.pix.height = height;
	    fmt.fmt.pix.bytesperline = 0;
	    fmt.fmt.pix.sizeimage = 0;
	    fmt.fmt.pix.pixelformat =
		v4l2_fourcc(fcbuf[0], fcbuf[1], fcbuf[2], fcbuf[3]);
	    if (DoIoctl(fd, VIDIOC_S_FMT, &fmt) < 0) {
		Tcl_SetObjResult(interp,
				 Tcl_ObjPrintf("error setting format: %s",
					       Tcl_PosixError(interp)));
		close(fd);
		return TCL_ERROR;
	    }
	    memset(&stp, 0, sizeof (stp));
	    stp.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
	    if (DoIoctl(fd, VIDIOC_G_PARM, &stp) >= 0) {
		stp.parm.capture.timeperframe = tpf;
		DoIoctl(fd, VIDIOC_S_PARM, &stp);
	    }
	    /* Turn on CID_KEEP_FORMAT again. */
	    xd.value = 1;
	    DoIoctl(fd, VIDIOC_S_CTRL, &xd);
	}
	close(fd);
#else
	Tcl_SetResult(interp, "unsupported on this platform", TCL_STATIC);
	ret = TCL_ERROR;
#endif
	break;
    }
    case CMD_mbcopy: {
	int mask0, mask, i, srcLen, dstLen;
	unsigned char *src, *dst;

	if (objc != 5) {
	    Tcl_WrongNumArgs(interp, 2, objv, "bytearray1 bytearray2 mask");
	    return TCL_ERROR;
	}
	if (Tcl_GetIntFromObj(interp, objv[4], &mask0) != TCL_OK) {
	    return TCL_ERROR;
	}
	dst = Tcl_GetByteArrayFromObj(objv[2], &dstLen);
	src = Tcl_GetByteArrayFromObj(objv[3], &srcLen);
	if ((srcLen != dstLen) || (srcLen % 3)) {
	    Tcl_SetResult(interp, "incompatible bytearrays", TCL_STATIC);
	    return TCL_ERROR;
	}
	mask = (mask0 >> 16) & 0xff;	/* red */
	if (mask != 0) {
	    for (i = 0; i < srcLen; i += 3) {
		dst[i] = (dst[i] & (~mask)) | (src[i] & mask);
	    }
	}
	mask = (mask0 >> 8) & 0xff;	/* green */
	if (mask != 0) {
	    for (i = 1; i < srcLen; i += 3) {
		dst[i] = (dst[i] & (~mask)) | (src[i] & mask);
	    }
	}
	mask = mask0 & 0xff;		/* blue */
	if (mask != 0) {
	    for (i = 2; i < srcLen; i += 3) {
		dst[i] = (dst[i] & (~mask)) | (src[i] & mask);
	    }
	}
	break;
    }

    case CMD_mcopy: {
	char *name;
	Tk_PhotoHandle ph1, ph2;
	int mask0, mask, nops = 0, x, y;
	Tk_PhotoImageBlock block1, block2;
	unsigned char *src, *dst;

	if (objc != 5) {
	    Tcl_WrongNumArgs(interp, 2, objv, "photo1 photo2 mask");
	    return TCL_ERROR;
	}
	if (CheckForTk(v4l2i, interp) != TCL_OK) {
	    return TCL_ERROR;
	}
	name = Tcl_GetString(objv[2]);
	ph1 = Tk_FindPhoto(interp, name);
	if (ph1 == NULL) {
	    Tcl_SetObjResult(interp,
		Tcl_ObjPrintf("can't use \"%s\": not a photo image", name));
	    return TCL_ERROR;
	}
	name = Tcl_GetString(objv[3]);
	ph2 = Tk_FindPhoto(interp, name);
	if (ph2 == NULL) {
	    Tcl_SetObjResult(interp,
		Tcl_ObjPrintf("can't use \"%s\": not a photo image", name));
	    return TCL_ERROR;
	}
	if (Tcl_GetIntFromObj(interp, objv[4], &mask0) != TCL_OK) {
	    return TCL_ERROR;
	}
	Tk_PhotoGetImage(ph1, &block1);
	Tk_PhotoGetImage(ph2, &block2);
	if ((block1.width != block2.width) ||
	    (block1.height != block2.height) ||
	    (block1.pixelSize != block2.pixelSize) ||
	    (block1.pixelSize != 4)) {
	    Tcl_SetResult(interp, "incompatible photo images", TCL_STATIC);
	    return TCL_ERROR;
	}
	mask = (mask0 >> 24) & 0xff;	/* alpha */
	if (mask != 0) {
	    for (y = 0; y < block1.height; y++) {
		dst = block1.pixelPtr + y * block1.pitch;
		src = block2.pixelPtr + y * block2.pitch;
		dst += block1.offset[3];
		src += block2.offset[3];
		for (x = 0; x < block1.width; x++) {
		    *dst = (*dst & (~mask)) | (*src & mask);
		    dst += block1.pixelSize;
		    src += block2.pixelSize;
		}
	    }
	    ++nops;
	}
	mask = (mask0 >> 16) & 0xff;	/* red */
	if (mask != 0) {
	    for (y = 0; y < block1.height; y++) {
		dst = block1.pixelPtr + y * block1.pitch;
		src = block2.pixelPtr + y * block2.pitch;
		dst += block1.offset[0];
		src += block2.offset[0];
		for (x = 0; x < block1.width; x++) {
		    *dst = (*dst & (~mask)) | (*src & mask);
		    dst += block1.pixelSize;
		    src += block2.pixelSize;
		}
	    }
	    ++nops;
	}
	mask = (mask0 >> 8) & 0xff;	/* green */
	if (mask != 0) {
	    for (y = 0; y < block1.height; y++) {
		dst = block1.pixelPtr + y * block1.pitch;
		src = block2.pixelPtr + y * block2.pitch;
		dst += block1.offset[1];
		src += block2.offset[1];
		for (x = 0; x < block1.width; x++) {
		    *dst = (*dst & (~mask)) | (*src & mask);
		    dst += block1.pixelSize;
		    src += block2.pixelSize;
		}
	    }
	    ++nops;
	}
	mask = mask0 & 0xff;		/* blue */
	if (mask != 0) {
	    for (y = 0; y < block1.height; y++) {
		dst = block1.pixelPtr + y * block1.pitch;
		src = block2.pixelPtr + y * block2.pitch;
		dst += block1.offset[2];
		src += block2.offset[2];
		for (x = 0; x < block1.width; x++) {
		    *dst = (*dst & (~mask)) | (*src & mask);
		    dst += block1.pixelSize;
		    src += block2.pixelSize;
		}
	    }
	    ++nops;
	}
	if (nops) {
	    ret = Tk_PhotoPutBlock(interp, ph1, &block1, 0, 0,
				   block1.width, block1.height,
				   TK_PHOTO_COMPOSITE_SET);
	}
	break;
    }

    case CMD_mirror: {
	int x, y;

	if ((objc != 3) && (objc != 5)) {
	    Tcl_WrongNumArgs(interp, 2, objv, "devid ?x y?");
	    return TCL_ERROR;
	}
	hPtr = Tcl_FindHashEntry(&v4l2i->v4l2c, Tcl_GetString(objv[2]));
	if (hPtr == NULL) {
	    goto devNotFound;
	}
	v4l2c = (V4L2C *) Tcl_GetHashValue(hPtr);
	if ((objc > 3) &&
	    ((Tcl_GetBooleanFromObj(interp, objv[3], &x) != TCL_OK) ||
	     (Tcl_GetBooleanFromObj(interp, objv[4], &y) != TCL_OK))) {
	    return TCL_ERROR;
	}
	if (objc > 3) {
	    v4l2c->mirror = (x ? 1 : 0) | (y ? 2 : 0);
	} else {
	    Tcl_Obj *list[2];

	    list[0] = Tcl_NewBooleanObj(v4l2c->mirror & 1);
	    list[1] = Tcl_NewBooleanObj(v4l2c->mirror & 2);
	    Tcl_SetObjResult(interp, Tcl_NewListObj(2, list));
	}
	break;
    }

    case CMD_open: {
	struct v4l2_format fmt;
	struct v4l2_streamparm stp;
	char *devName;
	Tcl_HashSearch search;
	int fd, fps = 15, loop = 0, isNew;
	struct stat sb;
	dev_t dt[2];
#ifdef linux
	int fd2 = -1;
#endif

	if (objc != 4) {
	    Tcl_WrongNumArgs(interp, 2, objv, "device callback");
	    return TCL_ERROR;
	}
	devName = Tcl_GetString(objv[2]);
	if (stat(devName, &sb) < 0) {
	    Tcl_SetObjResult(interp,
		Tcl_ObjPrintf("error while checking \"%s\": %s",
			      devName, Tcl_PosixError(interp)));
	    return TCL_ERROR;
	}
	dt[0] = sb.st_rdev;
	hPtr = Tcl_FirstHashEntry(&v4l2i->v4l2c, &search);
	while (hPtr != NULL) {
	    v4l2c = (V4L2C *) Tcl_GetHashValue(hPtr);
	    memset(&sb, 0, sizeof (sb));
	    fstat(v4l2c->fd, &sb);
	    dt[1] = sb.st_rdev;
	    if ((strcmp(devName, Tcl_DStringValue(&v4l2c->devName)) == 0) ||
		(dt[0] == dt[1])) {
		Tcl_SetObjResult(interp,
		    Tcl_ObjPrintf("\"%s\" is already open for \"%s\"",
			    devName, v4l2c->devId));
		return TCL_ERROR;
	    }
	    hPtr = Tcl_NextHashEntry(&search);
	}
#ifdef linux
	if (IsLoopDevice(devName)) {
	    int type;

	    fd2 = open(devName, O_RDWR | O_NONBLOCK, 0);
	    if (fd2 >= 0) {
		fcntl(fd2, F_SETFD, FD_CLOEXEC);
		/* Cheat write side, otherwise v4l2_open() might fail. */
		memset(&fmt, 0, sizeof (fmt));
		fmt.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
		DoIoctl(fd2, VIDIOC_G_FMT, &fmt);
		type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
		DoIoctl(fd2, VIDIOC_STREAMON, &type);
		loop = 1;
	    }
	}
#endif
	fd = v4l2_open(devName, O_RDWR | O_NONBLOCK, 0);
	if (fd < 0) {
	    Tcl_SetObjResult(interp,
		Tcl_ObjPrintf("error while opening \"%s\": %s",
			      devName, Tcl_PosixError(interp)));
#ifdef linux
	    if (fd2 >= 0) {
		close(fd2);
	    }
#endif
	    return TCL_ERROR;
	}
	fcntl(fd, F_SETFD, FD_CLOEXEC);
	memset(&fmt, 0, sizeof (fmt));
	fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if (DoIoctl(fd, VIDIOC_G_FMT, &fmt) < 0) {
	    Tcl_SetObjResult(interp,
		Tcl_ObjPrintf("error querying format: %s",
			      Tcl_PosixError(interp)));
	    v4l2_close(fd);
#ifdef linux
	    if (fd2 >= 0) {
		close(fd2);
	    }
#endif
	    return TCL_ERROR;
	}
	memset(&stp, 0, sizeof (stp));
	stp.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if (DoIoctl(fd, VIDIOC_G_PARM, &stp) >= 0) {
	    if ((stp.parm.capture.capability & V4L2_CAP_TIMEPERFRAME) &&
		(stp.parm.capture.timeperframe.numerator > 0)) {
		fps = stp.parm.capture.timeperframe.denominator /
		    stp.parm.capture.timeperframe.numerator;
		fps = ((fps > 0) && (fps < 200)) ? fps : 15;
	    }
	}
#ifdef linux
	if (fd2 >= 0) {
	    close(fd2);
	}
#endif
	v4l2c = (V4L2C *) ckalloc(sizeof (V4L2C));
	memset(v4l2c, 0, sizeof (V4L2C));
	v4l2c->fd = fd;
	v4l2c->isLoopDev = loop;
	v4l2c->format = v4l2c->wantFormat = 0;
	v4l2c->greyshift = 4;
	v4l2c->nvbufs = 0;
	v4l2c->mirror = 0;
	v4l2c->rotate = 0;
	v4l2c->bufrdy = -1;
	v4l2c->width = fmt.fmt.pix.width;
	if (v4l2c->width < 0) {
	    v4l2c->width = 640;
	}
	v4l2c->height = fmt.fmt.pix.height;
	if (v4l2c->height < 0) {
	    v4l2c->height = 320;
	}
	v4l2c->fps = fps;
	v4l2c->interp = interp;
	Tcl_DStringInit(&v4l2c->devName);
	Tcl_DStringAppend(&v4l2c->devName, devName, -1);
	Tcl_DStringInit(&v4l2c->cbCmd);
	Tcl_DStringAppend(&v4l2c->cbCmd, Tcl_GetString(objv[3]), -1);
	v4l2c->cbCmdLen = Tcl_DStringLength(&v4l2c->cbCmd);
	Tcl_InitHashTable(&v4l2c->ctrl, TCL_ONE_WORD_KEYS);
	Tcl_InitHashTable(&v4l2c->nctrl, TCL_STRING_KEYS);
	Tcl_DStringInit(&v4l2c->fsize.ds);
	Tcl_DStringInit(&v4l2c->frate.ds);
	InitControls(v4l2c);
	if (loop) {
	    if (DoIoctl(fd, VIDIOC_G_FMT, &fmt) < 0) {
		v4l2c->loopFormat = 0;
		v4l2c->loopWidth = 0;
		v4l2c->loopHeight = 0;
	    } else {
		v4l2c->loopFormat = fmt.fmt.pix.pixelformat;
		v4l2c->loopWidth = fmt.fmt.pix.width;
		v4l2c->loopHeight = fmt.fmt.pix.height;
	    }
	}
	sprintf(v4l2c->devId, "vdev%d", v4l2i->idCount++);
	hPtr = Tcl_CreateHashEntry(&v4l2i->v4l2c, v4l2c->devId, &isNew);
	Tcl_SetHashValue(hPtr, (ClientData) v4l2c);
	Tcl_SetObjResult(interp, Tcl_NewStringObj(v4l2c->devId, -1));
	break;
    }

    case CMD_orientation: {
	if (objc > 4) {
	    Tcl_WrongNumArgs(interp, 2, objv, "devid ?degrees?");
	    return TCL_ERROR;
	}
	hPtr = Tcl_FindHashEntry(&v4l2i->v4l2c, Tcl_GetString(objv[2]));
	if (hPtr == NULL) {
	    goto devNotFound;
	}
	v4l2c = (V4L2C *) Tcl_GetHashValue(hPtr);
	if (objc > 3) {
	    int degrees;

	    if (Tcl_GetIntFromObj(interp, objv[3], &degrees) != TCL_OK) {
		return TCL_ERROR;
	    }
	    degrees = degrees % 360;
	    if (degrees < 45) {
		v4l2c->rotate = 0;
	    } else if (degrees < 135) {
		v4l2c->rotate = 90;
	    } else if (degrees < 225) {
		v4l2c->rotate = 180;
	    } else if (degrees < 315) {
		v4l2c->rotate = 270;
	    } else {
		v4l2c->rotate = 0;
	    }
	} else {
	    Tcl_SetObjResult(interp, Tcl_NewIntObj(v4l2c->rotate));
	}
	break;
    }

    case CMD_parameters:
	if ((objc < 3) || (objc % 2 == 0)) {
	    Tcl_WrongNumArgs(interp, 2, objv, "devid ?key value ...?");
	    return TCL_ERROR;
	}
	hPtr = Tcl_FindHashEntry(&v4l2i->v4l2c, Tcl_GetString(objv[2]));
	if (hPtr != NULL) {
	    v4l2c = (V4L2C *) Tcl_GetHashValue(hPtr);
	    if (objc == 3) {
		Tcl_Obj *list = Tcl_NewListObj(0, NULL);

		GetControls(v4l2c, list);
		Tcl_SetObjResult(interp, list);
	    } else {
		ret = SetControls(v4l2c, objc - 3, objv + 3);
		if (ret == TCL_OK) {
		    Tcl_Obj *list = Tcl_NewListObj(0, NULL);

		    GetControls(v4l2c, list);
		    Tcl_SetObjResult(interp, list);
		}
	    }
	} else {
	    goto devNotFound;
	}
	break;

    case CMD_start:
	if (objc != 3) {
	    Tcl_WrongNumArgs(interp, 2, objv, "devid");
	    return TCL_ERROR;
	}
	hPtr = Tcl_FindHashEntry(&v4l2i->v4l2c, Tcl_GetString(objv[2]));
	if (hPtr != NULL) {
	    v4l2c = (V4L2C *) Tcl_GetHashValue(hPtr);
	    ret = StartCapture(v4l2c);
	} else {
	    goto devNotFound;
	}
	break;

    case CMD_state:
	if (objc != 3) {
	    Tcl_WrongNumArgs(interp, 2, objv, "devid");
	    return TCL_ERROR;
	}
	hPtr = Tcl_FindHashEntry(&v4l2i->v4l2c, Tcl_GetString(objv[2]));
	if (hPtr != NULL) {
	    v4l2c = (V4L2C *) Tcl_GetHashValue(hPtr);
	    Tcl_SetResult(interp, (v4l2c->running < 0) ? "error" :
			  (v4l2c->running ? "capture" : "stopped"),
			  TCL_STATIC);
	} else {
	    goto devNotFound;
	}
	break;

    case CMD_stop:
	if (objc != 3) {
	    Tcl_WrongNumArgs(interp, 2, objv, "devid");
	    return TCL_ERROR;
	}
	hPtr = Tcl_FindHashEntry(&v4l2i->v4l2c, Tcl_GetString(objv[2]));
	if (hPtr != NULL) {
	    v4l2c = (V4L2C *) Tcl_GetHashValue(hPtr);
	    ret = StopCapture(v4l2c);
	} else {
	    goto devNotFound;
	}
	break;

    case CMD_tophoto:
	if (DataToPhoto(v4l2i, interp, objc, objv) != TCL_OK) {
	    return TCL_ERROR;
	}
	break;

    case CMD_write: {
	unsigned char *data, *toFree = NULL;
	int length, n;

	if (objc != 4) {
	    Tcl_WrongNumArgs(interp, 2, objv, "devid bytearray");
	    return TCL_ERROR;
	}
	hPtr = Tcl_FindHashEntry(&v4l2i->v4l2c, Tcl_GetString(objv[2]));
	if (hPtr != NULL) {
	    v4l2c = (V4L2C *) Tcl_GetHashValue(hPtr);
	} else {
	    goto devNotFound;
	}
	if (!v4l2c->isLoopDev) {
	    Tcl_SetResult(interp, "not a loop device", TCL_STATIC);
	    return TCL_ERROR;
	}
	data = Tcl_GetByteArrayFromObj(objv[3], &length);
	if ((length != v4l2c->loopWidth * v4l2c->loopHeight * 3) &&
	    (length != v4l2c->loopWidth * v4l2c->loopHeight * 4) &&
	    (length != v4l2c->loopWidth * v4l2c->loopHeight)) {
	    Tcl_SetResult(interp, "unsupported width or height", TCL_STATIC);
	    return TCL_ERROR;
	}
	if ((v4l2c->loopFormat == V4L2_PIX_FMT_YUYV) ||
	    (v4l2c->loopFormat == V4L2_PIX_FMT_YVYU) ||
	    (v4l2c->loopFormat == V4L2_PIX_FMT_GREY)) {
	    Tk_PhotoImageBlock block;

	    block.offset[0] = 0;
	    block.offset[1] = 1;
	    block.offset[2] = 2;
	    block.offset[3] = 4;
	    block.pixelSize = 3;
	    if (length == v4l2c->loopWidth * v4l2c->loopHeight * 4) {
		block.pixelSize += 1;
		block.offset[3] -= 1;
	    } else if (length == v4l2c->loopWidth * v4l2c->loopHeight) {
		block.pixelSize = 1;
		block.offset[1] = 0;
		block.offset[2] = 0;
		block.offset[3] = 2;
	    }
	    block.pixelPtr = data;
	    block.width = v4l2c->loopWidth;
	    block.height = v4l2c->loopHeight;
	    block.pitch = block.pixelSize * block.width;
	    if ((v4l2c->loopFormat == V4L2_PIX_FMT_GREY) &&
		(block.pixelSize == 1)) {
		data = block.pixelPtr;
	    } else {
		toFree = ConvertToYUV(&block,
				      v4l2c->loopFormat == V4L2_PIX_FMT_YVYU,
				      &length);
		if (toFree == NULL) {
		    Tcl_SetResult(interp, "out of memory", TCL_STATIC);
		    return TCL_ERROR;
		}
		data = toFree;
	    }
	}
	n = write(v4l2c->fd, data, length);
	if (toFree != NULL) {
	    ckfree(toFree);
	}
	if (n == -1) {
	    Tcl_SetObjResult(interp,
			 Tcl_ObjPrintf("write error: %s",
				       Tcl_PosixError(interp)));
	    return TCL_ERROR;
	}
	break;
    }

    case CMD_writephoto: {
	char *name;
	Tk_PhotoHandle ph;
	Tk_PhotoImageBlock block;
	int length, n;
	unsigned char *toFree = NULL;

	if (objc != 4) {
	    Tcl_WrongNumArgs(interp, 2, objv, "devid photo");
	    return TCL_ERROR;
	}
	hPtr = Tcl_FindHashEntry(&v4l2i->v4l2c, Tcl_GetString(objv[2]));
	if (hPtr != NULL) {
	    v4l2c = (V4L2C *) Tcl_GetHashValue(hPtr);
	} else {
	    goto devNotFound;
	}
	if (!v4l2c->isLoopDev) {
	    Tcl_SetResult(interp, "not a loop device", TCL_STATIC);
	    return TCL_ERROR;
	}
	if (CheckForTk(v4l2i, interp) != TCL_OK) {
	    return TCL_ERROR;
	}
	name = Tcl_GetString(objv[3]);
	ph = Tk_FindPhoto(interp, name);
	if (ph == NULL) {
	    Tcl_SetObjResult(interp,
		Tcl_ObjPrintf("can't use \"%s\": not a photo image", name));
	    return TCL_ERROR;
	}
	Tk_PhotoGetImage(ph, &block);
	if ((block.pitch != block.width * block.pixelSize) ||
	    (block.pixelSize != 4)) {
	    Tcl_SetResult(interp, "unsupported photo format", TCL_STATIC);
	    return TCL_ERROR;
	}
	if ((block.width != v4l2c->loopWidth) ||
	    (block.height != v4l2c->loopHeight)) {
	    Tcl_SetResult(interp, "unsupported width or height", TCL_STATIC);
	    return TCL_ERROR;
	}
	length = block.pitch * block.height * block.pixelSize;
	if ((v4l2c->loopFormat == V4L2_PIX_FMT_YUYV) ||
	    (v4l2c->loopFormat == V4L2_PIX_FMT_YVYU)) {
	    toFree =
		ConvertToYUV(&block, v4l2c->loopFormat == V4L2_PIX_FMT_YVYU,
			     &length);
	    if (toFree == NULL) {
		Tcl_SetResult(interp, "out of memory", TCL_STATIC);
		return TCL_ERROR;
	    }
	    n = write(v4l2c->fd, toFree, length);
	} else if (v4l2c->loopFormat == V4L2_PIX_FMT_GREY) {
	    toFree = ConvertToGREY(&block, &length);
	    if (toFree == NULL) {
		Tcl_SetResult(interp, "out of memory", TCL_STATIC);
		return TCL_ERROR;
	    }
	    n = write(v4l2c->fd, toFree, length);
	} else {
	    n = write(v4l2c->fd, block.pixelPtr, length);
	}
	if (toFree != NULL) {
	    ckfree(toFree);
	}
	if (n == -1) {
	    Tcl_SetObjResult(interp, Tcl_ObjPrintf("write error: %s",
						   Tcl_PosixError(interp)));
	    return TCL_ERROR;
	}
	break;
    }

    }

    return ret;
}

/*
 *-------------------------------------------------------------------------
 *
 * V4l2_Init --
 *
 *	Module initializer:
 *	  - require Tcl/Tk infrastructure
 *	  - dynamic link libv4l2 and (optional) libudev
 *	  - initialize module data structures
 *	  - (optional) setup udev for plug/unplug events
 *
 * Results:
 *	A standard Tcl result.
 *
 * Side effects:
 *	See the user documentation.
 *
 *-------------------------------------------------------------------------
 */

int
V4l2_Init(Tcl_Interp *interp)
{
    V4L2I *v4l2i;

#ifdef USE_TCL_STUBS
    if (Tcl_InitStubs(interp, "8.4", 0) == NULL) {
	return TCL_ERROR;
    }
#else
    if (Tcl_PkgRequire(interp, "Tcl", "8.4", 0) == NULL) {
	return TCL_ERROR;
    }
#endif
    if (Tcl_PkgProvide(interp, PACKAGE_NAME, PACKAGE_VERSION) != TCL_OK) {
	return TCL_ERROR;
    }

    if (!v4l2Initialized) {
	int fd;

	Tcl_MutexLock(&v4l2Mutex);
	if (v4l2Initialized) {
	    Tcl_MutexUnlock(&v4l2Mutex);
	    goto doInit;
	}

	/* dynamic link libv4l2 */
	(void) dlerror();
	libv4l2 = dlopen("libv4l2.so.0", RTLD_NOW);
	if (libv4l2 == NULL) {
libv4l2Error:
	    if (libv4l2 != NULL) {
		dlclose(libv4l2);
		libv4l2 = NULL;
	    }
	    Tcl_SetObjResult(interp,
		    Tcl_ObjPrintf("unable to link libv4l2.so: %s", dlerror()));
	    Tcl_MutexUnlock(&v4l2Mutex);
	    return TCL_ERROR;
	}

#define V4L2DLSYM(name)							\
	v4l2_dl.name = (fn_ ## name) dlsym(libv4l2, "v4l2_" #name);	\
	if (v4l2_dl.name == NULL) goto libv4l2Error

#define V4L2DLSYMT(name, type)						\
	v4l2_dl.name = (type) dlsym(libv4l2, "v4l2_" #name);		\
	if (v4l2_dl.name == NULL) goto libv4l2Error

	V4L2DLSYMT(log_file, FILE **);
	V4L2DLSYM(open);
	V4L2DLSYM(close);
	V4L2DLSYM(ioctl);
	V4L2DLSYM(mmap);
	V4L2DLSYM(munmap);

#undef V4L2DLSYM
#undef V4L2DLSYMT

#ifdef HAVE_LIBUDEV
	/* dynamic link libudev */
	libudev = dlopen("libudev.so.1", RTLD_NOW);
	if (libudev == NULL) {
	    libudev = dlopen("libudev.so.0", RTLD_NOW);
	    if (libudev == NULL) {
		goto libudevEnd;
libudevError:
		dlclose(libudev);
		libudev = NULL;
		goto libudevEnd;
	    }
	}

#define UDEVDLSYM(name)							\
	udev_dl.name = (fn_ ## name) dlsym(libudev, "udev_" #name);	\
	if (udev_dl.name == NULL) goto libudevError

	UDEVDLSYM(device_get_action);
	UDEVDLSYM(device_get_devnode);
	UDEVDLSYM(device_new_from_syspath);
	UDEVDLSYM(device_unref);
	UDEVDLSYM(monitor_get_fd);
	UDEVDLSYM(monitor_receive_device);
	UDEVDLSYM(monitor_unref);
	UDEVDLSYM(new);
	UDEVDLSYM(unref);
	UDEVDLSYM(monitor_enable_receiving);
	UDEVDLSYM(monitor_filter_add_match_subsystem_devtype);
	UDEVDLSYM(monitor_new_from_netlink);
	UDEVDLSYM(enumerate_new);
	UDEVDLSYM(enumerate_add_match_subsystem);
	UDEVDLSYM(enumerate_get_list_entry);
	UDEVDLSYM(enumerate_scan_devices);
	UDEVDLSYM(enumerate_unref);
	UDEVDLSYM(list_entry_get_name);
	UDEVDLSYM(list_entry_get_next);

#undef UDEVDLSYM

libudevEnd:
	;
#endif

	/* redirect libv4l2 log output to /dev/null */
	fd = open("/dev/null", O_WRONLY);
	if (fd >= 0) {
	    fcntl(fd, F_SETFD, FD_CLOEXEC);
	    v4l2_log_file = fdopen(fd, "w");
	    if (v4l2_log_file == NULL) {
		close(fd);
	    }
	}

	v4l2Initialized = 1;
	Tcl_MutexUnlock(&v4l2Mutex);
    }

doInit:
    v4l2i = (V4L2I *) ckalloc(sizeof (V4L2I));
    memset(v4l2i, 0, sizeof (V4L2I));
    v4l2i->idCount = 0;
    Tcl_InitHashTable(&v4l2i->v4l2c, TCL_STRING_KEYS);
#ifdef HAVE_LIBUDEV
    /* setup udev */
    v4l2i->interp = interp;
    Tcl_InitHashTable(&v4l2i->vdevs, TCL_STRING_KEYS);
    Tcl_DStringInit(&v4l2i->cbCmd);
    v4l2i->cbCmdLen = 0;
    v4l2i->udev = (libudev == NULL) ? NULL : udev_new();
    if (v4l2i->udev != NULL) {
	v4l2i->udevMon = udev_monitor_new_from_netlink(v4l2i->udev, "udev");
	if (v4l2i->udevMon == NULL) {
	    udev_unref(v4l2i->udev);
	    v4l2i->udev = NULL;
	}
    }
    if (v4l2i->udevMon != NULL) {
	struct udev_enumerate *udevEnum;
	struct udev_list_entry *item;
	struct udev_device *dev;
	Tcl_HashEntry *hPtr;
	const char *devName;
	int isNew;

	/* watch "video4linux" subsystem */
	udev_monitor_filter_add_match_subsystem_devtype(v4l2i->udevMon,
							"video4linux", NULL);
	udev_monitor_enable_receiving(v4l2i->udevMon);
	Tcl_CreateFileHandler(udev_monitor_get_fd(v4l2i->udevMon),
			      TCL_READABLE, UdevMonitor, (ClientData) v4l2i);
	/* initial device scan */
	udevEnum = udev_enumerate_new(v4l2i->udev);
	if (udevEnum == NULL) {
	    /* trouble */
	    Tcl_DeleteFileHandler(udev_monitor_get_fd(v4l2i->udevMon));
	    udev_monitor_unref(v4l2i->udevMon);
	    v4l2i->udevMon = NULL;
	    udev_unref(v4l2i->udev);
	    v4l2i->udev = NULL;
	    goto endUdevInit;
	}
	udev_enumerate_add_match_subsystem(udevEnum, "video4linux");
	udev_enumerate_scan_devices(udevEnum);
	item = udev_enumerate_get_list_entry(udevEnum);
	while (item != NULL) {
	    dev = udev_device_new_from_syspath(v4l2i->udev,
					       udev_list_entry_get_name(item));

	    if (dev != NULL) {
		devName = udev_device_get_devnode(dev);
		hPtr = Tcl_CreateHashEntry(&v4l2i->vdevs, (ClientData) devName,
					   &isNew);
		Tcl_SetHashValue(hPtr, (ClientData)
				 Tcl_GetHashKey(&v4l2i->vdevs, hPtr));
	    }
	    item = udev_list_entry_get_next(item);
	}
	udev_enumerate_unref(udevEnum);
    }
endUdevInit:
    ;
#endif

    Tcl_CreateObjCommand(interp, "v4l2", V4l2ObjCmd,
			 (ClientData) v4l2i, V4l2ObjCmdDeleted);
    return TCL_OK;
}

/*
 * Local Variables:
 * mode: c
 * c-basic-offset: 4
 * fill-column: 78
 * tab-width: 8
 * End:
 */
