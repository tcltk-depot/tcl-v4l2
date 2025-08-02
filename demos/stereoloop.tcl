# Make anaglyph of a stereo camera and send
# resulting frames into v4l2 loopback device

package require v4l2

# Setup loopback device, use YUV here since
# jitsi.org in FireFox does not support RGB.
v4l2 loopback /dev/video2 YUYV 640 480 25

# Open loopback device for later writes
set loop [v4l2 open /dev/video2 #no-image-callback]

# Callback: image ready or error
proc image_callback {dev ind} {
    if {$ind eq "error"} {
	puts stderr "acquisition error, exiting"
	set ::forever 0
    }
    # Fetch image
    set n $::devs($dev)
    set ::imgs($n) [v4l2 image $dev]
    # When both images available, combine to anaglyph
    # and output on loopback device
    catch {
	lassign $::imgs(0) width0 height0 bpp0 img0
	lassign $::imgs(1) width1 height1 bpp1 img1
	v4l2 mbcopy $img1 $img0 0x00ff0000
	if {$n == 1} {
	    v4l2 write $::loop $img1
	}
    }
}

# Initialize cameras
proc init_cameras {} {
    set count 0
    # Find cameras by friendly name
    foreach name [glob -nocomplain /dev/v4l/by-id/*Stereo_Vision*] {
	set dev [v4l2 open $name image_callback]
	# Use MJPEG, otherwise USB 2.0 bandwidth is insufficient
	v4l2 parameters $dev frame-rate 25 frame-size 640x480@MJPG
	set ::devs($dev) $count
	incr count
	if {$count > 1} {
	    break
	}
    }
    if {$count != 2} {
	puts stderr "need two cameras"
	exit 1
    }
    # Start both cameras
    foreach dev [array names ::devs] {
	v4l2 start $dev
    }
}

init_cameras

# Need an event loop for operation
vwait forever
