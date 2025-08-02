package require Tk
package require v4l2

set DEVNAME [lindex $argv 0]
if {$DEVNAME eq ""} {
    set DEVNAME /dev/video2
}

image create photo P
P configure -width 320 -height 240

# set format on loopback device
v4l2 loopback $DEVNAME RGB3 320 240 10

# callback: image ready or error
proc imagecb {dev ind} {
    if {$ind ne "error"} {
	v4l2 image $dev P
    }
}

# make RGB3 gray image
proc mkimage {dev} {
    after 100 [list mkimage $dev]
    set hex [format "%02X" [expr {$::N % 0xff}]]
    incr ::N 8
    set hex [string repeat $hex [expr {320 * 240 * 3}]]
    v4l2 write $dev [binary decode hex $hex]
}

label .label -image P
pack .label

set V [v4l2 open $DEVNAME imagecb]
v4l2 start $V

set N 0
after 100 [list mkimage $V]

