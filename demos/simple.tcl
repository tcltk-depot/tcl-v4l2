package require Tk
package require v4l2

set mask {}

# callback: (un)plugged device
proc devicecb {but img op dev} {
    set dev [v4l2 info]
    if {$dev eq ""} {
	init $but $img
    } else {
	v4l2 close $dev
	$but configure -text "No Camera"
    }
}

# callback: image ready or error
proc imagecb {but img dev ind} {
    if {$ind eq "error"} {
	$but configure -text "Start"
    } elseif {$::mask ne {}} {
	v4l2 greyimage $dev $::mask $img
    } else {
	v4l2 image $dev $img
    }
}

# button handler: start/stop image capture
proc startstop {but} {
    set dev [v4l2 info]
    if {$dev eq ""} {
	return
    }
    switch -glob -- [v4l2 state $dev] {
	capture {
	    v4l2 stop $dev
	    $but configure -text "Start"
	}
	* {
	    v4l2 start $dev
	    $but configure -text "Stop"
	}
    }
}

# init device
proc init {but img {name {}}} {
    if {$name eq ""} {
	set name [lindex [v4l2 devices] 0]
    }
    if {$name eq ""} {
	$but configure -text "No Camera"
	return
    }
    set dev [v4l2 open $name [list imagecb $but $img]]
    array set d [v4l2 parameters $dev]
    if {[info exists d(frame-size)] &&
	[scan $d(frame-size) %dx%d w h] == 2} {
	$img configure -width $w -height $h
	$img blank
    }
    $but configure -text "Start"
}

proc changemirror {w} {
    set dev [v4l2 info]
    if {$dev ne ""} {
	lassign [v4l2 mirror $dev] x y
	set n [expr {$x + $y * 2 + 1}]
	v4l2 mirror $dev [expr {$n & 1}] [expr {$n & 2}]
    }
}

proc changeorientation {w} {
    set dev [v4l2 info]
    if {$dev ne ""} {
	set n [v4l2 orientation $dev]
	set n [expr {($n + 90) % 360}]
	v4l2 orientation $dev $n
	set img [$w cget -image]
	$img configure -width 1 -height 1
	$img configure -width 0 -height 0
    }
}

proc changemask {w} {
    set all {{} r g b rg rb gb rgb}
    set idx [lsearch $all $::mask]
    incr idx
    set ::mask [lindex $all $idx]
}

proc changeparam {which dir} {
    set dev [v4l2 info]
    array set p [v4l2 parameters $dev]
    if {[info exists p(${which}-absolute)]} {
	if {[info exists p(${which}-absolute-step)]} {
	    set dir [expr {$dir * $p(${which}-absolute-step)}]
	}
	set newval [expr {$p(${which}-absolute) + $dir}]
	v4l2 parameters $dev ${which}-absolute $newval
    } elseif {[info exists p($which)]} {
	if {[info exists p(${which}-step)]} {
	    set dir [expr {$dir * $p(${which}-step)}]
	}
	set newval [expr {$p($which) + $dir}]
	v4l2 parameters $dev $which $newval
    }
}

# user interface
button .b -command [list startstop .b]
label .l -image [image create photo]
pack .b .l -side top

bind .l <1> {changemirror %W}
bind .l <2> {changemask %W}
bind .l <3> {changeorientation %W}

bind . <Key-plus>  {changeparam zoom 1}
bind . <Key-minus> {changeparam zoom -1}
bind . <Key-Up>    {changeparam tilt 1}
bind . <Key-Down>  {changeparam tilt -1}
bind . <Key-Right> {changeparam pan 1}
bind . <Key-Left>  {changeparam pan -1}
bind . <Key-Prior> {changeparam focus-absolute 1}
bind . <Key-Next>  {changeparam focus-absolute -1}
bind . <Key-g>     {changeparam gain 1}
bind . <Shift-G>   {changeparam gain -1}
bind . <Key-s>     {changeparam saturation 1}
bind . <Shift-S>   {changeparam saturation -1}
bind . <Key-c>     {changeparam contrast 1}
bind . <Shift-C>   {changeparam contrast -1}
bind . <Key-b>     {changeparam brightness 1}
bind . <Shift-B>   {changeparam brightness -1}
bind . <Key-x>     {changeparam sharpness 1}
bind . <Shift-X>   {changeparam sharpness -1}
bind . <Key-f>     {changeparam focus-auto 1}
bind . <Shift-F>   {changeparam focus-auto -1}

# watch for (un)plugged devices
v4l2 listen [list devicecb .b [.l cget -image]]
# try to init device
init .b [.l cget -image]
