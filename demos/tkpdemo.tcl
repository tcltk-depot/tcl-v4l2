package require Tk
package require tkpath
package require v4l2

if {[info command "sdltk"] eq "sdltk"} {
    wm attributes . -fullscreen 1
}

pack [tkp::canvas .c -bg "#c6ceef" -highlightthickness 0] -fill both -expand 1

set dev [lindex [v4l2 devices] 0]
set dev [v4l2 open $dev getimage]

foreach fs [split [dict get [v4l2 parameters $dev] frame-size-values] ","] {
    lassign [split $fs "x"] width height
    if {![info exists minwidth] || ($width < $minwidth)} {
	set minwidth $width
	set minfs $fs
    }
}

v4l2 parameters $dev frame-size $minfs
lassign [split $minfs "x"] width height

set x0 [expr {1.5 * $width - 20}]
set y0 $x0
set dx [expr {$width * 1.3}]
set dy $dx

set anchors [list nw n ne w c e sw s se]
for {set i 0} {$i < [llength $anchors] } {incr i} {
    set a [lindex $anchors $i]
    set x [expr {$x0 + $i%3 * $dx}]
    set y [expr {$x0 + $i/3 * $dy}]
    set t $a
    image create photo img$i
    img$i configure -width $width -height $height
    .c create pimage $x $y -image img$i -anchor $a -tags $a
    .c create ptext $x $y -text $t -fontfamily "Times" \
	-fontsize 23 -fill black -stroke white -strokewidth 3 \
	-filloverstroke 1 -textanchor middle
    .c create path "M $x $y m -5 0 h 10 m -5 -5 v 10" -stroke red
}

proc getimage {dev ind} {
    if {$ind eq "error"} {
        return
    }
    set ind [expr {$ind % [llength $::anchors]}]
    v4l2 image $dev img$ind
    if {$ind != 4} {
	# full rate in center image
        v4l2 image $dev img4
    }
}

proc ticker {deg step tim} {
    if {[winfo exists .c]} {
	after $tim [list ticker [expr {[incr deg $step] % 360}] $step $tim]
	set phi [expr 2*$deg*3.14159/360.0]
	for {set i 0} {$i < [llength $::anchors]} {incr i} {
	    set a [lindex $::anchors $i]
	    set x [expr {$::x0 + $i%3 * $::dx}]
	    set y [expr {$::y0 + $i/3 * $::dy}]
	    set m [::tkp::matrix rotate $phi $x $y]
	    .c itemconfigure $a -m $m
	}
    }
}

bind all <Key-q> exit
bind all <Escape> exit

ticker 0 1 100
v4l2 start $dev
