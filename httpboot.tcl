# Internet-Bootloader for Atmega1284p and WizNet W5500, developped using Atmel Studio.
# See http://s.wangnick.de/doku.php?id=iot-basisstation for target hardware.
# (C) Copyright 2014 Sebastian Wangnick.
# Usage under "CC Attribution-Noncommercial-Share Alike 3.0 Unported" as described in http://creativecommons.org/licenses/by-nc-sa/3.0/ is granted.


proc data {text addr} {
  if {![info exists ::cnt(data)]} {set ::cnt(data) -1}
  set row [incr ::cnt(data)]
  grid [label .data.l$row -text $text] -sticky nw -row $row -column 0
  grid [label .data.a$row -text [format "0x%04X" $addr]] -sticky nw -row $row -column 1
  grid [entry .data.e$row -textvariable ::data($text)] -sticky we -row $row -column 2
  grid columnconfigure .data {0 1} -minsize 1c
  grid columnconfigure .data 2 -weight 1
}
set eeprom {{} 0x10 GWIP 4 MASK 4 MAC 6 MYIP 4 DNSIP 4 URL 0}
set mcusrbits {po ex bo wd jt}

pack [frame .data] -expand 1 -fill x
set addr 0x0000
foreach {text len} $eeprom {
  if {$text ne ""} {
    data $text $addr
  }
  incr addr $len
}

proc status {text} {
  if {![info exists ::cnt(status)]} {set ::cnt(status) -1}
  pack [label .status.l[incr ::cnt(status)] -text $text] -side left
  pack [label .status.l[incr ::cnt(status)] -textvariable ::status([string tolower $text])] -side left
}

pack [frame .status] -expand 1 -fill x
foreach {text val} {Port com3: Status ... MCUSR ...} {
  status $text
  set ::status([string tolower $text]) $val
}

pack [frame .menu] -expand 1 -fill x
grid [button .menu.b1 -text Read -command doread] -sticky we -row 0 -column 0
grid [button .menu.b2 -text Write -command dowrite] -sticky we -row 0 -column 1
grid [button .menu.b3 -text Boot -command "docmd B"] -sticky we -row 0 -column 2
grid [button .menu.b4 -text Go -command "docmd G"] -sticky we -row 0 -column 3
grid [button .menu.b5 -text Watchdog-Reset -command "docmd T"] -sticky we -row 0 -column 4
grid columnconfigure .menu all -weight 1 -uniform 1
proc buttonstate {state} {
  foreach b [winfo children .menu] {
    $b configure -state $state
  }
}

pack [text .log] -expand 1 -fill both
proc log {msg} {.log insert end $msg; .log see end}
proc esc {msg} {return [string map {"\n" "\\n"} $msg]}

after 1000 {tryopen}

proc tryopen {} {
  log "Opening serial port\n"
  if {[catch {set ::com(f) [open $::status(port) r+]} err]} {
    log $err
    set ::com(f) ""
    after 10000 tryopen
    return
  }
  log "Configuring serial port\n"
  fconfigure $::com(f) -blocking 0 -buffering none -translation binary 
  fileevent $::com(f) readable readable
  log "Sending command M to sync on the response\n"
  puts -nonewline $::com(f) "M"
}

proc init {} {
  buttonstate disabled
  set ::com(await) "> "
  set ::com(awaitcb) mcusr
}

proc glitched {msg} {
  log "\n$msg" 
  set ::status(status) "Glitched"
  init
}

init

proc readable {} {
  set c ""; catch {set c [read $::com(f) 1]} err
  log $c
  if {$c eq ""} {
    glitched "$err [fconfigure $::com(f) -lasterror]"
    close $::com(f)
    tryopen
    return
  }
  if {![string length $::com(await)]} {
    if {$::status(status) eq "Connected"} {
      glitched "Unawaited char '$c', resyncing ..."
    }
    init
    if {$c ne [string index $::com(await) 0]} return
  }
  # TODO: Handle timeouts
  set await [string index $::com(await) 0]
  if {$await eq "*"} {
    set next [string index $::com(await) 1]
    if {$c eq $next} {
      set ::com(await) [string range $::com(await) 1 end]
    } else {
      set ::com(await) "$c$::com(await)"
    }
    set await $c
  } elseif {$await eq "$"} {
    if {[string is xdigit $c]} {
      append ::com(awaitdata) $c
      set c $await
    } else {
      set await ""
    }
  }
  if {$c eq $await} {
    set ::com(await) [string range $::com(await) 1 end]
    if {$::com(await) eq ""} {
      # log "[esc $::com(awaitcb)]\n"
      eval $::com(awaitcb)
    }
  } else {
    if {$::status(status) eq "Connected"} {
      glitched "Unexpected character '$c'"
    } else {
      init
    }
  }
}

proc mcusrtxt {hex} {
  scan $hex "%2x" val
  set i 0
  foreach bit $::mcusrbits {
    if {$val & 1<<$i} {
      set bit [string toupper $bit]
    }
    lappend mclist $bit
    incr i
  }
  return "$hex ([join [lreverse $mclist] { }])"
}

proc mcusr {} {
  log "Sending command M\n"
  if {[catch {puts -nonewline $::com(f) "M"} err]} {
    glitched $err
  }
  set ::com(await) "M\nMCUSR: $$\n> "
  set ::com(awaitdata) ""
  set ::com(awaitcb) {
    set ::status(status) "Connected"
    set ::status(mcusr) [mcusrtxt $::com(awaitdata)]
    buttonstate normal
  }
}

proc docmd {cmd} {
  if {[catch {puts -nonewline $::com(f) $cmd} err]} {
    glitched $err
  }
  log "Sending command $cmd\n"
  set ::com(await) "$cmd\n"
  set ::com(awaitcb) "glitched \"Command $cmd triggered, awaiting resync ...\""
}

proc doread {} {
  if {[catch {puts -nonewline $::com(f) "RE"} err]} {
    glitched $err
  }
  log "Sending command RE\n"
  set ::ee(data) ""
  set ::com(await) "RE\n"
  set ::com(awaitcb) hexstart
}

proc hexstart {} {
  set ::com(await) "*\n:$$$$$$$$"
  set ::com(awaitdata) ""
  set ::com(awaitcb) hexhead
}

proc hexhead {} {
  set ::ee(head) $::com(awaitdata)
  scan $::ee(head) %2x%4x%2x ::ee(size) ::ee(addr) ::ee(type)
  set ::com(await) [string repeat "$$" [expr {$::ee(size)+1}]]
  set ::com(awaitdata) ""
  set ::com(awaitcb) hexdata
}

set ff [binary format H* FF]
proc hexdata {} {
  set line "$::ee(head)$::com(awaitdata)"
  set sum 0
  set vallist [scan $line [string repeat "%2x" [expr {[string length $line]/2}]]]
  foreach val $vallist {incr sum $val}
  if {$sum%256!=0} {
    glitched "Invalid checksum [format %04X $sum] in $line"
    return
  }
  set len $::ee(size)
  if {$::ee(type)==0} {
    set data [string range $::com(awaitdata) 0 end-2]
    set append [expr {$::ee(addr)+$len-[string length $::ee(data)]/2}]
    append ::ee(data) [string repeat "ff" $append]
    set ::ee(data) [string replace $::ee(data) [expr {$::ee(addr)*2}] [expr {($::ee(addr)+$len)*2-1}] $data]
    hexstart
  } elseif {$::ee(type)==1} {
    if {$len || $::ee(addr)} {
      glitched "Invalid end marker $line"
      return
    }
    # log "eeprom $::ee(data)\n"
    set ::com(await) "*\n\n> "
    set ::com(awaitcb) filldata
  } else {
    glitched "Invalid type in $line"
    return
  }
}

proc filldata {} {
  set addr 0x0000
  foreach {text len} $::eeprom {
    if {$text ne ""} {
      set end [expr {$len?($addr+$len)*2-1:"end"}]
      if {$len} {
        set end [expr {($addr+$len)*2-1}]
        set data [string range $::ee(data) [expr {$addr*2}] $end]
      } else {
        set data [string range $::ee(data) [expr {$addr*2}] end]
        set data [binary format H* $data]
        set end [string first [binary format H* 00] $data]
        if {$end>=0} {
          set data [string range $data 0 $end-1]
        }
      }
      set ::data($text) $data
    }
    incr addr $len
  }
}

proc dowrite {} {
  log "Sending command WE\n"
  if {[catch {puts -nonewline $::com(f) "WE"} err]} {
    glitched $err
  }
  set ::com(await) "WE\n"
  set ::com(awaitlist) ""
  set ::com(awaitcb) programline
  set addr 0x0000
  foreach {text len} $::eeprom {
    if {$text ne ""} {
      set data $::data($text)
      if {$len} {
        set data [string range $data 0 [expr {$len*2}]]
      } else {
        binary scan $data H* data
        append data "00"
      }
      set head [format "%02X%04X00" [expr {[string length $data]/2}] $addr]
      set line "$head$data"
      set vallist [scan $line [string repeat "%2x" [expr {[string length $line]/2}]]]
      set cksum 0
      foreach val $vallist {incr cksum $val}
      set cksum [expr {0x100-($cksum&0xFF)}]
      #log "line $line vallist $vallist cksum $cksum"
      set line ":$line[format %02X $cksum]"
      if {[llength $::com(awaitlist)]} {
        set line "\n$line"
      }
      for {set i 0} {$i<[string length $line]} {incr i 4} {
        lappend ::com(awaitlist) [string range $line $i $i+3]
      }
    }
    incr addr $len
  }
  lappend ::com(awaitlist) "\n:00" "0000" "01FF"
  # log "[esc [list awaitlist $::com(awaitlist)]]\n"
}

proc programline {} {
  if {[llength $::com(awaitlist)]} {
    set ::com(await) [lindex $::com(awaitlist) 0]
    set ::com(awaitlist) [lrange $::com(awaitlist) 1 end]
    after 20
    log "\nSending [esc $::com(await)]\n"
    if {[catch {puts -nonewline $::com(f) $::com(await)} err]} {
      glitched $err
    }
  } else {
    set ::com(await) "OK\n> "
    set ::com(awaitcb) ""
  }
}



  
