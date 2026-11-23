# check_hw.tcl  (Flexlink_py/HW_Development/Packet_Detector_FCOMM/FW)
#
# Hardware check for the step-2 bitstream (capture only), over JTAG, with no firmware:
#   1. program the bitstream, run ps7_init
#   2. axi_ad9361 (0x79020000) and axi_dmac (0x7C400000): version, ID, scratch write/read
#   3. AD9361 out of reset (EMIO GPIO 100), then read its product ID over PS SPI0 (reg 0x037)
#   4. axi_ad9361 clock status (only informative: the AD9361 sends no DATA_CLK until the
#      firmware configures it in step 3, so "no clock" is expected here)
#
# Board: VADJ jumper J18 at 2.5 V, FMCOMMS3 on FMC LPC, boot jumpers JTAG, USB PROG (J17).
#
#   cd Flexlink_py/HW_Development/Packet_Detector_FCOMM/FW
#   xsdb check_hw.tcl

set here  [file dirname [file normalize [info script]]]
set hw    [file normalize [file join $here .. HW build_fmcomms3 fmc_zed]]
set bit   [file join $hw fmc_zed.runs impl_1 system_wrapper.bit]
set inits [glob -nocomplain [file join $hw fmc_zed.gen sources_1 bd system ip * ps7_init.tcl]]
if {![file exists $bit]} { error "bitstream not found: $bit (run HW/build_hw_fmcomms3.tcl)" }
if {[llength $inits] != 1} { error "ps7_init.tcl not found (or not unique) under $hw" }

set fails 0
proc check {what ok detail} {
    global fails
    puts [format "  %-4s %-44s %s" [expr {$ok ? "OK" : "FAIL"}] $what $detail]
    if {!$ok} { incr fails }
}
# -force: xsdb otherwise refuses PL addresses ("PL AXI slave ports access is not allowed")
proc rd {a}   { return [mrd -force -value $a] }
proc wr {a v} { mwr -force $a $v }
proc hex {v}  { return [format 0x%08X $v] }

# --------------------------------------------------------------------------
connect
targets -set -nocase -filter {name =~ "arm*#0"}
rst -system
after 500
puts "Programming FPGA: $bit"
fpga -file $bit
targets -set -nocase -filter {name =~ "arm*#0"}
source [lindex $inits 0]
ps7_init
ps7_post_config
# xsdb blocks PL addresses unless they are in its memory map (normally taken from an ELF or
# the hardware design); add the two register windows by hand (flags 3 = read + write)
memmap -addr 0x79020000 -size 0x10000 -flags 3
memmap -addr 0x7C400000 -size 0x1000  -flags 3
puts ""

# --------------------------------------------------------------------------
puts "AXI registers"
set AD  0x79020000
set DMA 0x7C400000
foreach {name base} [list axi_ad9361 $AD axi_dmac $DMA] {
    set ver [rd $base]
    check "$name version (+0x000)" [expr {$ver != 0 && $ver != 0xFFFFFFFF}] \
        [format "%s = v%d.%02d.%s" [hex $ver] [expr {$ver >> 16}] [expr {($ver >> 8) & 0xFF}] \
             [format %c [expr {$ver & 0xFF}]]]
    wr [expr {$base + 8}] 0x5A5AA5A5
    set s1 [rd [expr {$base + 8}]]
    wr [expr {$base + 8}] 0x12345678
    set s2 [rd [expr {$base + 8}]]
    check "$name scratch (+0x008)" [expr {$s1 == 0x5A5AA5A5 && $s2 == 0x12345678}] "[hex $s1] [hex $s2]"
}
set ident [rd [expr {$DMA + 0x0C}]]
check "axi_dmac identification (+0x00C)" [expr {$ident == 0x444D4143}] "[hex $ident] (expect 'DMAC')"

# --------------------------------------------------------------------------
puts "AD9361 reset and SPI"
# EMIO GPIO bit 46 = Zynq GPIO 100 = bank 3 bit 14 (resetb, active low)
set GPIO_DATA3 0xE000A04C
set GPIO_DIRM3 0xE000A2C4
set GPIO_OEN3  0xE000A2C8
set rb [expr {1 << 14}]
wr $GPIO_DIRM3 [expr {[rd $GPIO_DIRM3] | $rb}]
wr $GPIO_OEN3  [expr {[rd $GPIO_OEN3]  | $rb}]
wr $GPIO_DATA3 [expr {[rd $GPIO_DATA3] & ~$rb}]
after 10
wr $GPIO_DATA3 [expr {[rd $GPIO_DATA3] | $rb}]
after 20

# PS SPI0 (0xE0006000): master, SPI mode 1 (CPOL 0, CPHA 1) as no-OS uses, clock /256
# (about 650 kHz), manual chip select, automatic start
set SPI 0xE0006000
set CR_IDLE   0x00027C3D   ;# CS lines all high
set CR_ASSERT 0x0002783D   ;# CS0 low
proc spi_read {reg} {
    global SPI CR_IDLE CR_ASSERT
    wr [expr {$SPI + 0x14}] 0                     ;# disable
    wr $SPI $CR_IDLE
    wr [expr {$SPI + 0x14}] 1                     ;# enable
    for {set i 0} {$i < 16 && ([rd [expr {$SPI + 4}]] & 0x10)} {incr i} { rd [expr {$SPI + 0x20}] }
    wr $SPI $CR_ASSERT
    # 16-bit instruction: bit 15 = 0 (read), bits 14:12 = 0 (one byte), bits 9:0 = address
    wr [expr {$SPI + 0x1C}] [expr {($reg >> 8) & 0x03}]
    wr [expr {$SPI + 0x1C}] [expr {$reg & 0xFF}]
    wr [expr {$SPI + 0x1C}] 0
    after 5
    set b {}
    for {set i 0} {$i < 3} {incr i} { lappend b [expr {[rd [expr {$SPI + 0x20}]] & 0xFF}] }
    wr $SPI $CR_IDLE
    wr [expr {$SPI + 0x14}] 0
    return [lindex $b 2]
}
set pid [spi_read 0x037]
check "AD9361 product ID (SPI reg 0x037)" [expr {($pid & 0xF8) == 0x08}] \
    [format "0x%02X (expect 0x08 in bits 7:3, revision in 2:0)" $pid]

# --------------------------------------------------------------------------
puts "axi_ad9361 clock (informative before step 3)"
wr [expr {$AD + 0x40}] 0x3                        ;# REG_RSTN: release ADC core resets
after 50
set st  [rd [expr {$AD + 0x5C}]]
set frq [rd [expr {$AD + 0x54}]]
puts [format "       status 0x5C = %s, clk_freq 0x54 = %s (about %.1f MHz)" [hex $st] [hex $frq] \
          [expr {$frq * 100.0 / 65536}]]
puts "       0 / no clock is expected: the AD9361 drives DATA_CLK only once configured"

puts ""
if {$fails == 0} { puts "All checks passed" } else { puts "$fails check(s) failed" }
disconnect
