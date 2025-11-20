# computer module carrier board design
## decide which rules be violated for which signals in what area and when
    trace impedance matching
    length matching
### high speed preferably direct ref to gnd layer
## 4-layer
    stitching capacitor -> pwr gnd layer2 3
        referenced by highg speed signals layer1 4
## 6-layer
    low speed signals in layer3 4
        - ref pwr gnd layer2 5 -> stitching cap pwr gnd
        - ref further -> trace wider -> common impedance 50ohm
## 8-layer
    - pwr gnd layer4 & 5
    - high speed signals layer1 3 6 8
        - signal w/ two adjacent gnd layer6 -> most critical impedance/less emc problem
        - signal layer3 -> stitiching cap pwr gnd
## single-ended trace impedence
        trace gnd impedence
    parallel RGB led, camera interface
## differential trace impedence
    - two signal traces of a pair impedence, priority
        + gnd single-ended impedence
    - Zsignal < Zsingle-ended
    - PCIe, SATA, USB, HDMI
## calculated impedance 
    - close to exact val/tolerance band mid -> max production yield
        cals tools
    - microstrip: top bottom layer traces
    - stripline: inner layer traces 
        two ref -> reduce electromagnetic emission
        embed in prepreg
    - 4-layer
        single-ended 50ohm+-15%
        differential 90
## component placement
    - signal return paths 
        -> avoid ref plane splitting
    - signal allow only certain amt of vias
        -> layer change low
    - avoid crossing high-speed signals
        -> interface chip bottom layer
        -> polarity reversal supported by interface/PCIe
        -> lane reversal if applicable
    - more spacing for high-speed signals
            w/ analog signal
        need ref plane -> suboptimal
            -> iterations placement/routing
## pwr supply
    - non-continuous current
    - high freq -> large peak current -> long trace
        -> noise emission -> other signals/circuits
        -> cant carry peak -> voltage drop
            -> bypass decoupling cap for every pwr -> energy reservoir
                - first cap, then supply pin
                - small cap -> speed, closer to supply 100nF
                - big cap -> energy 10uF

    - wider, as wide as possible
    - shorter
    - one via for one amp current
    - ground return current
        gnd vias same as supply
    - switch too fast -> current peak for caps high 
        -> EMI
        -> overcurrent
            -> limit switching speed 
                - 10nF cap 47k R
                - 100nF bypass cap close to sw transistor
    - trace copper thickness 
            1/2oz/sqf 17um
        resistance 1mohm per sq
            100um width 100mm length 1ohm
    - outer -> via plate -> thicker
            1oz/sqf 35um
        resistance less 0.5mohm per sq
            +vias resistance
    - signal vias -> void in pwr gnd -> current density increase
        -> avoid hot spots
            place vias in a grid w/ space in between 
                for pwr plane/src<->sink to pass
## bends
    - high speed -> minimize bends 135
    - serpentine -> for matching length
        adjacent distance > 4 trace width
        each seg/bend length > 1.5 trace width
## signal
    - crosstalk/noise to nearby due to mutual cap/inductance
        -> min distance 3 trace width
        -> differential pair min 30 mil for 6-mil
    - bottolenecks -> force closer -> min area enlarge distance outside
    - increase spacing whenever possible
## trace stubs
    - pull up/down resistor on high-speed signal
        -> long -> antenna > 1/10 wavelength v/f 4ghz -> 3.5mm max
        -> reflection/bounce back impact signal integrity
            -> daisy chain
    - vias 6-layer signal layer 1->3 via -> layer 6
        -> reduce via no.
## large pad underneath: gnd plane
    impedence discontinuity -> reflection
        -> large pad layer1 -> lower impedance than trace
            -> plane abstruct layer2/ref to layer1 under large connector/component pads
            -> active ref plane for pad/layer1 on another layer3, layer3 also ref to other signals/layer4
                -> stitch vias to normal ref plane/layer2
        -> vias
            -> rm unused in inner layer
## differential pair signal
    - constant distance -> impedence
        -> symmetrically parallel
            minimize pad entries enlarged spacing area
    - not in between -> impedence discontinuity emc
    - serial coupling cap -> discontinuity w/ pad
        -> symmetrically
        -> 0402 sized, not larger pkg 0805
    - vias
        -> reduce vias
        -> symmetrically
        -> same amount of vias to both traces on the same layer
## length matchign
    - time of arrival between < skew/oblique not exceeded max
        data packet arrive within a period
        -> propagation speed: light speed/sqrt(permittivity 1/4.5)
            inner/fr-4 layer slightly faster than outer/air/solder mask
            half the speed of light 150 um/ps
    - differential pair tight delay skew + -
        length diff -> serpentine origin of the length
            - pos neg propagated synchronously over the major conn part
            - close to bend max 15mm
            - two bends compensate each other
            - compensate in each seg/via/connector/coupling cap separated
            - same layer: not equal for diff layers
            - trace length on pad/via
            - symmetric preferably than serpentine
            - small loop preferably than serpentine
## signal return path
    - incorrect path -> noise coupling/emi
    - pwr/low-speed -> shortest
        high-speed -> follow signal trace back
    - not over a split plane
        -> src sink separated area -> loop antenna
        -> route around it     
    - two plane -> stitching cap between ref planes
        cap -> allow travel diff ref 
            -> close to signal path -> forward return distance small
            -> 10nF 100nF
    - avoid plane abstruction/stitching cap
    - via close -> void    
        -> adequate separatation
        -> place fewer gnd/pwr vias
    - signal src sink 
        -> one gnd via -> return current cant travel back over ref
            -> return path only top layer
                -> impedence calcd as ref to gnd, not top layer
                    -> place gnd at src sink
    - pwr ref
        -> stitching cap gnd->vcc
            - close to signal sink src entry/exit
            - 10nF 100nF
    - switch layer/via
        -> stitching cap close to change via
            - diff signal -> symmetrically
    - a diff net as ref/gnd->pwr
        another cap -> allow current gnd -> pwr                    
    - avoid edge/border -> affect impedance
## analogue digital gnd
    - capacitive inductive coupling -> noise
        -> separate signals from each other
            - increase space between signals
            - keep analog away from clk/high-current switching/power supply
    - conductive coupling 
        -> current spikes of digital coupled over resitance/inductance to analog
            -> separate digital analog return path
    - physical split plane
        e.g. anolog-to-digital converter adc
        - two diff gnd planes as ref -> separate placemnt
        - ferrite beads/0 ohm resistors between two nets
            merging gnd close to circuit
        - no route over another/complete separated
    - virtual split
        - help line/dotted line on unused mechanical layer
            -> correct side placement
            -> respect the line during routing
## interface
    - differential pair length
    - clk signal pair skew
        recover clk out of data signal -> matching 240mm
            -> dont overmatch -> signal quality issue
        no clk signal in lCD, external clk parallel buses/synchronous 
        -> tight length matching clk data 0.5mm
    - vias no. low
## PCI express
    - multi lane -> polarity reversion    
## SATA
## ethernet
    MDI signal -> anolog diff pair 
        -> separate from digital
    magnetic instead of jack
        -> require high voltage, isolate from others
            -> close to jack 
            -> min 2mm from other signal/plane
            -> dedicated gnd plane
            -> separate shield plane for LAN device
                -> digital gnd + magnetics gnd + shield gnd
## usb
    - 3 bidrectional data pairs
        - two high speed 5Gbit/s
        - polarity inversion
            -> not swap receiving signals w/ transmitting ones
## parallel rgb lcd
    - pixel clk -> display resolution
        -> lower VGA 640x480 -> relaxed requirement
## lcd
    no clk in data signal -> length matching essential
## hdmi/dvi
    long -> certain skew <5ps 750um/normal 1ps 150um
## analog vga
## parallel camera
    - pixel clk -> display resolutin
    - skew < 100ps
    - from timing perspective, trace length not limited/delay negligible
        not like high-speed require length matching
## sd/mmc/sdio    
## i2c
    - not diff pair, not separate data clk too much
    - capacity load/attched bus divice -> trace length limited 
        -> sub length not problem -> no daisy chain
            -> star topology -> keep trace short
## display serial 
## camera serial mipi/dsi d-phy

## section 0