# openpilot panda opendbc notes

## DBC/CAN database container
    BO_ 200 SENSOR_SONARS: 8 SENSOR
        object id name: length sender
    SG_ SENSOR_SONARS_mux M :
        24|12@1+ (0.01,-20.48) [-20.48|20.47] "" DRIVER,IO
            start|size@little_endian+/unsigned (scale/fraction, offset) [min|max] unit debugger
            min/max: 64-bit total, 12-bit, 0x1000 4096*0.01 - 20.48(40.96/2) = 20.47
        56|8@1+ (1,0) [0|255] ""  aps
    BA_ "FieldType" SG__500 IO_DEBUG_test_enum "IO_DEBUG_test_enum";
    multiplex 64-bit total:
        2-bit 2^3=8 64-2=62 62*8=248 bytes
            8-bit 2^8=256 64-8=56 56*256=1792 bytes
        M m0/m1: two separate msgs for m0 and m1
    NS_
        new symbols
    BA_DEF
        attribute definition
    BA_DEF_DEF_
        attribute default value
    BO_
        bus object
    BS_
        bus speed/baudrate settings
    BU_
        bus unit/network nodes
    CAT_
        category definition
    CM_
        comment
    EV_
        env var
    REL_
        relation/node relation
    SG_
        signal
    agc -> auto generated code
        mia -> missing in action handling -> default temp=68f
    typedef struct {
        int8_t MOTOR_CMD_steer;
        uint8_t MOTOR_CMD_drive;
    } MOTOR_CMD_t;
        BO_ 101 MOTOR_CMD: 1 DRIVER
            SG_ MOTOR_CMD_steer : 0|4@1- (1, -5) [-5|5] "" MOTOR
            SG_ MOTOR_CMD_drive : 4|4@1- (1, 0) [0|9] "" MOTOR
    100hz -> 100 cycles/sec 1s=1000ms -> one cycle 1000/100=10ms

    big-endian 0x12 0x34 -> 0x1234
        little-endian -> 0x3412
    05 96 E7 54 6D 58 00 6F
        05 (5, 0) scale by 5 5*5=25A current_limit
        96 E7 (0.25, 500) _6E7 hex6E7=1767 *0.25+500=58.25A battery_current
        6D 58 (0,0025, 0) hex6D58=27992 *0.0025=68.98% soc state_of_charge

## opendbc
    database
        dbc format -> parse -> py
        steering
    mull: test mutation/programming logic
    dbc
        BU_: CH DIPF DIPR ETH FC HVI HVS PARTY SDCV VEH VIRT
            BO_ 905 DAS_status2: 8 PARTY
            SG_ DAS_status2Checksum : 56|8@1+ (1,0) [0|255] ""  aps
        party: ecu, sender
            das: driver assistance sys
            aps: accelerator pedal sensor
            esp: electronic stability programme
            gtw: gateway
    d = (dat[i] >> (lsb - (i * 8))) & ((1 << size) - 1)
        i=0, lsb-0=lsb, move lsb bits, 1<<2 2^2 0xFF, keep only the last two bit
        i=1, lsb-8, shift lsb-offset(align lsb to bit0) higher bits
        -> extract sig val from CAN msg
    @dataclass

## panda flow
### board/main.c
    irq init disable
    clk
    init peripheral
    board type
    init
        led for debug
        adc analog/voltage/current-to-digital converter
            adc peripheral
        can mode normal
        harness
        fpu floating point unit/operation
        timer
        fan
            test power->rpm
            tuning: overshoot
                stop completely: sleep(0.1)
                30% power/onroad -> max rpm power
                max / expected from panda
        safety mode silent
            clear msg hardware queue in can core/unplugged
            set obd diagnostics mode when param 0ms
        can tx transceiver
        heartbeat loop reset/feed watchdog 375ms/avoid hanging
        fault tick/timer 8hz irq 10ms
            siren pulsing
                countdown
        usb, spi, irq
        led blink/fade increase, ON longer, OFF shorter, pulse smoothly
            pattern breaks -> sys overload
    wfi wait for irq, lowe-power sleep till irq
    scb_scr sys control block/register
        cpu sleep, not peripheral

### board/body/main.c
### board/body/can.h
    llcan low-level can
        llcan_init() reinit CAN at hardware level
    ring uart circular buffer
        data: uart inp
        loop: read data/char, inject back into ring
    acc: adaptive cruise control
    ul: unsigned long literal/32-bit
    u: unsigned integer literal suffix
    cpacr coprocessor access control register
        enable/disable 3 full access
        set bits[21:20] 3UL << (10U*2U)
    CAN periodic
        can command -> motor set target rpm left right
        if now - last >= period, send speed
    CAN init
        set gpio output
        safety hook
    CAN pkt
        bus-addr-returned
        rejected/extend/fd
        data_len
        data[0]...[7]
        checksum

### board/body/motor_controller.h
    inline funcs: speed_update()/write_pwm()
    absf: float
    control +=
        +kff feed-forward gain * target_rpm
        +kp proportional * error
        +ki integral
        +kd derivative
    motor write
        cfg config forward_timer forward_channel reverse_timer
        set gpio output
            enable port/pin

### board/body/flash.h
    scons .bin.signed
    parse arg
    get_usb_serial

### py __init__.py
    interact w/ openpilot
    can
        congestion -> NAK
        reset comm
            controlWrite(type, req, index, data) abstractmethod
        send many
            pack buffer/header, len, bus, addr, extended -> append arr
            loop bulkwWrite tx
        receive
            bytearray bulkRead
            unpack
        clear
    version
    init
        dfupdate
        connect to serial
            usb fst, spi snd
            usb: serialnum, in usb_list
            spi: check bootstub pid
    reset
    reconn
    flash
        choose sectors accumulate enum
        unlock
        erase
        flash via ep2/bulkWrite
        reset controlWrite()
    recover
        dfu bootsub/program
    dfu timeout wait
        monotonic() not going back
        list
    wait for panda
        serial list
    signature up to date
    call/write control/handler api
    health
        struct
        uptime, voltage, current, buf overflow, ignition, harness, safety
    can health
        bus_off, error, last_error, total_rx tx lost cnt, speed, irq, core rst cnt
    self._handle.controlread rqt
        uid, secret, irq rate
    config
        alter, power, loopback, obd, ena, speed, uart baud/parity equal/callback
    serial
        read, write
        hearbeat send/disable
    timer
    ir power
    fan power set/get rpm unpack
    siren
    debug
        arr timer period
        ccr channel pulse len
        force relay
        read gpio

### debug
    openocd  open on-chip debugger
       .cfg stlink interface config
       swd serial wire debug protocol transport
       -f load stm32h7 config
       init

### dfu device firware update
    bootstub/recover
        clear/erase
        program() write mcu_type.config.addr, code
    usb/spi connect/list


### drivers/usb.h
    tx comm
    enter critical
    EP3 out endpoint3 not processing
    EP(3U)->DOEPCTL access out endpoint 3 registers -> control
    OTG->NAKSTS usb on-the-go/phone to usb directly -> NAK/refusing data status bit
    set SIZ size 32UL<<19 32 pkts(shift to bit 19 pos) of 0x800U 2048 bytes transfer size
    ENA enable to accept data | CNAK clear NAK/refusing data bit
    exit critical
    usb port
    irq handler
        gintsts global irq status reg
            flag & usbrst reset, mmism mode mismatch, enumdne enum done
        gccfg global core config reg
            power-down mode, vbus sensing
        cidschg id status change
        rxflvl receive FIFO level
            rxstsp pktsts read rx/pkt status
        updt update
            DATA, SETUP
            EP endpoint, BCNT len
        gnak global negtive ack
            in, out
            dctl device control
        endpoint
            in, out
        clear handled irq
    usb set_up
        hex 0x0040U
        device desc/version/len, type
        config
            len, type, index, class sub, endpoint, dir, max pkt, polling interval
        str language
        driver
        binary obj store
        enable other endpoints
            config, addr, req
    reset
        unmask endpoint irq
            DAINT dataline irq
            DAINTMSK
        clear irq
            DIEPINT, DOEPINT = 0xFF
        DCFG device config reg
            unset addr
        GRXFSIZ FIFO setup
        FLSH flush fifo tx rx
        no global NAK dctl
        ready to receive pkt DOEPTSIZ
    write pkt
        EP0 DIEPTSIZ DFIFO
    read pkt
        dest_copy++ dest
        FIFO

### drivers/spi.h
    spi transfer msg struct
        header: endpoint, tx_len, rx_len, sync
        len
        tx_buf
        rx_buf
        speed_hz
    init &msg msg val/add tail
    memcpy header -> tx_buffer
    sync
    ack
        sync read spidev_data len
        == -> ret 0
        else -1
        >20 -> sleep
    response: rx_buf rx_len
    checksum
        ^= buf[0U, len++] xor accum
    retval copy -> user
    return rx_len

### stm32h7 config
    critical/faults/util
    driver registers/irqs/uart
    gpio/peripheral/irq/timers
    clock/usb/spi
    gpio:
        moder: mode register
            config pin mode/00 inp/01 general out/10 alternate/11 analog
        odr: out data register, out high/low
        pupdr: pull-up/down register internal resistor

### examples/tester.py
    adapter
        powertrain bus0
        body bus1
        chassis bus2
    safety alloutput/careful!
    0x248 mcu cmd msg
        0x01 lock/pop frunk
        0x04 open trunk
    can send msg
    safety silent
    read vin(index+str)
        bytes -> hex str -> int
        [:2] first 2, [2:] the rest

### examples/bit_transition.py
    msg low high range, start end
    transition low to high: parse, sort, diff
        after ':' hex->int
    csv file -> msg collection dict
        msg id: [2:] int
        bytearray data
            ones
            ~ invert -> zeros
    transition
       zero_to_one bitmask: other.zeros[i] & self.ones[i]
       one_to_zero

### examples/query_fw_versions.py
    parser arg
        addr
        sub_addr
            'scan' 0x0-0xff
        uds unified diagnostic services data id: std_id
        serial panda
    query tqdm progress bar library/indicator
        bus
        sub_addrs
        description str to addr
        -> uds client tester session control
        id -> data -> resp{}, (addr, sub_addr) -> res[]

## math
### time quanta/split 8 for 1mbp
    1 megabits per sec when prescaler*8/clk = 1us
    bit time on CAN bus divided into series of quanta
     ~~quanta = (prescaler + 1)/clk~~
    prescaler/baudrate prescaler brp
        = clk*10/(quanta*bit_rate/can_speed)
        42mhz = 42e6/(8*1m) = 5.25 -> 5
        48mhz = 48e6/(8*1m) = 6
        sample ptr 87.5% seq1 quanta 6 -> desired_ts1
                    12.5% seq2 quanta 1 -> desired_ts2
                6 + 1 + 1 = 8
    btr/bit time register /+1: hardware
        ~~= (1+ts1+1+ts2+1)*(prescaler+1)/clk~~
            #define CAN_BTR_TS1_0 0x00010000~~U~~ bit16 [19:16]
                *ts1/<<16
            #define CAN_BTR_TS2_0 0x00100000~~U~~ bit20 [22:20]
                *ts2/<<20
            prescaler 42mhz 5-1=4 [9:0] +1=5
        = (seq1-1)*ts1 | (seq2-1)*ts2 | prescaler-1
        = 5*0x00010000 = 1 left shift 5 times = bit[18:16] 101~~b~~ = 5+1=6
            | 0*0x00100000 = bit[22:20] 0+1=1
        = 0x001B0004_42mhz/0x001B0005_48mhz

### bus0=can1 bus1=can2 bus2=can3 bus3/gmlan
    can_speed[] = {5000, 5000, 5000, 333}
    logical bus bus_lookup[physical] = logical
        bus_lookup[] = {0,1,2}
            bus_lookup[1/2] = 3: can1/2 map to bus3
    physical can: can_num_lookup[logic] = physical
        can_num_lookup[] = {0,1,2,-1}
        *cans[] = {CAN1, CAN2, CAN3}
            can_num_lookup[1/2] = -1: invalid, bus1/2
            can_num_lookup[3] = 1/2: gmlan take over can3
                bus3 for can1/2

## capnp
### serialize, like json, fast obj
    aligned buf
        words_size calc size/word
            min 512 words
        copy data into buf
        ret arrptr to buf
        inline extract data from obj
        private storage persist across calls
### pubmaster wrapper
        serialized msg -> socket based on name id
    submaster
        upd, alive, valid
        frame, time
        msgs, service_list

## cereal
### msg builder
        heaparray ptr msg/*this obj->arr->bytes
        serialize obj *this/binary to buf for storage/transmission
            size <= buf
            arr out stream wrapping buf
            writemsg/serialize msg
        event init, time, valid

## msgq
### light weight msg queue
    bool num reader > 0
        updated/all read valid/equal/read up to cur write ptr pos
### poll
        periodically check for new msgs
            long/wait time
        msg ready/read ptr = write
            init sub
            if num exceeds, =0 reset/no inactive ones/valids[i]=false
            thread signal/kill tid/read->old uid
                macos no sys_tkill->rely on polling
            atomic compare-swap *__o == *__e -> *__o = __d, true
                else set expected/*__e = *__o/update, false
                two sub start at the same time    
                lock-free sync/concurrent
                    thread safe wrapper/no thread blocking
                    cpu level/low-level primitives
                read sync to write pos ,reader valid
                    start with false, first read sync
                reader_id, uid, reader_ptr
            reset
            read/write
                cycles, pointer
        num++
        timeout ms
            macos poll more frequently/min 10
                 <- signals cant irq nanosleep
        timespec nanosleep
            poll_ms -> ts
            sleep for duration 
            check msg ready
                cnt+=1
                repeat till msg ready
            avoid busy-waiting
        apple: exit if timeout remaining_ms <=0
        ret num
### msg recv
        reader_id/sub initialized
        uid local == uids[id]
            otherwise init sub
        goto: transfer control to loc/label
        valid
        read/write higher/cycles lower/ptr, inp/read_ptr[id]
            u32 <- unpack/combine 64
            save space/efficient transfering data
        *ptr = data + read_ptr
        msg avail
            read == write ptr, synced/no new msg/msg->size =0
            else new msg
        msg size valid
        size = -1, buf full
            circular buff wrap around/back to start
            cycles++
            -> pack64 atomically read_ptr/cycles
        assert size >0, max
            crash better than pass garbage data to consumer
        align new ptr += size
        conflate: skip bufed msg, only the latest
            check its latest/read=write ptr
            else theres new msg 
                pack64/upd ptr, goto start to grab the latest
        init size/structure
            alloc buf for size/metadata
            <0 -1/alloc failure
        sync
        memcpy data <- queue buf p/data+ptr + size + header/8
        upd ptr
        valid
### msg send
        uid not local -> kill pub/q->endpoint
        total size: align/round up to cache line/boundary detection
             size+header/8
        assert 3 * size 
            at least 3 msg in the queue
            to fetch latest/last msg safely
        unpack64 write cycles/ptr, queue_ptr
        base offset: data + write_ptr
        remaining space for wraparound tag for next msg/alignment
            <=0: p=-1
            invalid all readers > write ptr, cycle read != write
            upd global/local write ptr pack64/cycle+1
            ptr -> data seg beginning
        invalid readers in write area
            start w_ptr, end align start+size+header
            num loop
                unpack64 read
                invalid start-end
        write *size_p msg->size
        memcpy p+header <- msg->data
        upd ptr pack
        thread_signal: notify reader
        ret msg->size
    init pub
        uid num_reader
        invalid, uid=0
        write uid local <- uid
### new queue
        buf smaller than 2^32
        signal usr2 handler callback async <- thread_signal()
        path apple/linux
            prefix + '/'
            file fd path rw
            rc/return code from ftruncate size+header
                0 sucess, <0 err
        prealloc env mmap flag/populate
        nmap_p = mem <- mmap rw flag fd
        *header <- *mem
        ptrs to header
            num_readers/atomic ullong, w_ptr, w_uid <- (header->ptrs)
        num loop
            r_ptr, r_valids, r_uid <- header
        q->data, size, id, ep/path, confalte false
    close queue
        != Null: munmap
    reset store atomically
    msg close
        delete msg->data
        size=0
        ret 0
### setup.sh -> test.sh lefthook run test/lint+test
        install dependencies
            Darwin install zeromq venv libzmq3 opencl
        catch2 cpp unit test -> /tmp/catch2
        uv venv
### scons/pkg dir
        visionipc interprocess communication/sync, alloc by kernel
            shared mem by process a & b
            msg passing
            build
            lib
        vipc_obj <- shared obj .so file
        frameworks [] /libs
            opencl
        extra: test
### unit test catch2 "[integration]"
        1 pub 2 subs
            cleanup test queue
            new writer -> init pub
            new reader1 reader2 queues -> init sub
            send msg ==> recv msg1 msg2
            REQUIRE() size header, data i
            close msgs
        1 pub 1 slow sub
            1e5 100,000 buf size 100kb
            1024*3=3kb
            REQUIRE() % 10 6:1 ratio
                recv 8572 6 size !=0
                skip 1428 1 size 0
        2 pub 2 sub conflate
            msg_size 128
            msg1
                REQUIRE == msg_size
                memcmp(in, out, size) == 0
            msg2 == 0
        2 pub 2 sub conflate false
            msg2 == msg_size
        1 pub 1 sub
            msg2 == 0
        init sub init 2 sub
            REQUIRE num_readers 1 2
                reader_id 0 1
        invalidation
            sub.w_ptr 1<<32 2^32 cycle/high+buf/low
            REQUIRE valids[0] true
            r only at start
                r_ptr 0/tag/header/meta 64/data
                w_ptr 1<<32|1000/cycle 1, pos 1000 wrapped
                r_ptr 1020/cycle 0, hasnt wrapped yet
                    r in prev cycle vs w
            REQUIRE valid false
        wraparound
            init w_ptr>>32 == r_ptr>>32 == 0
            recv 
                only w, no r
                    msg.size == 0, needs to reset
                w r alternate in loop {}
                    >0 cnt while keeping up w/ w
                REQUIRE r_ptr[0]>>32 == 1
                    after 8 msgs, cycle cnt +1/wrapped once/high bits track cycle
            send
                w init 0
                REQUIRE 
                    8th msg written at the beginning/size+header
                    cycle cnt >>32 ==1
                    wraparound tag loc/data +=7*size == -1
### pytest sub socket
        recv timeout assert
            timeout monotonic - start < timeout
            recv not None
        conflate
            random socket, randinit->num_msgs, conflate true/false
            random bytes -> pub msgs, recv msg <- sub socket
            if conflate, assert len == 1, [0] == [-1]
            else, len == len(sent_msg), recv == sent
### visionipc interprocess communication
        w/ fds
            memset control_buf <- controlmsg_size*num_fds/file/socket descriptor
            struct
                iovector base/buf ptr, len/buf_size -> single buf
                msg header &iov/ptr to addr, len=1/1 entry
                    multiple bufs in one syscall
            control len
            send cmsg socket level/sol, type/socket ctl msg, len/size
                memcpy data <- fds, send
            else recv
                assert cmsg, level, fds>0, memcpy
                flags -> close(fds[i])
                out_num_fds = recv_fds
        bind
            unlink path
            bind(sock, addr), err 0, listen sock
        connect(sock, addr)
            err !=0 close ret -1
    impl
        poll evs: subsocket r [] <- push_back(sockets[i])
                num_polls
            recv
            connect addr new_queue
        zmq poll zero message queue
            num_polls within timeout ms
            ret num_socket, -1 err
                avoid busy_waiting/blocking until open
            ep tcp://* += port
    event
        socket ev handle 
            ena, recv, fake ev/prefix
        ev set, clear, peek, valid, fd, 
            wait pollfd, timeout, signals ev_cnt

## openpilot controlsd.py
### config realtime
        linux, not pc
        scheduler, affinity
### init
        cp car params
            fingerprint
            live state assistance
        ci car interface
        dt delta time
        steer curvature desired
        longitudinal control
            state pid last_accel
            state machine pid loop
                neg/pos limit
            off -> reset
            stop -> last min stop_rate, reset
            start -> start, reset
            pid err, upd
            ret last_ouo_accel
        latitude control
            angle(latcontrol)
            pid(latcontrol)
            torque control(latcontrol)
                upd limit: accel friction
                delay_frames jerk
                pid_log
                    err, actual, desire, saturation
                    saturation_time limit
                        output not limited by torque/angle limit
                        += dt delta time
### upd
        live calibrator
            euler angle roll/pitch/yaw
            -> rotation matrix 3x3
        live pose/measurement xyz/orientation/velocity/accel
### state control
        upd
            vehicle steer ratio, params
            steer angle, curvature
            torque
            plan
            model
            ena
            standstill actuator active
            blinker
            reset if not lat/long active
            accel pid loop
            accel actuator
            desire
            curvature
            delay
            upd steer angle degree
            actuator torque, steering angle
        ret car_control lac_log
### publish
        cur curvature
        calib frame -> car controller
            pose orientation/angular velocity
        cruisecontrol override, cancel, resume
        hudcontrol heads-up display from control command
            warnings, alerts, sys
            speed, lane, visual
            co car output
        contorl state
            new msg
            curvature, lon/lat time, desired
            pid p i/integral/reactive f/feedforward/desired
            force decel
                driver monitor
            tuning
                lat lac_log limit min
                    angle [-1...1]
                    pid
                    torque
### run
        upd
        state_control
        publish
        ratekeeper
            hz
            interval print_delay frame time
            monitor time

## plannerd.py
### cp car params
        msg bytes->log
    ldw lane departure warning
### longitudinal planner
        mpc model predictive control
        acc, cp car params
        fcw forward collision warning
        desired, prev, out, trajectory
        parse_model
            interpolated
            pos, velocity, accel
            throttle_prob
        upd
            mode blended, experimental, acc
            ego, cruise, decel, reset
            clip prev, desired, set_weights constraint
            cur state, interp()
            output -> target
        pub
            msg, valid
            plan, time, delay
            speed, accel, jerk, fcw
            target, stop, allow brake, throttle
### pub longplan driver assist
        drive helper
            limit/clip velocity/val min_speed, avoid divide 0
            basic curv psi/yaw angle target/vego*action
                target: desired heading angle
                    interpolate planned yaw at action timestep
            adjusted curv - psi_rate/vego
            steering cmd in curvature unit/inverse radius
            clip/max curvature, max lateral jerk
                min speed
                roll compensation, accel dut to gravity
                new_curvature clamp() float(clip())
    sub cc cs params radar model
### sub upd
        long_planner upd, pub
        ldw upd
        new msg assistance
        valid, left right
        pub send msg

## camera daemon
### config
        wide road
        road
        driver
        stream type, focal_len, init state, phy, ena, out
        isp image signal processor raw frame
        ife image front end fully processed
        bps bytes per sec fully processed
### v4l video4linux
        iterate thru avail devices
        open by name index
    cal exposure val
        pix_ptr/x y -> out img width -> luminance
        mean luminance val
            iterate 255-0, accumulate from histogram till middle
### raw frame
        heaparr buf->addr buf->len
        memcpy len -> addr
    img from device
        device frame single/arr pts -> transform to view frame
        -> img coords
        inp shape device
        pt_view depth never negtive -> filter out <0 NaN
        /[:, 2:3] divide xyz by z, convert 3d to 2d
        keep only xy, drop z [:, :2] after normalization
        reshape/restore img dim
### device from ecef frame
        earth-centered earth-fixed coord
        x forward, y right, z down
        np view as 2d
        device rotation matrix/transposed from orientation
        rel translate pt relative to device pos
        einsum() matrix multiplication -> rotate rel pt to device frame
        restore inp
### vp vanishing point calib
        intrinsics/internal property
            3d->2d focal len, principal pt/center, skew
            extrinsics: pos in the world
        normalize
            range 0-1 -1-1/distribution/unit vector
            invert 3x3 matrix
            pts->arr->2d->(x, y, 1) homogeneous coords
            ->stack arr in seq
            ->dot/multiplies by T transposed inv intrinsics matrix
            /pixel->normalized focal len=1, principal pt=origin,
            nan for neg
            input shape->reshape/drops 1, return just x,y [:2]

        yaw x [0] horizontal
        pitch y [1] vertical
        roll 0
    denormalize
        isfinite >width -> nan
### ke extract roll matrix-> intrinsics x essential/extrinsic matrix
        pitch xy
        yaw z
        roll
        arctan -> remove pitch yaw
    vp from product k*e intrinsic*extrinsic matrix
        x infinity parallel lines along x-axis
            dominated by x
            divide by depth
### view/camera/dest frame from calib/world/src/ref frame
        extrinsic transform from calib to view
            describe camera pose relative to calib
        3x4 get rotation from euler angles/roll pitch yaw
        apply additional fixed rotation/device-to-camera
        translation xz=0/centered, y-pos
        p_img = K * [R|t] * P_world
            K intrinsics
            rotation/R 3x3
            t/translation 3x1
             -> 3x4
    road/world/gravity: x->forward, y->left, z->up
    device/sensor/phone: x->forward, y->right, z->down
    view/camera len s/y=pixel: x->right, y->down, z->forward
        extrinsic

## model
### parse arg
    cl context
### model state
        frame
            run//context
        desire 0->1 rising edge & not prev_desire == inp
        pickle/py obj file
            vision
                map name to input_tensor_shape/(1, 640, 480, 3)
                out index, batch size
            policy
                map from state to action
        policy inp queue buf -> frequency mismatches
            model fps/context/state upd 10hz
            model run/env fps 20hz
            no. frame
        vision img buf -> opencl transform
            Tensor obj inp, out
            realize()
                immediate execute tensor computation
                alloc mem on device, run pending op
        run
            desire
            img cl
                name, transforms[name].flatten()
                tici qualcomm processor
                    img tensors <- opencl mem init once
                else cl buf reshape
                    tensor copy to np arr
            parse out vision
            inp enqueue vision out, desire
                env_fps == model_fps shape stay as is
                else rescale ratio env//model fps
                    img adjust + channels * frame
                reset zeros
                enqueue
                    key inps
                    single inp reshape
                    sliding window, fixed size for temp
                        [:,:-sz] [:,sz:] shift left, drop old
                        [:, -sz:] append new inp to end
                get/retrieve resample queued data based on fps
                    same fps: return queue directly
                    diff:
                        img: linearly interpolated n_frame_inp channels evenly spaced across the buf
                        pulse: max of any interval cnt
                        else: downsample/arrange, every env/model fps-th ele

            combine/concat vision+policy
### stream: camerad main
        extra client in avail
        wide camera not in avail_streams
### pub: model, drivingdata, camera odometry
### sub device, car, road state, live calib, monitor, control
    filter: track dropped frames
    main, extra
        transform
        buf
        meta
            frame_id, timestamp start end
        live calib false
    demo
    delay .2s
### desire helper
        lane change dir none/left/right
        off: dir <- blinker prevent ui flickering
        prelanechange:
            torque+dir >0 left, <0 right
            blindspot left/right
        starting: max
            proo > 98%
        finishing: min, fade in
        timer
        prev_blinker = blinker
        pulse_timer +=, only once, others none 0
### while true
        keep receiving frames until 1 frame ahead of prev
        buf_main = recv()
        extra client keep receiving til match main
            ck timestamp in sync
        else single camera: extra = main
        sub update
        desire, frame id, ego, delay
        upd, seen state -> live calib arr
            device camera type, sensor
            model transform main extra/warp matrix
        traffic convention
            left[1, 0] right[0, 1]
        desire < desire_len
        dropped frame max framed id-1
            <10 warm up =0
            drop ratio /1+
        bufs, transforms, inp(desire, traffic)
        perf counter
            before, after run time diff
        out
            new msg: model/drive/camera
            action, fill msg
            desire, lane_change, prob
            desire helper upd: sub, lat active,lane
                send model/driving lane change stat/dir
            pm send model/drive/camera
            last = main
### tinygrad helper
        qcom tensor from opencl addr
            extract raw gpu ptr from cl mem, wrap in tensor
                read 8-byte ptr to cl buf descriptor
                offset 0xA0
                create tensor directly from gpu mem w/o copying
            avoid data transfer overhead, using gpu mem
### parse out
        vision: pose, wide, road, lane
            parse mdn/mixture density network: plan
                out prob distribution, not just pt
                extraction:
                    pred_mu/means/predictions/first n_val
                    std/next n_val/exponentiated from stability
                    weight/remaining vals/softmaxed
                in_N>1/multi
                    extract all weights
                    sort by confidence/weight descending
                        [::-1] select/final the highest confidence [0]
                    store all weights/hypotheses for debugging
                output
                    name: means best prediction
                    name_stds
                    name_weight confidence
                    name_hypotheses: all hypotheses
                unpack compressed multimodal districution into predictions w/ prob
            binary crossentropy: lane, meta, lead prob
                sigmoid prob in [0,1]
            categorical cross entropy: desire pred
                raw.reshape + out_shape, unflatten to intended shape
                softmax: logits/rm to test importance
                     -> prob distribution across last dim
                        subtract max
                        clip to max 11 for loat 16 overflow
                        normalize sum to get prob
                normalize each hypothesis prob distribution separately
        policy mhp/multimodal hypothesis prediction
            out multiple predictions: plan
            catagorical cross entropy: desire state
### driving model frame:
        initialize gpu buf
            input_frames: raw frame data
            input_cl: inp frame
            buf_20hz: circular buf (temp_skip+1)frame
            last_img_cl: offset ptr sub_buf to latest frame in the circular
            loadyuv: set up for rgb conversion
            init_transform: set up perspective transform
        multiple frames w/o realloc, update sub buf
        prepare frame data for moddel
            run_transform() apply perspective transform to yuv frame
            (i+1)->i shift circular buffer, back one pos
            load queues yuv -> rgb conversion -> latest frame/img
            copy transformed + last frame
            finish sysc to ensure gpu work complete before model run
            ret ptr to inp_frame_cl, gpu buf ready for inference
### monitoring modelframe:
        inp frames cl/create buf
        init transform
        prepare
            run transform/model width height, frame width height
            finish/sync
        clean up for cl gpu rsc
            deinit_transfrom: release gpu mem buf
                destroy obj
                free v u y buf on gpu
            release mem/inp img buf
            destroy cl cmd queue

## laika
### gnss satellite processing lib -> pos/velocity
    einstein general relativity: 38 ms per day clk faster on sat than gnd
    compute station pos
        ecef earth centered earth fixed coord
        dog.cache_dir -> file -> obs data/rinex/gnss data 
            -> proc, corr -> local coord converter from slac center 
            -> fix/est err ecef[:3]/first 3 col to ned local
        slac vs cic vs c2p signal
            ned north east down, down has most variation
            mean
        glonass vs gps prob distribution
            global navigation satellite system --russia
            global positioning system -- us
            gssc.esa.int nauipedia
### walkthru
        constellation -> dog process, est_pos_fix, corr meas_epoch
            -> recv pos est
        satellite prn/pseudo-random noise 
            pos vel clk_err
            delay corr
                tropo/lower ionospheric/higher freq
                dcb/differential code biases/hardware corr
        plot sat orbit
            12h
            rescale, arr/256 -> rgb -> shape lons lats fig -> x y z
            ax subplot sample every 4th col
                limit 2.5e7 25m/ earth radius 6m
                box aspect equal scaling/no distortion
                pos
        1-min higway drive pos, vel
            arr -> obj -> measurements by epoch/sat plotting
                epoch a particular period of time
                -> no. of sat
        prr pseudorange-rate
            from doppler shifter each sat over time
                sat move towards, signal freq increases
            d1c l1 freq, civilian gps signal, obs
            tow time of week
        cp carrier phase prediction <- prev
            around 0 prediction from perfect meas/measurements
                incre = prr rate*time delta dt
        plot the world
            ecef vel est
            residuals of prr in velocity
            pos est
                earth radius -> x y z
                epoch
                    sat pos/np arr
            local coord/gnss
                converter sat pos
                x/azimuth/horizontal angle from north y/vertical angle from horizon
                    dop/dilution of precision high when x y similar
                sat high elevation vs low -> diff precision
### kalman filter 
        recursive best guess, normal/gaussian distribution, linear estimator
        combine/weight noisy, partial changing data over time
             -> a single est for pos, t, vel
             gain: weight to meas & current state est, tuned
        diff from ublox recvr raw data
            ublox gnss receiver
        log arr -> dog process corr -> build cache
        group meas by epoch/faster, easier to reason
        wls weighted least squares, est obs->model weighting by reliability/inv var
            -> output data -> laika pos
        ck -> gmap plotter
### astrodog
        valid latest data   
            closest min(candidate delta epoch) by norm(recv_pos-can_pos)
            download data func
        delay dgps
            correction cached = latest data
        sat_info prn data
            dcb differential code bias/hw diff, cached/downloaded from sat prn data, cache
            trop_delay trklib time, pos, el, humi, temp
            iono_delay ionexmap parsed from sat prn data, freq
                freq constellation/8 gps
            hierarchy approach
                orbits download sp3 3-day time step polynomial fitting
                    thread pool executor/parallel efficiency
                    orbit files from server/igs, cnt
                    parse sp3 file -> obj
                    ephemeris sat orbital interval data
                        min-max -> min_epoch, max_epoch
                navs
                    broadcast from rinex nav files/fallback
                    tgd time group delay/ionospheric delay corr
                        diff delay between l1 l2 freq -> corr meas
                    gpstime begin_day end_day
                qcom qualcomm polynomial ephemeris/device backup
### dgps
        closest station name/id, pos, tree, idxs
        parse
        corr/delay
### timerangeholder
        test if date in sparse ranges
        extend/create range
### sv space vehicle/sat identifier
        sat id/meas ranges -> constellation/gps prn[0] -> prn[1:] sv id -> offset
### dop
        hdop/horizontal, vdop, tdop/time, pdop
        ecef local -> 2ned/north west down relative -> norm -> col shape -> inv
### ublox receiver raw->rinex format
        c1c pseudorange in l1 freq c-code sigId==0 10x
        d1c doppler shift -> psudorange-rate l1 freq/offset
        l1c carriercycle l1, store
        s1c/cno signal strength db-hz, store signal-to-noise ratio

## agno-builder
### tools/extract_tools.sh
        arch/architecture cross-compilation !arm64
        extract tarball .tar.gz tape archive gnuzip/compress/zip if not already
            &>/dev/null suppress output
        git grep `-`/lfs ptr, pull large file storage obj
            awk text processing line-by-line, splitting into fields
                print 2nd col/extract name
### build_kernel.sh
        defconfig/default kernel config
        dir
        boot img
        macos apfs/apple file sys case-sensitive 
        docker build uid name, user/gid group
            container -d/detached, -v/volume flag, -w/working dir
            rm -r container_id
        clone git submodule status --cached | grep "^-" -> update --init
        tools/extract_tools.sh -> arch tar git lfs
        build params export cross-compilation, arm64, cache dir, host docker
            kcflags disable warnings, load defconfig
            nproc build kernel
                no. of processor scons -j4/4 jobs/build cmd in parallel
            cp image temp_dir dir/ 
            tools/mkbootimg image, ramdisk null, cmdline console, pagesize, base
            kernel/ramdisk/tags offset, output
        le/little endian signing/openssl
            dgst/cryptographic digest tool -sha256/hash boot img nonsecure 
                >/output
            dd/disk dump utility/cp data, if/infinite zeros, of/ouput 
                bs/block size cnt/block
            cat nonsecure sig.padded > $boot_img
        cp/mv img -> output_dir
        run kernel in docker exec
### build_system.sh
        ubuntu base if not, file, checksum/check sha256 sum
        dir build, output, rootfs, buld/img -> output/sys.img
        partition 4500m 10g
        tmp dir
        qemu x86_64 -> multiarch/user-static
        docker buildx
            docker buildkit check dir
            start build nsc/namespace container, bind-mount via container
                nsc build --load
            build dockerfile.agnos -> dir linux/arm64
            containerid = create entrypoint /bash
            check docker build dir
            meta/mount helper for macos/ci/namespace.so restricted
                docker build img in idr, load to local
            trap/cleanup container_id on exit
        docker -u/switch same user as script $@/pass all args
        docker exec mount container_id
        fallocate/prealloc disk space image size mkfs.ext4 img
            mount roofs dir <- image
            trap/run on exit/overwrite prev trap
                umount dir, docker rm container_id
        docker export .tar, tar .
            rm dockerenv
        network hostname host, resolv.config, ping setcap
            write build time/info
        git hash
        exec_as_root umount rootfs_dir 
        exec_as_user img/sparsify rootfs_img -> out_img
### dockerfile.agnos
        ubuntu/agno-compiler pkg compilation
            run apt-get update install pkgs
            env
            from cp run, cache, target, sharing
                capnp, ffmpeg/optimized bin, libqmi
                modemanager, lpac, qtwayland5.sh
        agnos-base/scratch as
            arg add ubuntu_base_image
            run mkdir set -xe/stop on err, arg username
            run echo resovconf, cp run base_setup.sh
            cp run/install openpilot dependencies.sh
            cp run/.deb qt libwayland debs
        from agnos-base hw setup
            run cp debs setup.sh, mv data/persist -> system
                rm data/
            cp libqmi/qualcomm msm interface comm
                modemanager/mobile broadband modem
                lpac/esim local sim profile assistant
                --from= pre-built bin, avoid recompilation
            run apt-get update install py lib polkit 
                overwrite libqmi/mode/lpac
        install
            cp pre-compiled capnp, run capnproto.deb
                pre-compiled ffmpeg
            cp uv, venv, opencl, compile bytecode
                python pkg
            cp run install_extras.sh
            cp run qtwayland5.deb 
            cp .patched libeglsubdriverwayland.so
                fixed nullptr deref bug in commitbuffer()
        config
            cp home/usename .config
            cp/populate/layer/freq-changed files last to cache earlier layers
                /lib .path .mount .service .timer override.conf fw/
                /etc fstab/table profile .rules .conf log, run .conf systemd
                /usr username font gnu abihf/app bin interface 
                    hw floating-point -> fast math op
            cp linux-headers.deb, run dpkg 
                large file, build on device
            cp weston/wayland compositor/display, gl-renderer.so/shared obj lib
                opencl lib gpu acceleration for rendering
                hacked touch rotate, color corr
            cp/setup services.sh
            run update-motd/msg of the day, cp root
                msg displayed when users log in
            cp devices.conf, networkmanager.conf
                manager eth0
            cp .nmconn, run chmod 600 .nmconn
                add cellular conn
            run echo -> gai.conf "precedence::fff" 
                ipv4 over ipv6
            cp logind.conf removeipc=no
                prevent logind delete shm/shared mem, persist
            cp/restart avahi-daemon override.conf
                sys drop-in file dns-sd, mem 50m, cpu 20%, timeout 10
                -> more robust
            cp/restart everyday polkit-override.conf -> etc/.../.
                -> prevent mem leak in policykit, ram usage increase
        network
            run rm -rf plugins/bearer from qt network
            iptable rules.v4/block incoming traffic on wwan0
            cp modemmanager.service in debug mode, run systemctl ena
            cp tmobile-network serviceproviders.xml
            cp camera fw binary/.elf host->img
                not provided in debian pkg
            run custom dns hostname nsswitch.conf
        rm
            arm-none-eabi for cortex-m4/m7
                rm find arm/, !thumb/v7
                rm find cpp server, !gcc objcopy objdump
                -> rm unused arch, smaller img, bare-metal c/no cpp
            run ldconfig/dynamic linker cache config/link for shared libs
                mkdir /tmp, cp run setup.sh
                rm -rf /tmp
            cp apt.conf, version file
            rm -rf icons, lists, cache, apt clean
### load_kernel.sh
        cd dir device
        scp/secure copy protocol boot.img -> device
        ssh
        sudo dd/disk dump reboot
### flash_kernel.sh
        tools/edl/emergency download mode
        w active_slot <- boot.img
            device recovery, bootloader upd
    flash_system.sh
    flash_all.sh
        w part_a/b <- img
        ./flash_kernel.sh flash_sys.sh
### weston gui
        test
            -o short-monotonic/time elapsed since boot
                -u weston display server logs
            socket path wayland-0, subprocess is_active
            weston.service file haslib
            subprocess open cmd/cd setup ui
                poll 10
            rc/return code
                kill proc
                out err
                sync()
                log
            else sudo reboot
        benchmark
            run/subproc call() wait timeout
            run systemctl stop weston, rm -rf
            time, cnt
            timestamp cmd/proc.open polll() setup ui
### sdm845 
        default config
            syscall, pid, module, arch, core dump, cpu, can driver
            spi, phy, debug, crypto, inv, mpu
        makefile/drivers
            +=esoc/external sys-on-chip
                if ena, compile incl. esoc dir
            powercap, fan, virt, hv, cpu, bluetooth, usb,audio, port 
            -y/always += tty/ gpu, soc, video, pwm, phy, ubs, irqchip
        module export func symbol for gpl-licensed kernel
            other modules, restrict to gpl
        drivers/fpga
            framework/mgr/manager config
                fw mgr ptr load -> req img
                    buf mgr ptr load -> dev w img to fpga -> config
                state/flags: power on/off, fw req, w init, err, complete, op
                dev->node match == data
                get dev node -> ref to mgr
                put mutext_unlock, put_device
                reg 
                    kazlloc()/alloc mem for struct, zeros, kernel flag
                    id alloc
                    mutex_init, sync ref cnt, prevent multiple threads
                    mgr->mops/ptr to ops w r cb, private, state
                    &mfgr->dev device init cls, parent, node, id
                    set drvdata, name
                        unreg get drvdata, rm
                cls init create group, release
                exit/destroy cls/ida interactive disassembler
                    module exit
            makefile
            obj-$() += drivers/socfpga.o
                link driver to kernel/loadable module
        drivers/watchdog/alim1535_wdt.c
            timer driver pmu/power management unit
            ...
            open/start/activate only once
                test, set_bit 
                |= (1<<25) timeout bits
            pci device id
            init
                find a watchdog/dev
                    ck series bridge, pmu
                    pci r config/init timer bits
                    reset
                        &= ~0x3F clears bits 0-5/timer
                        bits 24-27/issued ev
                        9-16, 12-13/no monitor bits
                    w config
                set timer timeout
                    t<60 |(1<<6) bit 6
                    <60min 1<<7
                reboot notifier for sys down
                misc/minor name fops register
                    write 
                        data addr, len, ptr pos offset
                        magic 'V' seq, reload, release, start
                    unlock/ioctl 
                        irq ack monitor extention 
                        cmd switch: cp_to get put user, alive
                watchdog exit/stop/clear/release
                    spin_lock
                    pci read config
                    mask disable &= ~0x3F 
                    &= ~(1<<25) timeout bits
        drivers/thermal/cpu_cooling.c
            state: get set max min/floor/limit cur power
                if state == max/last, isolate
                    unisolate
                upd freq
                clip/constraint > policy->max/user, if <, readjust policy->max
            cooling dev ptr-> tree node
                power extention/model opps/operation perf point lib
                    1ghz 0.9v 2ghz 1.1v
                    pwr -> freq
                np/node, clip/constraints/mask weight/num_cpus
                    capacitance/coefficient, static
                dyn = pwr - static
                    (raw_pwr*freq_dev->last_load)/100
                    load/delta time 
                    freq_dev dyn pwr_tb/kcalloc n ele, size bytes
                        max_level
                hp/hotplug ena/disable cpus runtime
                    upd/callback state lock, entry list cpu, unlock
                    online offline
                pw/pwr management
                    sleep, restore, suspend, post, resume
                    notify
            freq unreg
                dev ptr
                mutex_lock(&freq)
                unreg notifier for last dev
                mutex_lock(&list), del list, unlock
                thermal_unreg(cool_dev), release_idr/id allocator
                kfree(dev->tb/ts)
    scripts/download-from-manifest.py
        hash, lzma decompress, download json
### build.yaml
        artifacts output/ota
        pr no. cp version -> ota
        pkg_ota.py, push env
            files
                checksum sha hash, on device cksum/upd
                    header/unpack
                    magic, version, sz
                    chunk type 0xCAC1/hex raw in file format
                    0xCAC2/fill sz//4,upd()
                compress
                ret name, url, hash, raw, size, sparse, ck
                    shutil.copy(path, .img)
                    entry sectors
                gpts(6)
                partitions/persist, sys, xbl, dev, boot, user
            configs json url, staging, partitions
            deepcopy()/nested objs
            -> json.dump(f[alt][url])
        statistics analyze-rootfs.sh
            docker buld mount container_id, trap rm exit
            mount img -> dir
            du/disk usage -sh summary human-readable
            sed/stream editor, replace w/ '/'
            sz/rootfs total, local/py env, lib 
            umount img
        build_sys.sh
        cat version git pull.head.sha
        build_kernel.sh
        ccache hendrikmuhs, key 
        awk/text processing sdm845/submodule ref
        commit msg
### tesing.md checklist
        release
            test_onroad.py
                setup cls
                    debug
                    env replay, testing
                    shutil.rmtree/recur del log 
                        cleanup testruns
                    sys/mgr
                    time, proc
                    sub upd
                    route, len(seg) == 1
                    kill proc
                    path file log_size
                    msgs defaultdict(list)
                service freq
                    skip gps service
                    len(msg) >= floor(freq*t*0.8)
                        enough msgs subset 80%
                mgr start time
                    st - mgr_st < 15
                cloudlog size
                    msg sz < 3.5e5
                    3 most common, n/sum(cnt) > 30% == 0
                        zero log spam
                log size
                    min/log_sz/60*t*0.5 < size < *1.5
                ui timing
                    max(ts) < 250
                    mean < 20
                    camera preempt ui
                        len < 5
                cpu usage
                    log plot by proc
                    cpu_time[-1] - [0] usage < max
                    all/any(proc.key())
                    sum < total
                mem 
                    utility usage subporcess.decode()
                    avg(mems) <= 80
                    max(np.diff(mems)) <=4, avg <= 1
                camera frame timing
                    max(ts_diff, 50ms) < 5
                camera sync
                    eof > sof
                    len(frameId) == 1
                    next(id) < 100
                    max - min < 10
                    ts diff < 2
                camera encoder match
                    road camera state, enc idx
                    frames id, timestamp
                        enc_sof == cam_sof
                        enc_eof == cam_eof
                mpc/model predictive control, over time
                    cfgs longitudinalplan max 0.05
                model exec timing
                    cfgs avg freq 20hz
                    ts msgs[s] min max mean
                    max < instant_max
                    mean < avg_max
                service timing
                    header/service, max, min, mean 0.06
                        rsd/relative std deviation /mean, data consistency
                    ts np.mean(ts), max(ts)/nano 1e9/time diff between msgs
                    dt/expected time interval 1/list.freq
                        rtol/rel tolerance 0.03, atol 0
                    std(ts)/dt > rsd
                    tabulate(row, header)
                    passed if no err
                startup/release_only alert text
                engagable
                    roadev -> no. of entries/Counter()/dict cnt hashable objs
                    true/false offset/slice msg list from offset onward
            wifi, modem-cell, image size, sound
            scons -c/clean build, factory reset/boot
            color calib/tizi mici
            agnos upd
            ui.py/display
                main_layout.render()
                content, sidebar, rect, click, settings, mode, upd
                setup_callbacks, transistion
        abl/uefi bootloader
            a/b slot/boot from backup if primary fails
            boot/load kernel time <1s
        xbl extensible bootloader
            primary bootloader
            temp cold/hot
            60fps display
            boot time <2.4s
        setup
            real/correct url, valid installer_url, boot
    


## simulator
    setup
        tools/op.sh setup
        source .venv/bin/activate
        scons -u -j$(nproc)
    ctf 
        tools/ selfdrive/debug/
        tools/replay 
            ./replay '0c7f0c7f0c7f0c7f|2021-10-13--13-00-00' --dcam --ecam
        selfdrive/ui/ui 
            seeking in replay
    metadrive
        tools/sim/launch_openpilot.sh
        ./run_bridge.py -h
            engage 2
                up speed 1
                down speed 2
            | key  |   functionality       |
            |------|-----------------------|
            |  1   | Cruise Resume / Accel |
            |  2   | Cruise Set    / Decel |
            |  3   | Cruise Cancel         |
            |  r   | Reset Simulation      |
            |  i   | Toggle Ignition       |
            |  q   | Exit all              |
            | wasd | Control manually      |

