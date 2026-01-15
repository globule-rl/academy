data structures
    stack automatically
        local var: func locals, temp, val struct primitives
        call frames: return address, instruction pointer
    heap garbage collector
        Objs: strs, func, closures, func, class, boundmethod, instance
        globals: global vars
        constant: num, str in constant pool, the whole program lifetime, gc/never freed
e.g.
    locals
        callframe -> stack index, short lived until return
    upval -> captured locals
        heap/obj arr -> ptr from closure, as long as closure refd
    globals
        table/dict -> name lookup, permanent
    strings
        table -> func->chunk.constants.vals -> constant id index, as long as chunk exists
            ip = chunk.code
vm stacks -> frame
    val vm.stackTop(sentinel and then stacktop)
    callFrame
        vm.frames[] vm.frameCnt
        ptr to func closure being called
    local var  frame->slots[slot]
    func call state
            stackTop slot 0 -> op
            constant table slot++
    upval *frame->closure->upvals[slot]-location
        upval = vm.openupvals
vm (*objs frames[] frameCnt stack stackTop openUpvals **graystack ip globals strs)
    Vm vm frame: closure ip *slots frame = frames[i] 
        vm.frame->ip
        closure: upvalcnt func *upvals
            vm.frame->closure->upvalcnt
            vm.frome->closure->func
garbage collection
    any resizing triggers
types
    num 
    objs func closures frames
    vals 
        unions as
            primitives 
            objs
    boolean
    none
    cast 
        type VAL_OBJ VAL_NUM
        isType IS_OBJ IS_NIL
        val OBJ_VAL
        unions as AS_OBJ AS_NUM
set & get
    set: have a val on stacktop to update
        local: push from frame slot 0 to stacktop 
            push(frame->slots[slot]) 
                slot: local from index 0, first local to aritycnt
        global: in table, update var
    get: get val from tables
compiler
    compiler->func block recursively
    compiler->closure->upval->isLocal index
table:
    obj bound menthod table
    obj instance fields table
    globals table
    strs/constant table
class:
    invoke
        if is_instance
            -> call val initializer 
                -> call frame closure ip slots
        else class
            -> method closure func
chunk (cap, *code, *line, cnt, constants)
    cur -> prev/old
    GROW_CAP GROW_ARR(chunk->code) GROW_ARR(chunk->line)
    chunk->cnt index chunk->code[chunk->cnt] = byte chunk->cnt++
    ValArr constants
offset:
    cur - start
        +1: cnt
        +2: jump placeholder offset 1 from op, skip then jump
        -2: read_short 2 bytes/16-bit before jump
    -1: cnt-1, index in arr
jump:
    skipthenjump, pop condition true
    skipelsejump, patch then jump, pop condition false, patch else jump
    while emit back to loopstart loopstart=increm, patch jump body
1 byte = 8-bit
    frame stack 
    tables class methods fields constant/strs globals
    chunk valarr
    objs objstr objfunc objupval objclosure objboundmethod objclass objinstance

