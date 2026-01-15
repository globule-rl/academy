#pragma once

#include <common/types.h>
#include <gdt.h>

namespace os {
    /* general registers: 
        eax: accumulator registor/arthmetic
        ebx: base registor/data ptr
        ecx: counter register/loop counter
        edx: data register/data op/extensions
    index n ptrs:
        esi: source index/str mem op ptr
        edi: destination index/str dest ptr
        ebp: stack base pointer/stack frame ptr
        eip: instruction pointer/next instruction addr
        esp: stack pointer/stack top ptr
    seg registers:
        cs: code seg
        ds: data seg
        es: extra seg/video mem
        fs: extra seg/video mem
        gs: extra seg/video mem
        ss: stack seg/data
    eflags: flag register/status control sys flags 
        zf: zero flag
        cf: carry flag
        tf: trap flag
        if: irq flag 
    */
    struct CPUState {
        uint32_t eax;
        uint32_t ebx;
        uint32_t ecx;
        uint32_t edx;
        uint32_t esi;
        uint32_t edi;
        uint32_t ebp;
        uint32_t error;
        uint32_t eip;
        uint32_t cs;
        uint32_t eflags;
        uint32_t esp;
        uint32_t ss;

    };
    /* friend class: access to private/protected members */
    class Task {
        friend class TaskManager;
        private:
            uint8_t stack[4096];
            CPUState* cpustate;
        public:
            Task(Gdt* gdt, void entrypoint());
            ~Task();
    };
    class TaskManager {
        private:
            Task* tasks[256];
            int numTasks;
            int curTask;
        public:
            TaskManager();
            ~TaskManager();
            bool addTask(Task* task);
            CPUState* Schedule(CPUState* cpustate);
    };
}

