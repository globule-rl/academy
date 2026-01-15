#include <multiTasking.h>

using namespace os;
using namespace os::common;

/* 
    code seg: base addr, limit, flags/access/properties
    flag 0x202: 514 2^9=512 2^10=1024
*/
Task::Task(Gdt* gdt, void entrypoint()) {
    cpustate = (CPUState*)(stack + 4096 - sizeof(CPUState));
    cpustate -> eax = 0;
    cpustate -> ebx = 0;
    cpustate -> ecx = 0;
    cpustate -> edx = 0;
    cpustate -> esi = 0;
    cpustate -> edi = 0;
    cpustate -> ebp = 0;
    cpustate -> eip = (uint32_t)entrypoint;
    cpustate -> cs = gdt->CodeSegSelector();
    cpustate -> eflags = 0x202;
}
Task::~Task() {}

TaskManager::TaskManager() {
    numTasks = 0;
    curTask = -1;
}
TaskManager::~TaskManager() {}

bool TaskManager::addTask(Task* task) {
    if (numTasks >= 256) return false;
    tasks[numTasks++] = task;
    return true;
}
/* round-robin task scheduler
        each process equal portion in circular order
    no task -> no change for cpustate
        cur cpustate -> curTask cpustate
        increment curTask -> end, wrap around to the beginning %
        return cpustate for next task
*/
CPUState* TaskManager::Schedule(CPUState* cpustate) {
    if (numTasks <= 0) return cpustate;
    if (curTask >= 0 && curTask < numTasks && tasks[curTask]) tasks[curTask]->cpustate = cpustate;
    if (++curTask >= numTasks) curTask %= numTasks;
    return tasks[curTask]->cpustate;
}