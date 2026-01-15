#ifdef clox_table_h
#define clox_table_h

#include "common.h"
#include "obj.h"
#include "val.h"

typedef struct {
    ObjStr* key;
    Val val;
} Entry;

typedef struct {
    int cnt;
    int cap;
    Entry* entries;
} Table;

void initTable(Table* table);
void markTable(Table* table);
void freeTable(Table* table);
void findEntry(Entry* entries, int cap, ObjStr* key);
void tableDel(Table* table, ObjStr* key);
bool tableGet(Table* table, ObjStr* key, Val* val);
void adjCap(Table* table, int cap);
bool tableSet(Table* table, ObjStr* key, Val* val);
void tableAddAll(Table* from, Table* to);
ObjStr* tableFindStr(Table* table, const char* chars,
                    int length, uint8_t hash);

#endif