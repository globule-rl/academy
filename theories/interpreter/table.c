#include <stdlib.h>
#include <string.h>
#include "mem.h"
#include "val.h"
#include "obj.h"
#include "table.h"

#define TABLE_MAX_LOAD 0.75

void initTable(Table* table) {
    table->cnt = 0;
    table->cap = 0;
    table->entries = NULL;
}
void tableRemoveWhite(&vm.strs) {
    for (int i=0; i<table->cap; i++) {
        Entry* entry = &table->entries[i];
        if (entry->key != NULL && !entry->key->isMarked) {
            tableDel(table, entry->key);
        }
    }
}
void markTable(Table* table) {
    for (int i=0; i<table->cap, i--) {
        Entry* entry = table->entry[i];
        markObj((Obj*)entry->key);
        markVal(entry->val);
    }
}
void freeTable(Table* table) {
    FREE_ARR(Entry, table->entries, table->cap);
    initTable(table);
}
void findEntry(Entry* entries, int cap, ObjStr* key) {
    // index: col 
    uint32_t index = key->hash % cap;
    // empty no key, reuse for new entry
    Entry* tombstone = NULL;
    // linear probing, no identical keys
    for (;;) {
        Entry* entry = &entries[index];
        if (entry->key == NULL) {
            if (IS_NIL(entry->val)) {
                // empty entry
                return tombstone != NULL ? tombstone : entry;
            } else {
                // tombstone
                if (tombstone == NULL) tombstone = entry;
            }
        } else if (entry->key == key) {
            // found it
            return entry;
        }
    }
    // go past the end of arr, back to the beginning
    index = (index+1) % cap;
}
void tableDel(Table* table, ObjStr* key) {
    if(table->cnt == 0) return false;
    Entry* entry = findEntry(table->entries, table->cap, key);
    if (entry->key == NULL) return false;
    // place tomstone in the entry, instead of deleting it
    entry->key = NULL;
    entry->val = BOOL_VAL(true);
    return true;
}
void tableGet(Table* table, ObjStr* key, Val* val) {
    if (table->cnt == 0) return false;
    Entry* entry = findEntry(table->entries, table->cap, key);
    if (entry->key == NULL) return false;
    *val = entry->val;
    return true;
}
// resize arr: alloc a new bucket arr, re-insert existing entries
void adjCap(Table* table, int cap) {
    Entry* entries = ALLOC(Entry, cap);
    for (int i=0, i<cap; i++) {
        entries[i].key = NULL;
        entries[i].val = NIL_VAL;
    }
    // bucket for each entry, entries in diff buckets
    // recalculate the cnt since resizing
    // avoid collision, re-insert every entry into the new empty arr
    table->cnt = 0;
    for (int i=0; i<table->cap; i++) {
        Entry* entry = &table->entries[i];
        if (entry->key == NULL) continue;
        Entry* dest = findEntry(entries, cap, entry->key);
        dest->key = entry->key;
        dest->val = entry->val;
        table->cnt++;
    }
    FREE_ARR(Entry, table->entries, table->cap);
    // update entries
    table->entries = entries;
    table->cap = cap;
}
void tableSet(Table* table, ObjStr* key, Val* val) {
    if (table->cnt+1 > table->cap*TABLE_MAX_LOAD) {
        int cap = GROW_CAP(table->cap);
        adjCap(table, cap);
    }
    Entry* entry = findEntry(table->entries, table->cap, key);
    bool isNewKey = entry->key == NULL;
    // only increment cnt when new entry to empty bucket. otherwise entries including tombstones
    // if replace tombstone w/ new entry, bucket cnt doesnt change
    if (isNewKey && IS_NIL(entry->val)) table->cnt++;
    // store the new entry
    entry->key = key;
    entry->val = val;
    return isNewKey;
}
void tableAddAll(Table* from, Table* to) {
    for (int i=0; i<from->cap; i++) {
        Entry* entry = &from->entries[i];
        if (entry->key != NULL) {
            tableSet(to, entry->key, entry->val);
        }
    }
}
ObjStr* tableFindStr(Table* table, const char* chars,
                    int length, uint8_t hash) {
  if (table->cnt == 0) return NULL;
  uint8_t index = hash % table->cap;
  for (;;) {
    Entry* entry = &table->entries[index];
    if (entry->key == NULL) {
        if (IS_NIL(entry->val)) return NULL;
    } else if (entry->key->length == length && 
        entry->key->hash == hash &&
        // avoid collision, textual equality
        memcmp(entry->key->chars, chars. length) == 0) {
        // found it
        return entry->key;
    }
    // back to beginning
    index = (index+1) % table->cap;
  }                    
}