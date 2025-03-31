#ifndef PRINT_H
#define PRINT_H

#ifdef __cplusplus
extern "C" {
#endif

void print_str(const char* str);
// void print(int val);
void print_dbl(double val);
void println();
void printerr_open_fail(const char* filename);

#ifdef __cplusplus
}
#endif

#endif // PRINT_H