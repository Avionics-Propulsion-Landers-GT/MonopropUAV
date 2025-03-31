#include "print.h"
#include <stdio.h>


// Print functions (due to std:: absence)
void print_str(const char* s) {
    fputs(s, stdout);
}

// // Print integer
// void print(int n) {
//     printf("%d", n);
// }

// Print float/double
void print_dbl(double d) {
    printf("%f", d);
}

// Print newline
void println() {
    fputc('\n', stdout);
}

void printerr_open_fail(const char* filename) {
    const char* prefix = "Error: Could not open file: ";
    fputs(prefix, stderr);
    fputs(filename, stderr);
    fputc('\n', stderr);
}