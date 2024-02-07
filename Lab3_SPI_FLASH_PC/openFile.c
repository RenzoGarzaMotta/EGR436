#include <stdio.h>
#include <stdlib.h>

int main() {
    FILE *fp;
    char filename[100];
    char ch;

    printf("Enter a filename: ");
    scanf("%s", filename);

    fp = fopen(filename, "r");

    if (fp == NULL) {
        printf("Could not open file %s", filename);
        return 1;
    }

    while ((ch = fgetc(fp)) != EOF) {
        // printf("%c -> %x\n", ch, ch);
        printf("%c", ch);
    }

    fclose(fp);
    return 0;
}