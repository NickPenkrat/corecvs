#include <stdio.h>

#include "global.h"



int main (int argc, char **argv)
{
    printf("Bundler postreconstrutor.\n");

    if (argc != 2)
    {
        printf("No input\n");
        return -1;
    }

    const char* filename = argv[1];

    SYNC_PRINT(("Starting the read of %s\n", filename));







    return 0;
}
