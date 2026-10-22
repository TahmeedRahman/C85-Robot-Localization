#include "EV3_Localization.h"

int main(void)
{
    const char *colours[] = {
        "Yellow",
        "Black",
        "Red",
        "Blue",
        "Green",
        "White"};
    int r, g, b, a;
    FILE *file;

    if (BT_open(HEXKEY) != 0)
    {
        fprintf(stderr, "Unable to open comm socket to the EV3, make sure the EV3 kit is powered on, and that the \n");
        fprintf(stderr, " hex key for the EV3 matches the one in EV3_Localization.h \n");
        exit(1);
    }

    file = fopen(CALIBRATION_OUTPUT_FILE, "w");

    if (file == NULL)
    {
        perror("Failed to save calibration");
        BT_close();
        exit(1);
    }

    for (int i = 0; i < NUM_COLOURS; i++)
    {
        long red_total = 0;
        long green_total = 0;
        long blue_total = 0;
        long ambient_total = 0;
        int samples = 0;
        int tries = 0;

        printf("\n Hover the colour sensor over %s \n", colours[i]);
        printf("Press ENTER to calibrate");
        fflush(stdout);

        while (getchar() != '\n')
        {
        }

        while (samples < CALIBRATION_SAMPLE_COUNT &&
               tries < 2 * CALIBRATION_SAMPLE_COUNT)
        {
            tries++;

            if (BT_read_colour_RGBraw_NXT(COLOUR_PORT, &r, &g, &b, &a) == 1)
            {
                red_total += r;
                green_total += g;
                blue_total += b;
                ambient_total += a;
                samples++;
            }

            usleep(100000);
        }

        if (samples == 0)
        {
            fprintf(stderr, "Failed to read colour sensor for %s\n", colours[i]);
            fclose(file);
            BT_close();
            exit(1);
        }

        fprintf(file, "%s,%.2f,%.2f,%.2f,%.2f\n",
                colours[i],
                (double)red_total / samples,
                (double)green_total / samples,
                (double)blue_total / samples,
                (double)ambient_total / samples);

        fflush(file);

        if (i < NUM_COLOURS - 1)
        {
            printf("Move the sensor to %s when ready.\n",
                   colours[i + 1]);
        }
    }

    fclose(file);
    BT_close();

    printf("\n Calibration complete \n");
    exit(0);
}