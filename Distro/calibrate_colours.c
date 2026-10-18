
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
    int rgb[3];
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

        printf("\n Hover the colour sensor over %s \n", colours[i]);
        printf("Press ENTER to calibrate");
        fflush(stdout);

        while (getchar() != '\n')
        {
        }

        for (int j = 0; j < CALIBRATION_SAMPLE_COUNT; j++)
        {
            BT_read_colour_sensor_RGB(SENSOR_PORT, rgb);

            red_total += rgb[0];
            green_total += rgb[1];
            blue_total += rgb[2];

            usleep(100000);
        }

        fprintf(file, "%s,%.2f,%.2f,%.2f\n",
                colours[i],
                (double)red_total / CALIBRATION_SAMPLE_COUNT,
                (double)green_total / CALIBRATION_SAMPLE_COUNT,
                (double)blue_total / CALIBRATION_SAMPLE_COUNT);

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
