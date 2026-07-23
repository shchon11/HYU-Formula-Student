/*
 * sbg_settings_action — send an SBG ECom SETTINGS_ACTION over serial.
 *
 * The device REST API (sbgEComApi) has no save endpoint, and REST POSTs only
 * STAGE settings — they do not affect the live output until the device reboots
 * with the staged config applied. This tiny tool sends the binary
 * SBG_ECOM_CMD_SETTINGS_ACTION so staged REST changes actually take effect.
 *
 *   sbg_settings_action /dev/ttyUSB0 115200 save     # save to flash + reboot (applies staged config)
 *   sbg_settings_action /dev/ttyUSB0 115200 reboot   # reboot only (DISCARDS staged, reverts to flash)
 *
 * Build: see build-sbg-tools.sh (links the sbgECom static lib).
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sbgEComLib.h>

int main(int argc, char **argv)
{
    if (argc < 3)
    {
        printf("usage: %s SERIAL_DEVICE BAUDRATE [save|reboot]   (default: save)\n", argv[0]);
        return 2;
    }

    SbgInterface iface;
    SbgErrorCode err = sbgInterfaceSerialCreate(&iface, argv[1], atoi(argv[2]));
    if (err != SBG_NO_ERROR)
    {
        fprintf(stderr, "ERROR: cannot open serial %s @ %s (code %d)\n", argv[1], argv[2], err);
        return 1;
    }

    SbgEComHandle handle;
    err = sbgEComInit(&handle, &iface);
    if (err != SBG_NO_ERROR)
    {
        fprintf(stderr, "ERROR: sbgEComInit failed (code %d)\n", err);
        sbgInterfaceDestroy(&iface);
        return 1;
    }

    SbgEComSettingsAction action = SBG_ECOM_SAVE_SETTINGS;
    if (argc >= 4 && strcmp(argv[3], "reboot") == 0)
        action = SBG_ECOM_REBOOT_ONLY;

    printf("Sending SETTINGS_ACTION=%d (%s) ...\n", action,
           action == SBG_ECOM_SAVE_SETTINGS ? "SAVE_SETTINGS: save to flash + reboot"
                                            : "REBOOT_ONLY: reboot, discard staged");
    err = sbgEComCmdSettingsAction(&handle, action);
    if (err == SBG_NO_ERROR)
        printf("OK — device is saving and rebooting (give it ~5-10 s).\n");
    else
        fprintf(stderr, "ERROR: settings action failed (code %d)\n", err);

    sbgEComClose(&handle);
    sbgInterfaceDestroy(&iface);
    return (err == SBG_NO_ERROR) ? 0 : 1;
}
