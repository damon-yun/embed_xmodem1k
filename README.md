# embed_xmodem1k

you must to overwrite some funcation below

xmodem_fun_handle_t gp_xmodem_fun ={
    uartreadnoblock,
    uartsendnoblock,
    sysTimerClr,
    sysTimerGet,
    NULL,
    
};

just call this funcation "xmodem1k_client(boot_data_program,&gp_xmodem_fun,100,1000)" to start client.