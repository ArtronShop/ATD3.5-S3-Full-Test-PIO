SET(SOURCES screens/ui_Index.c
    screens/ui_Touch.c
    screens/ui_Display.c
    screens/ui_Sound.c
    screens/ui_I2C.c
    screens/ui_MicroSD_Card.c
    screens/ui_USB_OTG.c
    screens/ui_LED_SW.c
    screens/ui_IO.c
    screens/ui_WiFi.c
    ui.c
    components/ui_comp_hook.c
    ui_helpers.c)

add_library(ui ${SOURCES})
