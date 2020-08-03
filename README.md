# epsrelay

esp32 mqtt relay with hlw8032 current sensor

project consists of app and factory flashable fils and pcb,


[App partition](app)  
[Factory partition](factory)  
[PCB](pcb)

project was based on ESPMDF with 3.3idf version and might not compile with newer one

## TODO
libesphttpd lib used in factory has questionable security futures and should be replaced with smth better
add external pullup to factory reset pin as sometimes it falsely triggers on boot? EDIT factory pin is 2.8V might be faulty esp module or bad soldering work as dev board with same soft measures 3.2