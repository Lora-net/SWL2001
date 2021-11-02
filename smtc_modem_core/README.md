This is the soft modem core code that can be embedded in several top projects.

In this repository you will find 5 subfolders:  
* device_management:  
Functions and helpers that handle downlink from LoraCloud and all that is related with the soft modem context

* lorawan_api:  
Soft layer that is used to abstract the LoRaWAN stack that is used below

* modem_services:  
All services supported by the modem and the LoRaCloud (file upload, streaming, alc_sync)

* modem_supervisor:  
The soft modem task scheduler

* test_modem:  
Here is handled the soft modem test mode
