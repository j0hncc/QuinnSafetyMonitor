# QuinnSafetyMonitor
Temperature monitor for RV

## Mqtt API
	* String topic_prefix = "misc";  // tpx

  * Inputs to device (device subscriptions)
     * tpx/cmd  
       * start  
       * stop  
       * config  # invoke wifimanager settings access point  
     * tpx/setsec seconds   # report on X seconds intervals (default 1 minute)  

  * Outputs from device ( device publications)
    * tpx/temp/degF degrees   
    * tpx/humid/rh   %rh  
    * tpx/press/inHg  inchesMercury  
