function decodeUplink(input) {
    var data = {};
    data.battery     = ((input.bytes[0] << 8) + input.bytes[1])/1000;
    data.temperature = (((input.bytes[2] << 8) + input.bytes[3])-5000)/100;
    
    data.humidity    = ((input.bytes[4] << 8) + input.bytes[5])/100;
    data.pressure    = ((input.bytes[6] << 8) + input.bytes[7])/10;
    
    data.weight = (input.bytes[8] << 24 | input.bytes[9] << 16 | input.bytes[10] << 8 | input.bytes[11]);
        
    // TODO find a way to put RSSI to Payload 
    //data.dev_rssi = rssi;
    
    var warnings = [];
    // example for Warnings
    if (data.temperature < -10) {
      warnings.push("it's cold");
    }
    return {
      data: data,
      warnings: warnings
    };
  }