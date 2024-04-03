function Decoder(bytes, port) {
  var cpu_info;
  var used_memory;
  var free_memory;
  var memory_percent;
  if (bytes.length==7){
    cpu_info = (bytes[0]/100).toFixed(2);
    used_memory = ((bytes[1]<<8|bytes[2])/100).toFixed(2);
    free_memory = ((bytes[5]<<8|bytes[6])/100).toFixed(2);
    memory_percent = ((bytes[3]<<8|bytes[4])/100).toFixed(2);
    return{
      "cpu_info":cpu_info + "%",//cpu使用率
      "used_memory":used_memory+"GB",//已使用内存
      "free_memory":free_memory+"GB",//剩余内存
      "memory_percent":memory_percent+"%"//内存利用率
    };
  }
  else{
    cpu_info = ((bytes[0]<<8|bytes[1])/100).toFixed(2);
    used_memory = ((bytes[2]<<8|bytes[3])/100).toFixed(2);
    free_memory = ((bytes[6]<<8|bytes[7])/100).toFixed(2);
    memory_percent = ((bytes[5]<<8|bytes[6])/100).toFixed(2);
    return{
      "cpu_info":cpu_info + "%",//cpu使用率
      "used_memory":used_memory+"GB",//已使用内存
      "free_memory":free_memory+"GB",//剩余内存
      "memory_percent":memory_percent+"%"//内存利用率
    };
  }

}