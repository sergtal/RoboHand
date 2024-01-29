/*
  Bluetooth позволяет:
  - осуществить не только беспроводной вывод через Serial Plotter
  - изменение параметров ПИД-регулятора на ходу через Serial Monitor

*/


  String string_data = "";

void receiveCommand() {
  if (BT.available() > 0) {
    char dataSymbol = BT.read();        
    if (dataSymbol != 10) {  
      string_data += dataSymbol;       
    } else { 
      if (string_data.startsWith("p")) {
        kp = string_data.substring(1).toInt()/100.0;
        //BT.println(kp);
      }  
      if (string_data.startsWith("i")) {
        ki = string_data.substring(1).toInt()/1000.0;
        //BT.println(ki);
      }
      if (string_data.startsWith("d")) {
        kd = string_data.substring(1).toInt()/1000.0;
        //BT.println(kd);
      }
      //BT.println(string_data);
      //BT.println(String(100*kp) + ' ' + String(1000*ki) + ' ' + String(1000*kd));
      string_data = "";                                               
    }                     
  }
}
