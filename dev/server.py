import firebase_admin 
from firebase_admin import credentials
from firebase_admin import firestore
import json

import serial
import time

ser = serial.Serial(
    port = '/dev/ttyS0',
    baudrate = 9600,
    parity = serial.PARITY_NONE,
    bytesize = serial.EIGHTBITS,
    timeout = 1000000000,
    stopbits = serial.STOPBITS_ONE
)

cred = credentials.Certificate('./serviceAccount.json');
app = firebase_admin.initialize_app(cred)
db = firestore.client()

def read_database(col, doc):
    doc_ref = db.collection(col).document(doc)
    return (doc_ref.get()).to_dict()

def write_database(col, doc, body):
    db.collection(col).document(doc).set(body)


def main():
    current_data = {
        'temperature': "0.0",
        'pH': "0.0"
    }
    while True:
        ser.flush()
        ser.flushInput()
        ser.flushOutput()
        # Read new target data
        target_data = read_database('Controls', 'Target')
        
        # Send target data to stm via uart
        msg = ""
        # Temperature data
        msg += "T" + target_data["temperature"]
        # PH data
        msg += "P" + target_data["pH"]
        # Light data
        msg += "L" + target_data["LED"][0] + target_data["LED"][1] + target_data["LED"][2] + target_data["LED"][3]
        
        # Feeding data
        msg += "F";
        
        if target_data["serve_betta"] == "1":
            msg += "1"
            # Make feeding no longer necessary
            target_data["serve_betta"] = "0";
            write_database('Controls', 'Target', target_data)   
        else:
            msg += "0"
        msg += target_data["size_betta"]
        
        if target_data["serve_flake"] == "1":
            msg += "1"
            # Make feeding no longer necessary
            target_data["serve_flake"] = "0";
            write_database('Controls', 'Target', target_data)  
        else:
            msg += "0"
        msg += target_data["size_flake"]
        
        # Send the data via uart
        print(msg)
        ser.write(msg.encode())
        
        # Receive data from the stm
        receive_data = ser.read(12)
        
        
        # make sure we read 12 valid bytes, if not, wait for msgs to be realigned
        #if receive_data[11] != 10:
         #   ser.flush()
          #  ser.flushInput()
           # ser.flushOutput()
            #time.sleep(5)
            #print("Invalid UART Message Received")
            #continue
        
        current_data['temperature'] = str(receive_data[:5])
        current_data['pH'] = str(receive_data[6:10])
        print(current_data)
        # Update database for current statistics
        write_database('Controls', 'Current', current_data)
        time.sleep(1)
#     ser.close();
    

if __name__ == "__main__":
    main()
    
 