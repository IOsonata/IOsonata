//
//  ViewController.swift
//  BlueIOThingy
//
//  Created by Tai on 2021-12-14.
//

import Cocoa
import CoreBluetooth
import Charts

class BlueIOPeripheral: NSObject {

        public static let BLUEIO_UUID_SERVICE   = CBUUID.init(string: "ef680100-9b35-4933-9b10-52ffa9740042")
        public static let BLUEIO_UUID_ENVIRONMENT   = CBUUID.init(string: "ef680200-9b35-4933-9b10-52ffa9740042A")
        public static let BLUEIO_UUID_MOTION   = CBUUID.init(string: "ef680400-9b35-4933-9b10-52ffa9740042")
    
        
    }

class ViewController: NSViewController, CBCentralManagerDelegate, CBPeripheralDelegate {

    var bleCentral : CBCentralManager!
    var mBlueIODevice: CBPeripheral!
    var tempDataEntries = [ChartDataEntry]()
    var humiDataEntries = [ChartDataEntry]()
    var pressDataEntries = [ChartDataEntry]()
    var xValue: Double = 1
    
    var tempDS = LineChartDataSet()
    var humiDS = LineChartDataSet()
    var pressDS = LineChartDataSet()
    
    var tempData = LineChartData()
    var humiData = LineChartData()
    var pressData = LineChartData()
    
    @IBOutlet weak var tempLabel: NSTextField!
   
    @IBOutlet weak var humiLabel: NSTextField!
    @IBOutlet weak var pressLabel: NSTextField!
    
    @IBOutlet weak var rssiLabel: NSTextField!
    @IBOutlet weak var aqiLabel: NSTextField!
    
    @IBOutlet weak var tempChart: LineChartView!
    
    @IBOutlet weak var humiChart: LineChartView!
    
    @IBOutlet weak var pressChart: LineChartView!
    
    override func viewDidLoad() {
        super.viewDidLoad()

        // Do any additional setup after loading the view.
        bleCentral = CBCentralManager(delegate: self, queue: DispatchQueue.main)
        SetupChart()
        
    }

    func SetupChart(){
        self.tempDS = LineChartDataSet(entries: self.tempDataEntries, label: "Temperature")
        self.tempDS.colors = [NSUIColor.red]
        self.tempData.append(self.tempDS)
        self.tempChart.data = self.tempData
        
        self.humiDS = LineChartDataSet(entries: self.humiDataEntries, label: "Humidity")
        self.humiDS.colors = [NSUIColor.red]
        self.humiData.append(self.humiDS)
        self.humiChart.data = self.humiData
        self.humiChart.data = self.humiData
        
        self.pressDS = LineChartDataSet(entries: self.pressDataEntries, label: "Pressure")
        self.pressDS.colors = [NSUIColor.red]
        self.pressData.append(self.pressDS)
        self.pressChart.data = self.pressData
        self.pressChart.data = self.pressData
        

    }
    func UpdataCharts(with newTempDataEntry: ChartDataEntry, with newHumiDataEntry: ChartDataEntry, with newPressDataEntry: ChartDataEntry){
        self.tempDataEntries.append(newTempDataEntry)
        self.tempDS.removeAll()
        self.tempData.removeAll(keepingCapacity: false)
        self.tempDS = LineChartDataSet(entries: self.tempDataEntries, label: "Temperature")
        self.tempData.append(self.tempDS)
        self.tempChart.data = self.tempData
        
        self.tempDS.notifyDataSetChanged()
        self.tempData.notifyDataChanged()
        self.tempChart.notifyDataSetChanged()
        
        self.humiDataEntries.append(newHumiDataEntry)
        self.humiDS.removeAll()
        self.humiData.removeAll(keepingCapacity: false)
        self.humiDS = LineChartDataSet(entries: self.humiDataEntries, label: "Humidity")
        self.humiData.append(self.humiDS)
        self.humiChart.data = self.humiData
        
        self.humiDS.notifyDataSetChanged()
        self.humiData.notifyDataChanged()
        self.humiChart.notifyDataSetChanged()
        
        self.pressDataEntries.append(newPressDataEntry)
        self.pressDS.removeAll()
        self.pressData.removeAll(keepingCapacity: false)
        self.pressDS = LineChartDataSet(entries: self.pressDataEntries, label: "Pressure")
        self.pressData.append(self.pressDS)
        self.pressChart.data = self.pressData
        
        self.pressDS.notifyDataSetChanged()
        self.pressData.notifyDataChanged()
        self.pressChart.notifyDataSetChanged()
        
        
    }
    override var representedObject: Any? {
        didSet {
        // Update the view, if already loaded.
        }
    }
    func centralManagerDidUpdateState(_ central: CBCentralManager) {
        switch central.state {
            
        case CBManagerState.poweredOff:
            print("CoreBluetooth BLE hardware is powered off")
            break
        case CBManagerState.poweredOn:
            print("CoreBluetooth BLE hardware is powered on and ready")
            
           
            bleCentral.scanForPeripherals(withServices: nil, options: nil)
            //bleCentral.scanForPeripherals(withServices: [BluePyroPeripheral.BLUEPYRO_SERVICE_UUID], options: [CBCentralManagerScanOptionAllowDuplicatesKey : true])
            break
        case CBManagerState.resetting:
            print("CoreBluetooth BLE hardware is resetting")
            break
        case CBManagerState.unauthorized:
            print("CoreBluetooth BLE state is unauthorized")
            
            break
        case CBManagerState.unknown:
            print("CoreBluetooth BLE state is unknown")
            break
        case CBManagerState.unsupported:
            print("CoreBluetooth BLE hardware is unsupported on this platform")
            break
            
        @unknown default:
            print("CoreBluetooth BLE state is unknown")
            break
        }
    }
    func centralManager(_ central: CBCentralManager, didDiscover peripheral: CBPeripheral, advertisementData: [String : Any], rssi RSSI: NSNumber) {
        if let pname = peripheral.name {
            print(peripheral.name)
            if pname == "BlueIOThingy" {
                //self.bleCentral.stopScan()
                //mDeviceNameLabel.text = peripheral.name
                self.mBlueIODevice = peripheral
                self.mBlueIODevice.delegate = self
                //print(peripheral.identifier)
                
                
                
                if advertisementData[CBAdvertisementDataManufacturerDataKey] == nil {
                    return
                }
                
                var manId = UInt16(0)
                let manData = advertisementData[CBAdvertisementDataManufacturerDataKey] as! NSData
                
                if manData.length < 3 {
                    return
                }
              
                manData.getBytes(&manId, range: NSMakeRange(0, 2))
                if manId != 0x177 {
                    return
                }
              
                var type = UInt8(0)
                manData.getBytes(&type, range: NSMakeRange(2, 1))
                switch (type) {
                case 1:    // TPH sensor data
                    var press = Int32(0)
                    manData.getBytes(&press, range: NSMakeRange(3, 4))
                    pressLabel.stringValue = String(format:"%.3f KPa", Float(press) / 1000.0)
                
                    var temp = Int16(0)
                    manData.getBytes(&temp, range: NSMakeRange(7, 2))
                    tempLabel.stringValue = String(format:"%.2f C", Float(temp) / 100.0)
                    
                    var humi = UInt16(0)
                    manData.getBytes(&humi, range: NSMakeRange(9, 2))
                    humiLabel.stringValue = String(format:"%d%%", humi / 100)
                    
                    let newTempDataEntry = ChartDataEntry(x: Double(xValue),y: Double(Float(temp) / 100.0))
                    let newHumiDataEntry = ChartDataEntry(x: Double(xValue),y: Double(humi / 100))
                    let newPressDataEntry = ChartDataEntry(x: Double(xValue),y: Double(Float(press) / 1000.00))
                    UpdataCharts(with: newTempDataEntry, with: newHumiDataEntry, with: newPressDataEntry)
                    xValue += 1
                    break
                case 2: // Gas sensor data
                    var gasResistance = UInt32(0)
                    var gasAQI = UInt16(0)
                    manData.getBytes(&gasResistance, range: NSMakeRange(3, 4))
                    manData.getBytes(&gasAQI, range: NSMakeRange(7, 2))
                    aqiLabel.stringValue = String(format:"%d", gasAQI)
                    
                    print("Gas resistance value : \(gasResistance) AQI \(gasAQI)")
                    break
                case 10:
                    print("Motion detected")
                    break
                default:
                    break
                }
                rssiLabel.stringValue = String( describing: RSSI)
                //graphView.add(double3(Double(temp) / 100.0, Double(press) / 100000.0, Double(humi) / 100.0))
                
                
                    
            }
        }
    }
    func centralManager(_ central: CBCentralManager, didConnect peripheral: CBPeripheral) {
      
        peripheral.discoverServices([BlueIOPeripheral.BLUEIO_UUID_SERVICE])
        print("Connected to BlueIO peripheral")
        
        
    }
    func centralManager(_ central: CBCentralManager,
                        didDisconnectPeripheral peripheral: CBPeripheral,
                        error: Error?) {
        print("Disconnected from BlueIO peripheral")
        
        
    }
    
    func centralManager(_ central: CBCentralManager, didFailToConnect peripheral: CBPeripheral, error: Error?) {
        print("Device connect fail")
    }
    
    func scanPeripheral(_ sender: CBCentralManager)
    {
        print("Scan for peripherals")
        bleCentral.scanForPeripherals(withServices: nil, options: nil)
        //bleCentral.scanForPeripherals(withServices: [BlystHeaterPeripheral.BLUEPYRO_SERVICE_UUID], options: [CBCentralManagerScanOptionAllowDuplicatesKey : true])
    }
    // Handles discovery event
    func peripheral(_ peripheral: CBPeripheral, didDiscoverServices error: Error?) {
        if let services = peripheral.services {
            for service in services {
                if service.uuid == BlueIOPeripheral.BLUEIO_UUID_SERVICE {
                    print("BlueIO service found")
                    //Now kick off discovery of characteristics
                    peripheral.discoverCharacteristics([BlueIOPeripheral.BLUEIO_UUID_ENVIRONMENT,BlueIOPeripheral.BLUEIO_UUID_MOTION], for: service)
                    return
                }
            }
        }
    }
    // Handling discovery of characteristics
    func peripheral(_ peripheral: CBPeripheral, didDiscoverCharacteristicsFor service: CBService, error: Error?) {
        var mEnvChar: CBCharacteristic
        var mMotionChar: CBCharacteristic
        
        if let characteristics = service.characteristics {
            for characteristic in characteristics {
                if characteristic.uuid == BlueIOPeripheral.BLUEIO_UUID_ENVIRONMENT{
                    print("Environment service found")
                    mEnvChar = characteristic
                    
                    
                    
                } else if characteristic.uuid == BlueIOPeripheral.BLUEIO_UUID_MOTION{
                    print("Motion service found");
                    mMotionChar = characteristic
                    
                    
                }
                
            }
                       
        }
        DispatchQueue.main.asyncAfter(deadline: .now() + 2.0, execute: {
            
                
            self.bleCentral.cancelPeripheralConnection(peripheral)
                
            
        })
    }
    func peripheral(_ peripheral: CBPeripheral, didWriteValueFor characteristic: CBCharacteristic, error: Error?) {
        guard let data = characteristic.value else { return }
        print("\nValue: \(data) \nwas written to Characteristic:\n\(characteristic)")
        if(error != nil){
            print("\nError while writing on Characteristic:\n\(characteristic). Error Message:")
            print(error as Any)
        }
    }
    
    func peripheral(_ peripheral: CBPeripheral, didUpdateNotificationStateFor characteristic: CBCharacteristic, error: Error?){
        print("didUpdateNotificationStateFor")

        print("characteristic description:", characteristic.description)
    }
    
    func peripheral(_ peripheral: CBPeripheral, didUpdateValueFor characteristic: CBCharacteristic, error: Error?){
        //print("didUpdateValueFor")
        if let error = error {
            print("error:", error)
        }

        guard characteristic.value != nil else {
            return
        }

        
    }

}

