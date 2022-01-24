//
//  ViewController.swift
//  Magnetometer
//
//  Created by Tai on 2022-01-18.
//

import UIKit
import CoreBluetooth
import Charts

class BlueIOPeripheral: NSObject {

        public static let BLUEIO_UUID_SERVICE   = CBUUID.init(string: "ef680400-9b35-4933-9b10-52ffa9740042")
        public static let BLE_UUID_TMS_RAW_CHAR   = CBUUID.init(string: "ef680406-9b35-4933-9b10-52ffa9740042")
            
        
    }

class ViewController: UIViewController, CBCentralManagerDelegate, CBPeripheralDelegate {

    var bleCentral : CBCentralManager!
    var mBlueIODevice: CBPeripheral!
    var MagXDataEntries = [ChartDataEntry]()
    var MagYDataEntries = [ChartDataEntry]()
    var MagZDataEntries = [ChartDataEntry]()
    var xValue: Double = 1
    var counter: UInt32 = 0
    var MagXDS = LineChartDataSet()
    var MagYDS = LineChartDataSet()
    var MagZDS = LineChartDataSet()
    
    var MagXData = LineChartData()
    var MagYData = LineChartData()
    var MagZData = LineChartData()
    
    @IBOutlet weak var MagXChart: LineChartView!
    @IBOutlet weak var MagYChart: LineChartView!
    @IBOutlet weak var MagZChart: LineChartView!
    
    @IBOutlet weak var StopButton: UIButton!
    @IBOutlet weak var StartButton: UIButton!
    
    override func viewDidLoad() {
        super.viewDidLoad()
        // Do any additional setup after loading the view.
        bleCentral = CBCentralManager(delegate: self, queue: DispatchQueue.main)
        SetupChart()
    }

    func SetupChart(){
        self.MagXDS = LineChartDataSet(entries: self.MagXDataEntries, label: "MagX")
        self.MagXDS.colors = [NSUIColor.red]
        self.MagXDS.drawCirclesEnabled = false
        self.MagXData.addDataSet(self.MagXDS)
        self.MagXChart.data = self.MagXData
        
        self.MagYDS = LineChartDataSet(entries: self.MagYDataEntries, label: "MagY")
        self.MagYDS.colors = [NSUIColor.green]
        self.MagYDS.drawCirclesEnabled = false
        self.MagYData.addDataSet(self.MagYDS)
        self.MagYChart.data = self.MagYData
    
        
        self.MagZDS = LineChartDataSet(entries: self.MagZDataEntries, label: "MagZ")
        self.MagZDS.colors = [NSUIColor.blue]
        self.MagZDS.drawCirclesEnabled = false
        self.MagZData.addDataSet(self.MagZDS)
        self.MagZChart.data = self.MagZData
    }

    func UpdataCharts(with newMagXDataEntry: ChartDataEntry, with newMagYDataEntry: ChartDataEntry, with newMagZDataEntry: ChartDataEntry){
        self.MagXDataEntries.append(newMagXDataEntry)
        //self.MagXDS.removeAll()
        //self.MagXData.removeAll(keepingCapacity: false)
        self.MagXDS = LineChartDataSet(entries: self.MagXDataEntries, label: "MagX")
        self.MagXDS.drawCirclesEnabled = false
        self.MagXDS.drawValuesEnabled = false
        self.MagXDS.colors = [NSUIColor.red]
        self.MagXData.addEntry(newMagXDataEntry, dataSetIndex: 0)
        self.MagXChart.data = self.MagXData
        self.MagXDS.notifyDataSetChanged()
        self.MagXData.notifyDataChanged()
        self.MagXChart.notifyDataSetChanged()
        self.MagXChart.backgroundColor = NSUIColor.black
        self.MagXChart.legend.textColor = NSUIColor.white
        self.MagXChart.xAxis.labelTextColor = NSUIColor.white
        self.MagXChart.leftAxis.labelTextColor = NSUIColor.white
        self.MagXChart.rightAxis.labelTextColor = NSUIColor.white
        self.MagXDS.mode = .cubicBezier
        
        self.MagYDataEntries.append(newMagYDataEntry)
        //self.MagYDS.removeAll()
        //self.MagYData.removeAll(keepingCapacity: false)
        self.MagYDS = LineChartDataSet(entries: self.MagYDataEntries, label: "MagY")
        self.MagYDS.drawCirclesEnabled = false
        self.MagYDS.drawValuesEnabled = false
        self.MagYDS.colors = [NSUIColor.green]
        self.MagYData.addEntry(newMagYDataEntry, dataSetIndex: 0)
        self.MagYChart.data = self.MagYData
        self.MagYChart.backgroundColor = NSUIColor.black
        self.MagYChart.legend.textColor = NSUIColor.white
        self.MagYChart.xAxis.labelTextColor = NSUIColor.white
        self.MagYChart.leftAxis.labelTextColor = NSUIColor.white
        self.MagYChart.rightAxis.labelTextColor = NSUIColor.white
        self.MagYDS.mode = .cubicBezier
        self.MagYDS.notifyDataSetChanged()
        self.MagYData.notifyDataChanged()
        self.MagYChart.notifyDataSetChanged()
        
        
        self.MagZDataEntries.append(newMagZDataEntry)
        //self.MagZDS.removeAll()
        //self.MagZData.removeAll(keepingCapacity: false)
        self.MagZDS = LineChartDataSet(entries: self.MagZDataEntries, label: "MagZ")
        self.MagZDS.drawCirclesEnabled = false
        self.MagZDS.drawValuesEnabled = false
        self.MagZDS.colors = [NSUIColor.blue]
        self.MagZData.addEntry(newMagZDataEntry, dataSetIndex: 0)
        self.MagZChart.data = self.MagZData
        self.MagZChart.legend.textColor = NSUIColor.white
        self.MagZChart.xAxis.labelTextColor = NSUIColor.white
        self.MagZChart.leftAxis.labelTextColor = NSUIColor.white
        self.MagZChart.rightAxis.labelTextColor = NSUIColor.white
        self.MagZDS.mode = .cubicBezier
        self.MagZDS.notifyDataSetChanged()
        self.MagZData.notifyDataChanged()
        self.MagZChart.notifyDataSetChanged()
        self.MagZChart.backgroundColor = NSUIColor.black
        
        
    }
    @IBAction func StartButtonClick(_ sender: UIButton) {
        if self.mBlueIODevice != nil {
            print("Start click")
            self.bleCentral.connect(self.mBlueIODevice, options: nil)
        }
    }
    @IBAction func StopButtonClick(_ sender: UIButton) {
        if self.mBlueIODevice != nil {
            
            self.bleCentral.cancelPeripheralConnection(self.mBlueIODevice)
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
        //bleCentral.scanForPeripherals(withServices: [BlueIOPeripheral.BLUEPYRO_SERVICE_UUID], options: [CBCentralManagerScanOptionAllowDuplicatesKey : true])
    }
    // Handles discovery event
    func peripheral(_ peripheral: CBPeripheral, didDiscoverServices error: Error?) {
        if let services = peripheral.services {
            for service in services {
                if service.uuid == BlueIOPeripheral.BLUEIO_UUID_SERVICE {
                    print("BlueIO service found")
                    //Now kick off discovery of characteristics
                    peripheral.discoverCharacteristics([BlueIOPeripheral.BLE_UUID_TMS_RAW_CHAR], for: service)
                    return
                }
            }
        }
    }
    // Handling discovery of characteristics
    func peripheral(_ peripheral: CBPeripheral, didDiscoverCharacteristicsFor service: CBService, error: Error?) {
        var mRawChar: CBCharacteristic
       
        if let characteristics = service.characteristics {
            for characteristic in characteristics {
                if characteristic.uuid == BlueIOPeripheral.BLE_UUID_TMS_RAW_CHAR{
                    print("TMS Raw service found")
                    mRawChar = characteristic
                    
                    peripheral.setNotifyValue(true, for: mRawChar)
                    
                }
              
                              
            }
                       
        }
        
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
        
        
        //print("characteristic description:", characteristic.description)
        let data = characteristic.value
        //print(data)
        
        let magXdata = data![12...13]
        let magYdata = data![14...15]
        let magZdata = data![16...17]
        let magX_array = UInt16(bigEndian: magXdata.withUnsafeBytes { $0.pointee })
        let magX = UInt16(bigEndian: magX_array)
        //print(magX)
        let magY_array = UInt16(bigEndian: magYdata.withUnsafeBytes { $0.pointee })
        let magY = UInt16(bigEndian: magY_array)
        //print(magY)
        let magZ_array = UInt16(bigEndian: magZdata.withUnsafeBytes { $0.pointee })
        let magZ = UInt16(bigEndian: magZ_array)
        //print(magZ)
        
        let newMagXDataEntry = ChartDataEntry(x: Double(xValue),y: Double(magX))
        let newMagYDataEntry = ChartDataEntry(x: Double(xValue),y: Double(magY))
        let newMagZDataEntry = ChartDataEntry(x: Double(xValue),y: Double(magZ))
        counter += 1
        if (counter % 20 == 0){
            UpdataCharts(with: newMagXDataEntry, with: newMagYDataEntry, with: newMagZDataEntry)
            xValue += 1
            print(xValue)
        }
        
    }
    
    
}

