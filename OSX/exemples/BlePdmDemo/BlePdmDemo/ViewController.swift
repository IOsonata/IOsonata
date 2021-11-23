//
//  ViewController.swift
//  BlePdmDemo
//
//  Created by Tai on 2021-11-22.
//

import Cocoa
import CoreBluetooth


class BlePdmPeripheral: NSObject {

        public static let BLEPDM_SERVICE_UUID   = CBUUID.init(string: "00000100-3462-475C-96E9-58C72CFF09AA")
        public static let BLEPDM_WRITECHAR_UUID   = CBUUID.init(string: "00000200-3462-475C-96E9-58C72CFF09AA")
        public static let BLEPDM_READCHAR_UUID   = CBUUID.init(string: "00000101-3462-475C-96E9-58C72CFF09AA")
        
    }

class MyStreamer : NSObject{
    lazy var fileHandle: FileHandle? = {
        let fileHandle = FileHandle(forWritingAtPath: self.logPath)
        return fileHandle
    }()

    lazy var logPath: String = {
        let downloadsDirectory = FileManager.default.urls(for: .downloadsDirectory, in: .userDomainMask).first!
        let downloadsDirectoryWithFolder = downloadsDirectory.appendingPathComponent("BlePdmDemo")

        do {
            try FileManager.default.createDirectory(at: downloadsDirectoryWithFolder, withIntermediateDirectories: true, attributes: nil)
        } catch let error as NSError {
            print(error.localizedDescription)
        }
        let path : NSString = NSString(string: downloadsDirectoryWithFolder.path)
        
        let date = Date()
        let format = DateFormatter()
        format.dateFormat = "yyyy-MM-dd-HH-mm-ss"
        let timestamp = format.string(from: date)
        
        let filePath = (path as NSString).appendingPathComponent("record_" + timestamp + ".pcm")

        if !FileManager.default.fileExists(atPath: filePath) {
            FileManager.default.createFile(atPath: filePath, contents: nil, attributes: nil)
        }
        print(filePath)
        return filePath

    }()

    func write(data: Data) {
        //fileHandle?.seekToEndOfFile()
        fileHandle?.write(data)
    }
}
class ViewController: NSViewController, CBCentralManagerDelegate, CBPeripheralDelegate  {

    var bleCentral : CBCentralManager!
    var mBlePdmDevice: CBPeripheral!
    
    @IBOutlet weak var StatusLabel: NSTextField!
    
    @IBOutlet weak var StartButton: NSButton!
    @IBOutlet weak var StopButton: NSButton!
    
    @IBOutlet weak var waveformView: NSView!
    
    @IBOutlet weak var PktDropCounterLabel: NSTextField!
    @IBOutlet weak var PktCounterLabel: NSTextField!
    @IBOutlet weak var AudioModeComboBox: NSComboBox!
    @IBOutlet weak var UpdateButton: NSButton!
    
    
    lazy var pencil = NSBezierPath(rect: waveformView.bounds)
    lazy var firstPoint = CGPoint(x: 6, y: (waveformView.bounds.midY))
    
    lazy var jump : CGFloat = (waveformView.bounds.width - (firstPoint.x * 2))/500
    let waveLayer = CAShapeLayer()
    var traitLength : CGFloat!
    var start : CGPoint!
    var counter = UInt32(0)
    var pkt_drop_counter = Int32(0)
    var current_pkt = UInt32(0)
    var previous_pkt = UInt32(0)
    var myStream = MyStreamer()
    
    func writeWave( _ input : Float, _ bool : Bool){
        if !bool {
            start = firstPoint
            return
        } else {
            
            traitLength = (CGFloat(input))
            pencil.lineWidth = jump
            pencil.move(to: start)
            pencil.line(to: CGPoint(x: start.x, y: start.y + traitLength))
            pencil.move(to: start)
            pencil.line(to: CGPoint(x: start.x, y: start.y - traitLength))
            
            waveLayer.strokeColor = NSColor.black.cgColor
            waveLayer.path = pencil.cgPath
            waveLayer.fillColor = NSColor.clear.cgColor
            waveLayer.lineWidth = jump
            waveLayer.borderColor = CGColor.black
            waveLayer.borderWidth = 1
            
            waveformView.layer?.addSublayer(waveLayer)
            waveLayer.contentsCenter = waveformView.frame
            waveformView.needsDisplay = true
            
            start = CGPoint(x: start.x + jump, y: start.y)
        }
    }
    override func viewDidLoad() {
        super.viewDidLoad()

        // Do any additional setup after loading the view.
        bleCentral = CBCentralManager(delegate: self, queue: DispatchQueue.main)
        pencil.removeAllPoints()
        waveLayer.removeFromSuperlayer()
        writeWave(0, false)
        
        AudioModeComboBox.selectItem(withObjectValue: "MONO")
        
        
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
            if pname == "Badger" {
                //self.bleCentral.stopScan()
                //mDeviceNameLabel.text = peripheral.name
                self.mBlePdmDevice = peripheral
                self.mBlePdmDevice.delegate = self
                //print(peripheral.identifier)
                
                
                print(self.mBlePdmDevice.identifier)
                print(self.mBlePdmDevice.name as Any)
                //self.bleCentral.connect(self.mBlePdmDevice, options: nil)
                self.StatusLabel.stringValue = "Status: CONNECTED"
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
              
                
                
                
                    
            }
        }
    }
    func centralManager(_ central: CBCentralManager, didConnect peripheral: CBPeripheral) {
      
        peripheral.discoverServices([BlePdmPeripheral.BLEPDM_SERVICE_UUID])
        print("Connected to BlePdm peripheral")
        
    }
    func centralManager(_ central: CBCentralManager,
                        didDisconnectPeripheral peripheral: CBPeripheral,
                        error: Error?) {
        print("Disconnected from BlePdm peripheral")
        
        
    }
    
    func centralManager(_ central: CBCentralManager, didFailToConnect peripheral: CBPeripheral, error: Error?) {
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
                if service.uuid == BlePdmPeripheral.BLEPDM_SERVICE_UUID {
                    print("BlePdm service found")
                    //Now kick off discovery of characteristics
                    peripheral.discoverCharacteristics([BlePdmPeripheral.BLEPDM_READCHAR_UUID,BlePdmPeripheral.BLEPDM_WRITECHAR_UUID], for: service)
                    return
                }
            }
        }
    }
    // Handling discovery of characteristics
    func peripheral(_ peripheral: CBPeripheral, didDiscoverCharacteristicsFor service: CBService, error: Error?) {
        var mReadChar: CBCharacteristic
        var mWriteChar: CBCharacteristic
        
        if let characteristics = service.characteristics {
            for characteristic in characteristics {
                if characteristic.uuid == BlePdmPeripheral.BLEPDM_READCHAR_UUID{
                    print("Read characteristic found")
                    mReadChar = characteristic
                    peripheral.setNotifyValue(true, for: mReadChar)
                    
                } else if characteristic.uuid == BlePdmPeripheral.BLEPDM_WRITECHAR_UUID{
                    print("Write characteristic found");
                    mWriteChar = characteristic
                    
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
        let counter_data = data![0...4]
        
        let counter_array = UInt32(bigEndian: counter_data.withUnsafeBytes { $0.pointee })
        //print(counter_array)
        let pkt_counter = UInt32(bigEndian: counter_array)
        PktCounterLabel.stringValue = String(pkt_counter)
        current_pkt = pkt_counter
        if (current_pkt >= previous_pkt) {
            let drop_pkt = Int32(current_pkt - previous_pkt)
            if ( drop_pkt > 1) {
                pkt_drop_counter += Int32(drop_pkt)
            }
            PktDropCounterLabel.stringValue = String(pkt_drop_counter)
            previous_pkt = pkt_counter
        }
        let audio_data = data![4...data!.count-1]
        if (previous_pkt > 20) {
            myStream.write(data: audio_data)
        }
        var max_audio_value = UInt8(0)
        for i in audio_data{
            if (i > max_audio_value) && (i < 80) {
                max_audio_value = i
            }
            
        }
        max_audio_value += 1
        print(max_audio_value)
        //print(pkt_counter)
        
        //let randomFloat = Float.random(in: 1..<255)
        if counter % 500 == 0{
            pencil.removeAllPoints()
            waveLayer.removeFromSuperlayer()
            writeWave(0, false)
            
        } else {
            writeWave(Float(max_audio_value), true)
        }
        counter += 1
    }
    
    func convertByteArrayToInt2(_ bytes: [UInt8]) -> UInt32 {
        let value = ((bytes[3] & 0xFF) << 24) | ((bytes[2] & 0xFF) << 16) | ((bytes[1] & 0xFF) << 8) | ((bytes[0] & 0xFF) << 0)
        return UInt32(value)
    }
    
    @IBAction func StopButtonClick(_ sender: Any) {
        if self.mBlePdmDevice != nil {
            self.bleCentral.cancelPeripheralConnection(self.mBlePdmDevice)
            self.StatusLabel.stringValue = "Status: DISCONNECTED"
            self.counter = UInt32(0)
            self.pkt_drop_counter = Int32(0)
            self.current_pkt = UInt32(0)
            self.previous_pkt = UInt32(0)
        } else {
            self.StatusLabel.stringValue = "Status: NO_DEVICE"
        }
    }
    
    @IBAction func StartButtonClick(_ sender: Any) {
        if self.mBlePdmDevice != nil {
            self.bleCentral.connect(self.mBlePdmDevice, options: nil)
            self.StatusLabel.stringValue = "Status: CONNECTED"
            self.myStream = MyStreamer()
        } else {
            self.StatusLabel.stringValue = "Status: NO_DEVICE"
        }
        
    }
    
    
}

extension NSBezierPath {
    
    /// A `CGPath` object representing the current `NSBezierPath`.
    var cgPath: CGPath {
        let path = CGMutablePath()
        let points = UnsafeMutablePointer<NSPoint>.allocate(capacity: 3)

        if elementCount > 0 {
            var didClosePath = true

            for index in 0..<elementCount {
                let pathType = element(at: index, associatedPoints: points)

                switch pathType {
                case .moveTo:
                    path.move(to: points[0])
                case .lineTo:
                    path.addLine(to: points[0])
                    didClosePath = false
                case .curveTo:
                    path.addCurve(to: points[2], control1: points[0], control2: points[1])
                    didClosePath = false
                case .closePath:
                    path.closeSubpath()
                    didClosePath = true
                @unknown default:
                    break
                }
            }

            if !didClosePath { path.closeSubpath() }
        }

        points.deallocate()
        return path
    }
}

